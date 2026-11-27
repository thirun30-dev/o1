#!/usr/bin/env python3
# Copyright 2026 dk
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Optimus AMR System Manager Node.

Coordinates the state machine, health monitoring, process supervision, map
management, and emergency stop safety protocols for the Optimus autonomous
mobile robot.
"""

from enum import Enum
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from optimus_bot.srv import MapOperation, SetMode
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, JointState, LaserScan, PointCloud2
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros


class AMRState(Enum):
    OFF = 'OFF'
    BOOTING = 'BOOTING'
    READY = 'READY'
    MAPPING = 'MAPPING'
    MAP_SAVING = 'MAP_SAVING'
    LOCALIZATION = 'LOCALIZATION'
    NAVIGATION = 'NAVIGATION'
    ERROR = 'ERROR'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


class HealthStatus:
    OK = 'OK'
    WARNING = 'WARNING'
    ERROR = 'ERROR'


class AMRStateMachine:
    VALID_TRANSITIONS = {
        AMRState.OFF: [AMRState.BOOTING, AMRState.READY, AMRState.MAPPING,
                       AMRState.LOCALIZATION, AMRState.EMERGENCY_STOP],
        AMRState.BOOTING: [AMRState.READY, AMRState.ERROR, AMRState.EMERGENCY_STOP, AMRState.OFF],
        AMRState.READY: [AMRState.MAPPING, AMRState.LOCALIZATION, AMRState.NAVIGATION,
                         AMRState.OFF, AMRState.ERROR, AMRState.EMERGENCY_STOP],
        AMRState.MAPPING: [AMRState.MAP_SAVING, AMRState.READY, AMRState.OFF,
                           AMRState.ERROR, AMRState.EMERGENCY_STOP],
        AMRState.MAP_SAVING: [AMRState.MAPPING, AMRState.READY, AMRState.OFF,
                              AMRState.ERROR, AMRState.EMERGENCY_STOP],
        AMRState.LOCALIZATION: [AMRState.NAVIGATION, AMRState.MAPPING, AMRState.READY,
                                AMRState.OFF, AMRState.ERROR, AMRState.EMERGENCY_STOP],
        AMRState.NAVIGATION: [AMRState.LOCALIZATION, AMRState.MAPPING, AMRState.READY,
                              AMRState.OFF, AMRState.ERROR, AMRState.EMERGENCY_STOP],
        AMRState.ERROR: [AMRState.READY, AMRState.OFF, AMRState.EMERGENCY_STOP],
        AMRState.EMERGENCY_STOP: [AMRState.READY, AMRState.OFF],
    }

    def __init__(self, initial_state: AMRState = AMRState.OFF):
        self._current_state = initial_state
        self._previous_state = initial_state
        self._error_reason = ''
        self._active_map = ''

    @property
    def current_state(self) -> AMRState:
        return self._current_state

    @property
    def previous_state(self) -> AMRState:
        return self._previous_state

    @property
    def error_reason(self) -> str:
        return self._error_reason

    @property
    def active_map(self) -> str:
        return self._active_map

    @active_map.setter
    def active_map(self, map_name: str):
        self._active_map = map_name

    def can_transition_to(self, new_state: AMRState) -> bool:
        return new_state in self.VALID_TRANSITIONS.get(self._current_state, [])

    def transition_to(self, new_state: AMRState, reason: str = '') -> tuple[bool, str]:
        if new_state == self._current_state:
            return True, f'Already in state {self._current_state.value}'

        if not self.can_transition_to(new_state):
            allowed = [st.value for st in self.VALID_TRANSITIONS.get(self._current_state, [])]
            msg = (f'Invalid transition from {self._current_state.value} to {new_state.value}. '
                   f'Allowed targets: {allowed}')
            return False, msg

        self._previous_state = self._current_state
        self._current_state = new_state
        if new_state == AMRState.ERROR:
            self._error_reason = reason
        elif new_state in (AMRState.READY, AMRState.OFF):
            self._error_reason = ''

        return True, (f'Transitioned from {self._previous_state.value} '
                      f'to {self._current_state.value}')

    def trigger_emergency_stop(
            self, reason: str = 'Operator Triggered Emergency Stop') -> tuple[bool, str]:
        self._previous_state = self._current_state
        self._current_state = AMRState.EMERGENCY_STOP
        self._error_reason = reason
        return True, 'Emergency Stop Activated'

    def reset_emergency_stop(self) -> tuple[bool, str]:
        if self._current_state != AMRState.EMERGENCY_STOP:
            return False, 'Not in EMERGENCY_STOP state'
        self._previous_state = AMRState.EMERGENCY_STOP
        self._current_state = AMRState.READY
        self._error_reason = ''
        return True, 'Emergency Stop Reset. System is READY'


class AMRProcessSupervisor:
    def __init__(self, node: Node, use_sim_time: bool = True,
                 map_frame: str = 'map', odom_frame: str = 'odom',
                 base_frame: str = 'base_link', lidar_frame: str = 'lidar_link'):
        self.node = node
        self.use_sim_time = use_sim_time
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.lidar_frame = lidar_frame
        self.processes: dict[str, subprocess.Popen] = {}

    def is_running(self, name: str) -> bool:
        proc = self.processes.get(name)
        if proc is None:
            return False
        return proc.poll() is None

    def _wait_for_nodes(self, process_name: str, node_names: list[str],
                        timeout: float = 10.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout
        wanted = set(node_names)
        while time.monotonic() < deadline:
            proc = self.processes.get(process_name)
            if proc is None or proc.poll() is not None:
                return False, f"Managed process '{process_name}' exited while starting."
            active = set(self.node.get_node_names())
            if wanted.issubset(active):
                return True, f'{process_name} nodes are ready.'
            time.sleep(0.2)
        missing = sorted(wanted - set(self.node.get_node_names()))
        return False, f"Timed out waiting for {process_name} nodes: {', '.join(missing)}."

    def start_slam(self) -> tuple[bool, str]:
        if self.is_running('slam'):
            return True, 'SLAM Toolbox is already running'

        self.stop_navigation()
        self.stop_localization()

        cmd = [
            'ros2', 'launch', 'optimus_bot', 'mapping.launch.py',
            f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
            f'map_frame:={self.map_frame}', f'odom_frame:={self.odom_frame}',
            f'base_frame:={self.base_frame}', 'scan_topic:=/scan'
        ]
        self.node.get_logger().info(f"Starting SLAM: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                # Inherit the unified bringup terminal.  Feature launches
                # must keep their hardware, lifecycle, and TF diagnostics
                # visible to the operator; piping without a reader can also
                # deadlock a long-running ros2 launch process.
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid
            )
            self.processes['slam'] = proc
            ready, ready_msg = self._wait_for_nodes('slam', ['slam_toolbox'])
            if not ready:
                self._stop_process('slam')
                return False, ready_msg
            return True, 'SLAM Toolbox process launched and ready'
        except Exception as e:
            return False, f'Failed to start SLAM: {str(e)}'

    def stop_slam(self) -> tuple[bool, str]:
        ok, msg = self._stop_process('slam')
        for pattern in ['async_slam_toolbox_node', 'sync_slam_toolbox_node']:
            try:
                subprocess.run(['pkill', '-f', pattern], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        return ok, msg

    def start_localization(self, map_yaml_path: str) -> tuple[bool, str]:
        if not os.path.exists(map_yaml_path):
            return False, f'Map file does not exist: {map_yaml_path}'

        self.stop_slam()
        if self.is_running('localization'):
            self.stop_localization()

        cmd = [
            'ros2', 'launch', 'optimus_bot', 'localization.launch.py',
            f'map:={map_yaml_path}',
            f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
            f'map_frame:={self.map_frame}', f'odom_frame:={self.odom_frame}',
            f'base_frame:={self.base_frame}', 'scan_topic:=/scan'
        ]
        self.node.get_logger().info(f"Starting Localization: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid
            )
            self.processes['localization'] = proc
            ready, ready_msg = self._wait_for_nodes(
                'localization', ['map_server', 'amcl'], timeout=20.0)
            if not ready:
                self.stop_localization()
                return False, ready_msg
            return True, 'Localization (Map Server + AMCL) process launched and ready'
        except Exception as e:
            return False, f'Failed to start localization: {str(e)}'

    def stop_localization(self) -> tuple[bool, str]:
        self.stop_navigation()
        ok, msg = self._stop_process('localization')
        for pattern in ['lifecycle_manager_localization', 'nav2_amcl', 'nav2_map_server']:
            try:
                subprocess.run(['pkill', '-f', pattern], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        return ok, msg

    def start_navigation(self) -> tuple[bool, str]:
        if self.is_running('navigation'):
            return True, 'Nav2 navigation stack is already running'

        if not self.is_running('localization'):
            return False, 'Cannot start navigation without active localization'

        # Do not start Nav2 while AMCL is alive but still unlocalized. In
        # that state map->odom is unavailable/unstable and every goal is
        # likely to fail. The operator must set the initial pose first.
        if (not self.node.amcl_pose_received or
                (time.time() - self.node.last_amcl_time) > 5.0):
            return False, ('Cannot start Nav2: AMCL has no recent pose. '
                           'Set the initial pose and wait for AMCL to converge.')

        cmd = [
            'ros2', 'launch', 'optimus_bot', 'navigation.launch.py',
            f"use_sim_time:={'true' if self.use_sim_time else 'false'}"
        ]
        self.node.get_logger().info(f"Starting Navigation: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid
            )
            self.processes['navigation'] = proc
            ready, ready_msg = self._wait_for_nodes(
                'navigation',
                [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'lifecycle_manager_navigation',
                ],
                timeout=25.0)
            if not ready:
                self.stop_navigation()
                return False, ready_msg
            return True, 'Nav2 navigation stack process launched and ready'
        except Exception as e:
            return False, f'Failed to start navigation: {str(e)}'

    def stop_navigation(self) -> tuple[bool, str]:
        ok, msg = self._stop_process('navigation')
        for pattern in ['lifecycle_manager_navigation', 'controller_server', 'smoother_server',
                        'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower',
                        'velocity_smoother']:
            try:
                subprocess.run(['pkill', '-f', pattern], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        return ok, msg

    def stop_all(self):
        for name in list(self.processes.keys()):
            self._stop_process(name)

    def _stop_process(self, name: str) -> tuple[bool, str]:
        proc = self.processes.get(name)
        if proc is None:
            return True, f'Process {name} not running'

        if proc.poll() is not None:
            del self.processes[name]
            return True, f'Process {name} already terminated'

        self.node.get_logger().info(f'Stopping process {name} (PID {proc.pid})')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            for _ in range(30):
                if proc.poll() is not None:
                    break
                time.sleep(0.1)

            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                time.sleep(0.5)

            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as e:
            self.node.get_logger().warn(f'Error terminating {name}: {e}')

        if name in self.processes:
            del self.processes[name]
        return True, f'Process {name} stopped successfully'


class AMRSystemManager(Node):
    VALID_MAP_PATTERN = re.compile(r'^[a-zA-Z0-9_\-]+$')

    def __init__(self):
        super().__init__('amr_system_manager')

        for param_name, default_val in [
            ('use_sim_time', True),
            ('maps_dir', str(Path.home() / 'optimus_data' / 'maps')),
            ('auto_ready', True),
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('lidar_frame', 'lidar_link'),
        ]:
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_val)

        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.maps_dir = self.get_parameter('maps_dir').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        auto_ready = self.get_parameter('auto_ready').get_parameter_value().bool_value

        os.makedirs(self.maps_dir, exist_ok=True)
        self.pkg_maps_dir = os.path.join(
            get_package_share_directory('optimus_bot'), 'maps'
        )

        self.get_logger().info(
            f'AMR System Manager starting (use_sim_time={self.use_sim_time}, '
            f'maps_dir={self.maps_dir})')

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Components
        self.state_machine = AMRStateMachine(AMRState.OFF)
        self.supervisor = AMRProcessSupervisor(
            self, use_sim_time=self.use_sim_time,
            map_frame=self.map_frame, odom_frame=self.odom_frame,
            base_frame=self.base_frame, lidar_frame=self.lidar_frame)

        # Telemetry tracking
        self.last_scan_time = 0.0
        self.last_camera_time = 0.0
        self.last_depth_time = 0.0
        self.last_odom_time = 0.0
        self.last_joint_states_time = 0.0
        self.last_map_time = 0.0
        self.map_received = False
        self.amcl_pose_received = False
        self.last_amcl_time = 0.0
        self.auto_nav_pending = False
        self.battery_pct = 95.0
        self.battery_voltage = 25.4

        # Subscriptions
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Optional obstacle cameras (camera:=..., depth_camera:=...).
        self.sub_camera = self.create_subscription(
            LaserScan, '/camera/scan', self._camera_cb, 5)
        self.sub_depth = self.create_subscription(
            PointCloud2, '/depth_camera/obstacles', self._depth_cb, 5)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, rclpy.qos.qos_profile_sensor_data)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.cmd_sub = self.create_subscription(String, '/amr/command', self._command_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/amr/system_state', 10)
        self.health_pub = self.create_publisher(String, '/amr/system_health', 10)
        self.maps_pub = self.create_publisher(String, '/amr/maps', 10)
        operation_qos = QoSProfile(depth=10)
        operation_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.operation_pub = self.create_publisher(
            String, '/amr/map_operation_result', operation_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/amr/battery_state', 10)
        self.serialize_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.deserialize_client = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map')

        # Standard Trigger Services for ROS and Dashboard
        self.srv_start_robot = self.create_service(
            Trigger, '/amr/start_robot', self._srv_start_robot)
        self.srv_stop_robot = self.create_service(Trigger, '/amr/stop_robot', self._srv_stop_robot)
        self.srv_start_mapping = self.create_service(
            Trigger, '/amr/start_mapping', self._srv_start_mapping)
        self.srv_stop_mapping = self.create_service(
            Trigger, '/amr/stop_mapping', self._srv_stop_mapping)
        self.srv_start_localization = self.create_service(
            Trigger, '/amr/start_localization', self._srv_start_localization)
        self.srv_stop_localization = self.create_service(
            Trigger, '/amr/stop_localization', self._srv_stop_localization)
        self.srv_start_navigation = self.create_service(
            Trigger, '/amr/start_navigation', self._srv_start_navigation)
        self.srv_stop_navigation = self.create_service(
            Trigger, '/amr/stop_navigation', self._srv_stop_navigation)
        self.srv_estop = self.create_service(
            Trigger, '/amr/emergency_stop', self._srv_emergency_stop)
        self.srv_reset_estop = self.create_service(
            Trigger, '/amr/reset_emergency_stop', self._srv_reset_emergency_stop)
        self.srv_system_reset = self.create_service(
            Trigger, '/amr/system_reset', self._srv_system_reset)
        self.srv_map_operation = self.create_service(
            MapOperation, '/amr/map_operation', self._srv_map_operation)
        self.srv_set_mode = self.create_service(SetMode, '/amr/set_mode', self._srv_set_mode)

        # Timers
        self.state_timer = self.create_timer(0.2, self._publish_state)
        self.health_timer = self.create_timer(0.5, self._publish_health)
        self.maps_timer = self.create_timer(2.0, self._publish_maps)
        self.battery_timer = self.create_timer(1.0, self._publish_battery)
        self.estop_watchdog = self.create_timer(0.05, self._estop_safety_loop)
        self.process_monitor_timer = self.create_timer(1.0, self._monitor_processes)

        # Initialize to READY
        if auto_ready:
            self.state_machine.transition_to(AMRState.BOOTING)
            self.state_machine.transition_to(AMRState.READY)
            self.get_logger().info('System auto-transitioned to READY')

    def _camera_cb(self, _msg):
        self.last_camera_time = time.time()

    def _depth_cb(self, _msg):
        self.last_depth_time = time.time()

    def _scan_cb(self, msg: LaserScan):
        self.last_scan_time = time.time()

    def _odom_cb(self, msg: Odometry):
        self.last_odom_time = time.time()

    def _joint_cb(self, msg: JointState):
        self.last_joint_states_time = time.time()

    def _map_cb(self, msg: OccupancyGrid):
        self.last_map_time = time.time()
        self.map_received = True

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_time = time.time()
        self.amcl_pose_received = True
        # Once AMCL has accepted an initial pose, automatically bring up
        # Nav2 when the operator is in localization mode. The actual launch
        # is performed by _monitor_processes so this subscription callback
        # stays short and responsive.
        if (self.state_machine.current_state == AMRState.LOCALIZATION and
                not self.supervisor.is_running('navigation')):
            self.auto_nav_pending = True

    def _estop_safety_loop(self):
        if self.state_machine.current_state == AMRState.EMERGENCY_STOP:
            self.cmd_vel_pub.publish(Twist())

    def _monitor_processes(self):
        """Convert unexpected feature-process exits into a visible ERROR state."""
        if (self.auto_nav_pending and
                self.state_machine.current_state == AMRState.LOCALIZATION and
                self.amcl_pose_received and
                (time.time() - self.last_amcl_time) <= 5.0):
            self.auto_nav_pending = False
            self.get_logger().info('AMCL pose received; starting Nav2 automatically')
            ok, message = self.supervisor.start_navigation()
            if ok:
                self.state_machine.transition_to(AMRState.NAVIGATION)
                self.get_logger().info(message)
            else:
                self.get_logger().error(f'Automatic Nav2 startup failed: {message}')
                self._publish_operation('start_navigation', False, message)

        for name, proc in list(self.supervisor.processes.items()):
            return_code = proc.poll()
            if return_code is None:
                continue
            del self.supervisor.processes[name]
            message = f"Managed process '{name}' exited unexpectedly with code {return_code}."
            self.get_logger().error(message)
            if self.state_machine.current_state in (
                    AMRState.MAPPING, AMRState.LOCALIZATION, AMRState.NAVIGATION):
                self.state_machine.transition_to(AMRState.ERROR, message)

    def check_tf(self, target_frame: str, source_frame: str) -> tuple[bool, str]:
        try:
            can_tf = self.tf_buffer.can_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.1))
            if can_tf:
                return True, f'Transform {source_frame} -> {target_frame} OK'
            return False, f'Transform {source_frame} -> {target_frame} unavailable in TF tree'
        except Exception as e:
            return False, f'Transform {source_frame} -> {target_frame} error: {str(e)}'

    def _validate_preflight_mapping(self) -> tuple[bool, str]:
        now = time.time()
        if self.last_scan_time == 0 or (now - self.last_scan_time) > 2.0:
            return False, 'Pre-flight failed: LiDAR topic /scan is not actively publishing'
        if self.last_odom_time == 0 or (now - self.last_odom_time) > 2.0:
            return False, 'Pre-flight failed: Odometry topic /odom is not actively publishing'
        ok_tf, tf_msg = self.check_tf(self.odom_frame, self.base_frame)
        if not ok_tf:
            return False, f'Pre-flight failed: {tf_msg}'
        ok_lidar, lidar_msg = self.check_tf(self.base_frame, self.lidar_frame)
        if not ok_lidar:
            return False, f'Pre-flight failed: {lidar_msg}'
        return True, 'All mapping prerequisites satisfied.'

    def list_maps(self) -> list[str]:
        maps = set()
        for directory in [self.maps_dir, self.pkg_maps_dir]:
            if os.path.exists(directory):
                for f in os.listdir(directory):
                    if f.endswith('.yaml') and not f.startswith('.'):
                        maps.add(f[:-5])
        return sorted(maps)

    def get_map_path(self, map_name: str) -> str | None:
        name = map_name.strip()
        if not self.VALID_MAP_PATTERN.match(name):
            return None
        p1 = os.path.join(self.maps_dir, f'{name}.yaml')
        if os.path.exists(p1):
            return p1
        p2 = os.path.join(self.pkg_maps_dir, f'{name}.yaml')
        if os.path.exists(p2):
            return p2
        return None

    def save_map(self, map_name: str) -> tuple[bool, str]:
        name = map_name.strip()
        if not name or not self.VALID_MAP_PATTERN.match(name):
            return False, ('Invalid map name. Only letters, numbers, underscores, '
                           'and hyphens are allowed.')

        target_base = os.path.join(self.maps_dir, name)
        self.get_logger().info(f'Saving map to target: {target_base}')
        try:
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', target_base,
                '--ros-args', '-p', f'use_sim_time:={str(self.use_sim_time).lower()}',
                '-p', 'save_map_timeout:=10.0'
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
            if proc.returncode == 0:
                return True, f"Map '{name}' saved successfully"
            else:
                err_msg = (proc.stderr.strip() or proc.stdout.strip()
                           or 'map_saver_cli returned non-zero code')
                return False, f'Map saving failed: {err_msg}'
        except subprocess.TimeoutExpired:
            return False, 'Map saving timed out after 20s. Verify /map topic is active.'
        except Exception as e:
            return False, f'Error saving map: {str(e)}'

    def _publish_operation(self, operation: str, success: bool, message: str, **extra):
        payload = {'operation': operation, 'success': success, 'message': message,
                   'timestamp': time.time(), **extra}
        msg = String()
        msg.data = json.dumps(payload)
        self.operation_pub.publish(msg)

    def serialize_pose_graph(self, map_name: str) -> tuple[bool, str]:
        """Ask async SLAM Toolbox to persist its graph beside the occupancy map."""
        name = map_name.strip()
        if not name or not self.VALID_MAP_PATTERN.match(name):
            return False, ('Invalid map name. Only letters, numbers, underscores, '
                           'and hyphens are allowed.')
        if not self.supervisor.is_running('slam'):
            return False, ('SLAM Toolbox is not running; '
                           'start mapping before serializing the graph.')
        if not self.serialize_client.wait_for_service(timeout_sec=10.0):
            return False, 'SLAM Toolbox serialize service is unavailable.'
        filename = os.path.join(self.maps_dir, name)
        request = SerializePoseGraph.Request()
        request.filename = filename
        future = self.serialize_client.call_async(request)
        future.add_done_callback(lambda f: self._serialize_done(f, name, filename))
        return True, f"Pose-graph serialization requested for '{name}'."

    def _serialize_done(self, future, name: str, filename: str):
        try:
            result = future.result()
            success = result.result == SerializePoseGraph.Response.RESULT_SUCCESS
            message = (f"Pose graph '{name}' serialized successfully."
                       if success
                       else f'Pose graph serialization failed (result {result.result}).')
        except Exception as exc:
            success = False
            message = f'Pose graph serialization failed: {exc}'
        self._publish_operation('serialize_pose_graph', success, message,
                                map_name=name, filename=filename)

    def deserialize_pose_graph(self, map_name: str) -> tuple[bool, str]:
        name = map_name.strip()
        if not name or not self.VALID_MAP_PATTERN.match(name):
            return False, 'Invalid map name.'
        filename = os.path.join(self.maps_dir, name)
        if not os.path.exists(filename + '.posegraph'):
            return False, f'Serialized pose graph not found: {filename}.posegraph'
        if not self.supervisor.is_running('slam'):
            return False, 'SLAM Toolbox is not running; start mapping before deserializing.'
        if not self.deserialize_client.wait_for_service(timeout_sec=10.0):
            return False, 'SLAM Toolbox deserialize service is unavailable.'
        request = DeserializePoseGraph.Request()
        request.filename = filename
        request.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        future = self.deserialize_client.call_async(request)
        future.add_done_callback(lambda f: self._deserialize_done(f, name, filename))
        return True, f"Pose-graph deserialization requested for '{name}'."

    def _deserialize_done(self, future, name: str, filename: str):
        try:
            # DeserializePoseGraph has an empty response (no result code);
            # reading result.result raised AttributeError, so every load was
            # reported as failed. A completed call is the success signal.
            future.result()
            success = True
            message = f"Pose graph '{name}' loaded successfully."
        except Exception as exc:
            success = False
            message = f'Pose graph deserialization failed: {exc}'
        self._publish_operation('deserialize_pose_graph', success, message,
                                map_name=name, filename=filename)

    def _publish_state(self):
        data = {
            'state': self.state_machine.current_state.value,
            'previous_state': self.state_machine.previous_state.value,
            'error_reason': self.state_machine.error_reason,
            'active_map': self.state_machine.active_map,
            'is_emergency_stop': (self.state_machine.current_state == AMRState.EMERGENCY_STOP),
            'timestamp': time.time(),
            'active_processes': {
                'slam': self.supervisor.is_running('slam'),
                'localization': self.supervisor.is_running('localization'),
                'navigation': self.supervisor.is_running('navigation'),
                'amcl_pose_received': self.amcl_pose_received
            }
        }
        msg = String()
        msg.data = json.dumps(data)
        self.state_pub.publish(msg)

    def _publish_health(self):
        now = time.time()
        ok, warn, err = HealthStatus.OK, HealthStatus.WARNING, HealthStatus.ERROR
        components = {}

        def age(stamp):
            return now - stamp if stamp > 0 else 999.0

        def tiered(dt, ok_below, warn_below):
            if dt < ok_below:
                return ok
            return warn if dt < warn_below else err

        # LiDAR
        lidar_dt = age(self.last_scan_time)
        components['lidar'] = {
            'status': tiered(lidar_dt, 1.5, 5.0),
            'message': ('LiDAR publishing normally (~10Hz)' if lidar_dt < 1.5
                        else f'Delayed ({lidar_dt:.1f}s ago)'),
            'last_seen_sec': round(lidar_dt, 2)
        }

        # Obstacle cameras are optional: never seen = not fitted / disabled,
        # which is not a fault. Once seen, a stall is a warning.
        for key, stamp, label in (('camera', self.last_camera_time, 'Camera obstacle scan'),
                                  ('depth_camera', self.last_depth_time, 'Depth obstacles')):
            dt = age(stamp)
            if stamp == 0:
                components[key] = {'status': ok, 'running': False,
                                   'message': 'Not enabled', 'last_seen_sec': -1}
            else:
                components[key] = {
                    'status': ok if dt < 2.0 else warn,
                    'running': dt < 2.0,
                    'message': f'{label} publishing' if dt < 2.0 else f'Stalled ({dt:.1f}s ago)',
                    'last_seen_sec': round(dt, 2)
                }

        # Odometry
        odom_dt = age(self.last_odom_time)
        components['odometry'] = {
            'status': tiered(odom_dt, 1.0, 4.0),
            'message': ('Odometry publishing normally' if odom_dt < 1.0
                        else f'Delayed ({odom_dt:.1f}s ago)'),
            'last_seen_sec': round(odom_dt, 2)
        }

        # Drivetrain
        joints_dt = age(self.last_joint_states_time)
        components['controller'] = {
            'status': ok if joints_dt < 1.5 else warn,
            'message': 'Active joint states' if joints_dt < 1.5 else 'Waiting for joint states',
            'last_seen_sec': round(joints_dt, 2)
        }

        # TF
        tf_odom_base, msg_odom_base = self.check_tf(self.odom_frame, self.base_frame)
        tf_base_lidar, msg_base_lidar = self.check_tf(self.base_frame, self.lidar_frame)
        tf_ok = tf_odom_base and tf_base_lidar
        if tf_ok:
            tf_message = 'Base TF tree valid'
        else:
            tf_message = msg_odom_base if not tf_odom_base else msg_base_lidar
        components['tf'] = {
            'status': ok if tf_ok else err,
            'message': tf_message,
            'details': {'odom_to_base': tf_odom_base, 'base_to_lidar': tf_base_lidar}
        }

        # Map
        map_dt = age(self.last_map_time)
        map_fresh = self.map_received and map_dt < 5.0
        map_expected = self.state_machine.current_state in (
            AMRState.MAPPING, AMRState.LOCALIZATION)
        components['map'] = {
            'status': ok if map_fresh else (warn if map_expected else ok),
            'message': 'Occupancy grid available' if map_fresh else 'Idle',
            'last_seen_sec': round(map_dt, 2) if self.map_received else -1
        }

        # Subsystems
        slam_running = self.supervisor.is_running('slam')
        localization_running = self.supervisor.is_running('localization')
        navigation_running = self.supervisor.is_running('navigation')
        localized = self.amcl_pose_received and now - self.last_amcl_time < 5.0

        components['slam_toolbox'] = {
            'status': ok,
            'running': slam_running,
            'message': 'Running' if slam_running else 'Stopped'
        }
        components['map_server'] = {
            'status': ok if localization_running else warn,
            'running': localization_running,
            'message': 'Running' if localization_running else 'Stopped'
        }
        if localized:
            amcl_status, amcl_message = ok, 'Publishing AMCL pose'
        elif localization_running:
            amcl_status, amcl_message = warn, 'Waiting for initial pose/convergence'
        else:
            amcl_status, amcl_message = ok, 'Stopped'
        components['amcl'] = {
            'status': amcl_status,
            'running': localization_running,
            'localized': localized,
            'message': amcl_message
        }
        components['nav2'] = {
            'status': ok if navigation_running else warn,
            'running': navigation_running,
            'message': 'Ready' if navigation_running else 'Stopped'
        }

        has_error = any(components[k]['status'] == err for k in ('lidar', 'odometry', 'tf'))
        has_warn = any(v.get('status') == warn for v in components.values())

        health_data = {
            'overall': err if has_error else (warn if has_warn else ok),
            'current_mode': self.state_machine.current_state.value,
            'components': components,
            'timestamp': now
        }
        msg = String()
        msg.data = json.dumps(health_data)
        self.health_pub.publish(msg)

    def _publish_maps(self):
        msg = String()
        msg.data = json.dumps(self.list_maps())
        self.maps_pub.publish(msg)

    def _publish_battery(self):
        if self.state_machine.current_state in (AMRState.MAPPING, AMRState.NAVIGATION):
            self.battery_pct = max(10.0, self.battery_pct - 0.01)
            self.battery_voltage = 22.0 + (self.battery_pct / 100.0) * 3.4
        b_msg = BatteryState()
        b_msg.percentage = float(self.battery_pct / 100.0)
        b_msg.voltage = float(self.battery_voltage)
        b_msg.current = -1.5 if self.state_machine.current_state == AMRState.NAVIGATION else -0.3
        b_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        b_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b_msg.present = True
        self.battery_pub.publish(b_msg)

    def _command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            action = data.get('action', '').lower()
            self.get_logger().info(f'Executing JSON command: {action}')

            if action == 'start_robot':
                self._srv_start_robot(Trigger.Request(), Trigger.Response())
            elif action == 'stop_robot':
                self._srv_stop_robot(Trigger.Request(), Trigger.Response())
            elif action == 'start_mapping':
                self._srv_start_mapping(Trigger.Request(), Trigger.Response())
            elif action == 'stop_mapping':
                self._srv_stop_mapping(Trigger.Request(), Trigger.Response())
            elif action == 'save_map':
                map_name = data.get('map_name', 'warehouse_map')
                was_mapping = self.state_machine.current_state == AMRState.MAPPING
                if was_mapping:
                    self.state_machine.transition_to(AMRState.MAP_SAVING)
                ok, result = self.save_map(map_name)
                if was_mapping:
                    self.state_machine.transition_to(AMRState.MAPPING)
                self._publish_operation('save_map', ok, result, map_name=map_name)
            elif action in ('serialize_map', 'serialize_pose_graph'):
                map_name = data.get('map_name', 'warehouse_map')
                ok, result = self.serialize_pose_graph(map_name)
                self._publish_operation('serialize_pose_graph', ok, result, map_name=map_name)
            elif action in ('deserialize_map', 'deserialize_pose_graph', 'resume_mapping'):
                map_name = data.get('map_name', 'warehouse_map')
                ok, result = self._switch_mode('MAPPING', map_name)
                if ok:
                    ok, result = self.deserialize_pose_graph(map_name)
                self._publish_operation('deserialize_pose_graph', ok, result, map_name=map_name)
            elif (action == 'load_map'
                  or (action == 'set_mode' and data.get('mode') == 'LOCALIZATION')):
                map_name = data.get('map_name', '')
                self._switch_mode('LOCALIZATION', map_name)
            elif action == 'start_localization':
                map_name = data.get('map_name', '')
                self._switch_mode('LOCALIZATION', map_name)
            elif action == 'stop_localization':
                self._srv_stop_localization(Trigger.Request(), Trigger.Response())
            elif action == 'start_navigation':
                self._switch_mode('NAVIGATION')
            elif action == 'stop_navigation':
                self._srv_stop_navigation(Trigger.Request(), Trigger.Response())
            elif action == 'emergency_stop':
                self._srv_emergency_stop(Trigger.Request(), Trigger.Response())
            elif action == 'reset_emergency_stop':
                self._srv_reset_emergency_stop(Trigger.Request(), Trigger.Response())
            elif action == 'system_reset':
                self._srv_system_reset(Trigger.Request(), Trigger.Response())
            elif action == 'set_mode':
                mode = data.get('mode', '')
                map_name = data.get('map_name', '')
                self._switch_mode(mode, map_name)
        except Exception as e:
            self.get_logger().error(f'Error handling JSON command: {e}')

    def _srv_map_operation(self, request, response):
        operation = request.operation.strip().lower()
        name = request.map_name.strip()
        if operation == 'save_map':
            was_mapping = self.state_machine.current_state == AMRState.MAPPING
            if was_mapping:
                self.state_machine.transition_to(AMRState.MAP_SAVING)
            response.success, response.message = self.save_map(name)
            if was_mapping:
                self.state_machine.transition_to(AMRState.MAPPING)
            self._publish_operation(operation, response.success, response.message, map_name=name)
        elif operation in ('serialize_map', 'serialize_pose_graph'):
            response.success, response.message = self.serialize_pose_graph(name)
            self._publish_operation('serialize_pose_graph', response.success,
                                    response.message, map_name=name)
        elif operation in ('deserialize_map', 'deserialize_pose_graph', 'resume_mapping'):
            response.success, response.message = self._switch_mode('MAPPING', name)
            if response.success:
                response.success, response.message = self.deserialize_pose_graph(name)
            self._publish_operation('deserialize_pose_graph', response.success,
                                    response.message, map_name=name)
        else:
            response.success = False
            response.message = f"Unknown map operation '{request.operation}'."
        return response

    def _srv_set_mode(self, request, response):
        response.success, response.message = self._switch_mode(request.mode, request.map_name)
        return response

    def _switch_mode(self, mode: str, map_name: str = '') -> tuple[bool, str]:
        target = mode.upper()
        if (self.state_machine.current_state == AMRState.EMERGENCY_STOP
                and target != 'EMERGENCY_STOP'):
            return False, 'System in EMERGENCY_STOP. Call reset_emergency_stop first.'

        if target == 'READY':
            self.supervisor.stop_all()
            return self.state_machine.transition_to(AMRState.READY)

        elif target == 'MAPPING':
            ok_pf, pf_msg = self._validate_preflight_mapping()
            if not ok_pf:
                return False, pf_msg
            ok, msg = self.state_machine.transition_to(AMRState.MAPPING)
            if not ok:
                return False, msg
            p_ok, p_msg = self.supervisor.start_slam()
            if not p_ok:
                self.state_machine.transition_to(AMRState.READY)
            return p_ok, f'{msg}. {p_msg}'

        elif target == 'LOCALIZATION':
            chosen_map = map_name or self.state_machine.active_map
            if not chosen_map:
                maps = self.list_maps()
                if maps:
                    chosen_map = maps[0]
                else:
                    return False, 'No map specified and no maps found.'
            map_path = self.get_map_path(chosen_map)
            if not map_path:
                return False, f"Map '{chosen_map}' not found."

            self.state_machine.active_map = chosen_map
            if self.state_machine.current_state == AMRState.MAPPING:
                self.supervisor.stop_slam()
                self.state_machine.transition_to(AMRState.READY)

            # A pose from a previous AMCL instance must not satisfy the
            # "recent AMCL pose" gate for the new one.
            self.amcl_pose_received = False
            self.last_amcl_time = 0.0
            self.auto_nav_pending = False

            ok, msg = self.state_machine.transition_to(AMRState.LOCALIZATION)
            if not ok:
                return False, msg
            p_ok, p_msg = self.supervisor.start_localization(map_path)
            if not p_ok:
                self.state_machine.transition_to(AMRState.READY)
            return p_ok, f'{msg}. Active map: {chosen_map}. {p_msg}'

        elif target == 'NAVIGATION':
            if (self.state_machine.current_state == AMRState.NAVIGATION and
                    self.supervisor.is_running('navigation')):
                # Re-requesting NAVIGATION used to restart localization,
                # which tears down the running Nav2 stack mid-mission.
                return True, 'Already in NAVIGATION'
            if self.state_machine.current_state != AMRState.LOCALIZATION:
                s_ok, s_msg = self._switch_mode('LOCALIZATION', map_name)
                if not s_ok:
                    return False, f'Prerequisite localization failed: {s_msg}'
                time.sleep(1.0)
            ok, msg = self.state_machine.transition_to(AMRState.NAVIGATION)
            if not ok:
                return False, msg
            p_ok, p_msg = self.supervisor.start_navigation()
            if not p_ok:
                self.state_machine.transition_to(AMRState.LOCALIZATION)
            return p_ok, f'{msg}. {p_msg}'

        elif target == 'EMERGENCY_STOP':
            self.supervisor.stop_navigation()
            for _ in range(5):
                self.cmd_vel_pub.publish(Twist())
            return self.state_machine.trigger_emergency_stop()

        elif target == 'OFF':
            self.supervisor.stop_all()
            self.cmd_vel_pub.publish(Twist())
            return self.state_machine.transition_to(AMRState.OFF)

        return False, f'Unknown mode {target}'

    def _srv_start_robot(self, request, response):
        if self.state_machine.current_state == AMRState.OFF:
            self.state_machine.transition_to(AMRState.BOOTING)
            ok, msg = self.state_machine.transition_to(AMRState.READY)
            response.success = ok
            response.message = 'Robot booted and ready.'
        else:
            response.success = True
            response.message = f'Robot is online (State: {self.state_machine.current_state.value})'
        return response

    def _srv_stop_robot(self, request, response):
        self.supervisor.stop_all()
        self.cmd_vel_pub.publish(Twist())
        ok, msg = self.state_machine.transition_to(AMRState.OFF)
        response.success = ok
        response.message = 'Robot software state transitioned to OFF.'
        return response

    def _srv_start_mapping(self, request, response):
        ok, msg = self._switch_mode('MAPPING')
        response.success = ok
        response.message = msg
        return response

    def _srv_stop_mapping(self, request, response):
        self.supervisor.stop_slam()
        ok, msg = self.state_machine.transition_to(AMRState.READY)
        response.success = ok
        response.message = 'Mapping stopped. System is READY.'
        return response

    def _srv_start_localization(self, request, response):
        ok, msg = self._switch_mode('LOCALIZATION')
        response.success = ok
        response.message = msg
        return response

    def _srv_stop_localization(self, request, response):
        self.supervisor.stop_localization()
        ok, msg = self.state_machine.transition_to(AMRState.READY)
        response.success = ok
        response.message = 'Localization stopped. System is READY.'
        return response

    def _srv_start_navigation(self, request, response):
        ok, msg = self._switch_mode('NAVIGATION')
        response.success = ok
        response.message = msg
        return response

    def _srv_stop_navigation(self, request, response):
        self.supervisor.stop_navigation()
        if self.state_machine.current_state == AMRState.NAVIGATION:
            self.state_machine.transition_to(AMRState.LOCALIZATION)
        response.success = True
        response.message = 'Navigation stopped. Subsystem in LOCALIZATION.'
        return response

    def _srv_emergency_stop(self, request, response):
        self.get_logger().warn('EMERGENCY STOP TRIGGERED!')
        ok, msg = self._switch_mode('EMERGENCY_STOP')
        response.success = ok
        response.message = msg
        return response

    def _srv_reset_emergency_stop(self, request, response):
        self.get_logger().info('Resetting emergency stop')
        ok, msg = self.state_machine.reset_emergency_stop()
        response.success = ok
        response.message = msg
        return response

    def _srv_system_reset(self, request, response):
        self.supervisor.stop_all()
        self.cmd_vel_pub.publish(Twist())
        self.state_machine.reset_emergency_stop()
        self.state_machine.transition_to(AMRState.READY)
        response.success = True
        response.message = 'System fully reset to READY.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AMRSystemManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.supervisor.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
