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

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


def _xacro_path():
    return (
        Path(__file__).resolve().parents[1]
        / 'description'
        / 'optimus.urdf.xacro'
    )


def test_robot_description_files_exist():
    root = Path(__file__).resolve().parents[1]
    assert (root / 'description' / 'optimus.urdf.xacro').is_file()
    assert (
        root / 'description' / 'control' / 'optimus.ros2_control.xacro'
    ).is_file()


def _process(use_sim, sim_backend='classic'):
    doc = xacro.process_file(
        str(_xacro_path()),
        mappings={
            'use_sim': use_sim,
            'sim_backend': sim_backend,
            'wheel_radius': '0.0655',
            'wheel_separation': '0.65',
        },
    )
    return ET.fromstring(doc.toxml())


def _hardware_params(root):
    return {
        p.get('name'): p.text.strip()
        for p in root.findall('ros2_control/hardware/param')
    }


def test_real_hardware_urdf_is_valid_and_has_wheel_joints():
    root = _process('false')

    assert root.tag == 'robot'

    joint_names = {j.get('name') for j in root.findall('joint')}
    assert 'left_joint' in joint_names
    assert 'right_joint' in joint_names

    link_names = {link.get('name') for link in root.findall('link')}
    assert 'base_link' in link_names
    assert 'left_wheel' in link_names
    assert 'right_wheel' in link_names
    assert 'lidar_link' in link_names


def test_real_hardware_urdf_uses_zlac_drive_plugin():
    root = _process('false')

    ros2_control = root.find('ros2_control')
    assert ros2_control is not None

    plugin = ros2_control.find('hardware/plugin')
    assert plugin is not None
    assert plugin.text.strip() == 'zlac_drive/ZlacSystemHardware'

    joint_names = {
        j.get('name') for j in ros2_control.findall('joint')
    }
    assert joint_names == {'left_joint', 'right_joint'}

    for joint in ros2_control.findall('joint'):
        command_interfaces = {
            c.get('name') for c in joint.findall('command_interface')
        }
        state_interfaces = {
            s.get('name') for s in joint.findall('state_interface')
        }
        assert command_interfaces == {'velocity'}
        assert state_interfaces == {'position', 'velocity'}


def test_real_hardware_direction_mapping():
    # Physical Optimus mapping: left motor inverted, right not. The hardware
    # plugin honours these parameters (it no longer hard-codes them).
    params = _hardware_params(_process('false'))
    assert params['left_motor_inverted'] == '1'
    assert params['right_motor_inverted'] == '0'


def _assert_full_scan(horizontal_scan):
    assert horizontal_scan is not None
    # LiDAR is on top of the body: full 360-degree scan.
    assert horizontal_scan.findtext('samples') == '360'
    assert horizontal_scan.findtext('min_angle') == '-3.14159265359'
    assert horizontal_scan.findtext('max_angle') == '3.14159265359'


def test_classic_simulation_urdf_uses_gazebo_hardware_not_zlac_drive():
    root = _process('true', 'classic')

    ros2_control = root.find('ros2_control')
    assert ros2_control is not None

    plugin = ros2_control.find('hardware/plugin')
    assert plugin is not None
    # Simulation must use Gazebo hardware, never the real ZLAC driver.
    assert plugin.text.strip() != 'zlac_drive/ZlacSystemHardware'
    assert plugin.text.strip() == 'gazebo_ros2_control/GazeboSystem'

    gazebo_plugin = root.find('gazebo/plugin')
    assert gazebo_plugin is not None
    assert gazebo_plugin.get('filename') == 'libgazebo_ros2_control.so'

    lidar_sensor = root.find("gazebo[@reference='lidar_link']/sensor")
    assert lidar_sensor is not None
    assert lidar_sensor.get('type') == 'ray'
    lidar_plugin = lidar_sensor.find('plugin')
    assert lidar_plugin is not None
    assert lidar_plugin.get('filename') == 'libgazebo_ros_ray_sensor.so'

    _assert_full_scan(lidar_sensor.find('ray/scan/horizontal'))


def test_gz_simulation_urdf_uses_gz_ros2_control():
    root = _process('true', 'gz')

    plugin = root.find('ros2_control/hardware/plugin')
    assert plugin is not None
    assert plugin.text.strip() == 'gz_ros2_control/GazeboSimSystem'

    gazebo_plugin = root.find('gazebo/plugin')
    assert gazebo_plugin is not None
    assert gazebo_plugin.get('filename') == 'gz_ros2_control-system'
    assert gazebo_plugin.findtext('parameters').endswith('config/controllers.yaml')

    lidar_sensor = root.find("gazebo[@reference='lidar_link']/sensor")
    assert lidar_sensor is not None
    assert lidar_sensor.get('type') == 'gpu_lidar'
    assert lidar_sensor.findtext('topic') == '/scan'
    assert lidar_sensor.findtext('gz_frame_id') == 'lidar_link'
    _assert_full_scan(lidar_sensor.find('lidar/scan/horizontal'))

    # Ball casters must not drag under Gazebo Sim's default friction.
    for caster in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        surface = root.find(f"gazebo[@reference='{caster}_caster']/mu1")
        assert surface is not None and float(surface.text) == 0.0


def test_urdf_lidar_on_top_and_storage_container():
    root = _process('false')
    joints = {j.get('name'): j for j in root.findall('joint')}
    lidar_z = float(joints['lidar_joint'].find('origin').get('xyz').split()[2])
    container = joints['storage_container_joint'].find('origin').get('xyz').split()
    container_top = float(container[2]) + 0.025 / 2.0
    # Scan plane must clear the container for the 360-degree view.
    assert lidar_z > 0.30
    assert lidar_z > container_top


def _axis_in_base(joint):
    # Rotate the joint axis by the joint origin's roll (wheels only roll).
    roll = float(joint.find('origin').get('rpy').split()[0])
    ax, ay, az = (float(v) for v in joint.find('axis').get('xyz').split())
    return (ax,
            ay * math.cos(roll) - az * math.sin(roll),
            ay * math.sin(roll) + az * math.cos(roll))


def test_wheels_follow_rep103_and_positive_spin_drives_forward():
    joints = {j.get('name'): j for j in _process('false').findall('joint')}
    left_y = float(joints['left_joint'].find('origin').get('xyz').split()[1])
    right_y = float(joints['right_joint'].find('origin').get('xyz').split()[1])
    # +y is the robot's left (REP-103).
    assert left_y > 0.0 > right_y
    # Spin axis along +y: positive wheel velocity rolls the robot forward.
    for name in ('left_joint', 'right_joint'):
        axis = _axis_in_base(joints[name])
        assert abs(axis[1] - 1.0) < 1e-6, (name, axis)
