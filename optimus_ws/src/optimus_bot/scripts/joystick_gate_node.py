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
External joystick gate.

teleop_twist_joy publishes the gamepad's velocity on an intermediate topic
(default /joy_teleop/cmd_vel). This node forwards it to /cmd_vel only while
the external joystick is enabled, which the dashboard toggles through the
/amr/joystick/enable (std_srvs/SetBool) service.

Safety behaviour:
  * starts disabled unless enabled_on_start is true;
  * a zero twist is published whenever forwarding stops (disable, E-STOP,
    gamepad unplugged), so the robot never coasts on a stale command;
  * EMERGENCY_STOP (from /amr/system_state) disables the joystick, and it
    cannot be enabled again until the E-STOP is cleared.

Status is published as JSON on /amr/joystick/status (std_msgs/String,
transient-local) once per second and on every change:
  {"enabled": bool, "device_connected": bool, "active": bool,
   "estop": bool, "message": str}
"""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class JoystickGate(Node):

    def __init__(self):
        super().__init__('joystick_gate')
        self.input_topic = self.declare_parameter(
            'input_topic', '/joy_teleop/cmd_vel').value
        self.output_topic = self.declare_parameter('output_topic', '/cmd_vel').value
        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        # joy_node publishes at autorepeat_rate while a device is open, so a
        # gap longer than this means the gamepad was unplugged or lost.
        self.device_timeout = float(self.declare_parameter('device_timeout_s', 1.0).value)
        self.enabled = bool(self.declare_parameter('enabled_on_start', False).value)

        self.estop = False
        self.last_joy_time = 0.0
        self.active = False          # forwarding a non-zero command
        self.message = 'External joystick disabled'
        self._last_status = None

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, '/amr/joystick/status', status_qos)

        self.create_subscription(Twist, self.input_topic, self._on_cmd, 10)
        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_subscription(String, '/amr/system_state', self._on_state, 10)
        self.create_service(SetBool, '/amr/joystick/enable', self._on_enable)
        self.create_timer(0.2, self._watchdog)
        self.create_timer(1.0, lambda: self._publish_status(force=True))

        self.get_logger().info(
            f'Joystick gate: {self.input_topic} -> {self.output_topic} '
            f"(starts {'ENABLED' if self.enabled else 'DISABLED'})")
        self._publish_status(force=True)

    # ------------------------------------------------------------------
    def _device_connected(self):
        return (time.monotonic() - self.last_joy_time) < self.device_timeout

    def _stop(self):
        self.cmd_pub.publish(Twist())
        self.active = False

    def _set_enabled(self, enabled, message):
        was = self.enabled
        self.enabled = enabled
        self.message = message
        if was and not enabled:
            self._stop()
        if was != enabled:
            self.get_logger().info(message)
        self._publish_status()

    def _publish_status(self, force=False):
        status = {
            'enabled': self.enabled,
            'device_connected': self._device_connected(),
            'active': self.active,
            'estop': self.estop,
            'message': self.message,
        }
        if not force and status == self._last_status:
            return
        self._last_status = status
        self.status_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------
    def _on_enable(self, request, response):
        if request.data:
            if self.estop:
                response.success = False
                response.message = 'Cannot enable external joystick during EMERGENCY STOP'
            elif not self._device_connected():
                response.success = False
                response.message = 'No external joystick detected on the robot'
            else:
                self._set_enabled(True, 'External joystick enabled (hold LB to drive)')
                response.success = True
                response.message = self.message
        else:
            self._set_enabled(False, 'External joystick disabled')
            response.success = True
            response.message = self.message
        return response

    def _on_joy(self, _msg):
        self.last_joy_time = time.monotonic()

    def _on_state(self, msg):
        try:
            estop = bool(json.loads(msg.data).get('is_emergency_stop', False))
        except (ValueError, AttributeError):
            return
        if estop and not self.estop and self.enabled:
            self.estop = True
            self._set_enabled(False, 'External joystick disabled by EMERGENCY STOP')
        elif estop != self.estop:
            self.estop = estop
            self._publish_status()

    def _on_cmd(self, msg):
        if not self.enabled or self.estop:
            return
        self.cmd_pub.publish(msg)
        self.active = (abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3)
        self._publish_status()

    def _watchdog(self):
        if self.enabled and not self._device_connected():
            self._set_enabled(False, 'External joystick disconnected; control disabled')
        else:
            self._publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.enabled:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
