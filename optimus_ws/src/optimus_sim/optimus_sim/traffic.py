#!/usr/bin/env python3
"""Drives the physical trolley back and forth along aisle C-D (x 6.5 -> 14.5, y -4).

Param `trolley_speed` (m/s, 0 = parked), `trolley_travel` (m).
Pause/resume: ros2 param set /traffic trolley_speed 0.0
"""
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Traffic(Node):
    def __init__(self):
        super().__init__("traffic")
        self.declare_parameter("trolley_speed", 0.4)
        self.declare_parameter("trolley_travel", 8.0)
        self.pub = self.create_publisher(Twist, "/trolley/cmd_vel", 10)
        self.create_subscription(Odometry, "/trolley/odom", self._odom, 10)
        self.x = 0.0          # odometry is relative to the trolley's start pose
        self.direction = 1.0
        self.create_timer(0.05, self._tick)

    def _odom(self, msg):
        self.x = msg.pose.pose.position.x

    def _tick(self):
        speed = self.get_parameter("trolley_speed").value
        travel = self.get_parameter("trolley_travel").value
        if self.x >= travel:
            self.direction = -1.0
        elif self.x <= 0.0:
            self.direction = 1.0
        t = Twist()
        t.linear.x = self.direction * speed
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Traffic()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.pub.publish(Twist())
        rclpy.try_shutdown()
