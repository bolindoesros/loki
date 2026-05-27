#!/usr/bin/env python3
"""
monitor.py
──────────
Foxglove monitoring node for the Loki AUV.
Republishes all state and commands under /monitor/* as Float64.
Setpoint echoes (/monitor/target/*) are published by loki_controller directly.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int32


class MonitorNode(Node):

    def __init__(self) -> None:
        super().__init__("loki_monitor")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._pub = {
            # Vehicle state
            "depth":           self.create_publisher(Float64, "/monitor/depth",           qos),
            "pitch":           self.create_publisher(Float64, "/monitor/pitch_deg",       qos),
            "roll":            self.create_publisher(Float64, "/monitor/roll_deg",        qos),
            "heading":         self.create_publisher(Float64, "/monitor/heading_deg",     qos),
            "speed":           self.create_publisher(Float64, "/monitor/speed",           qos),
            # Commands
            "cmd_thruster":    self.create_publisher(Float64, "/monitor/cmd/thruster",    qos),
            "cmd_elevator":    self.create_publisher(Float64, "/monitor/cmd/elevator",    qos),
            "cmd_rudder":      self.create_publisher(Float64, "/monitor/cmd/rudder",      qos),
            "cmd_moving_mass": self.create_publisher(Float64, "/monitor/cmd/moving_mass", qos),
            # System
            "armed":           self.create_publisher(Float64, "/monitor/armed",           qos),
        }

        # State
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, qos)

        # Commands
        self.create_subscription(Int32, "/cmd/thruster",
            lambda m: self._pub_f("cmd_thruster", float(m.data)), qos)
        self.create_subscription(Int32, "/cmd/elevator",
            lambda m: self._pub_f("cmd_elevator", float(m.data)), qos)
        self.create_subscription(Int32, "/cmd/rudder",
            lambda m: self._pub_f("cmd_rudder", float(m.data)), qos)
        self.create_subscription(Float64, "/cmd/moving_mass",
            lambda m: self._pub_f("cmd_moving_mass", m.data), qos)

        # System
        self.create_subscription(Bool, "/system/arm_state",
            lambda m: self._pub_f("armed", 1.0 if m.data else 0.0), qos)

        self.get_logger().info("MonitorNode ready — /monitor/*")

    def _on_odom(self, msg: Odometry) -> None:
        self._pub_f("depth", -msg.pose.pose.position.z)
        self._pub_f("speed",  msg.twist.twist.linear.x)

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)

        self._pub_f("pitch", math.degrees(pitch))
        self._pub_f("roll",  math.degrees(roll))

        heading = math.degrees(yaw)
        if heading < 0.0:
            heading += 360.0
        self._pub_f("heading", heading)

    def _pub_f(self, key: str, value: float) -> None:
        msg = Float64()
        msg.data = value
        self._pub[key].publish(msg)

    @staticmethod
    def _quat_to_rpy(x, y, z, w):
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw


def main() -> None:
    rclpy.init()
    rclpy.spin(MonitorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
