#!/usr/bin/env python3
"""
hardware bridge for loki auv
Relays /cmd/* from loki_control to ESP32 via micro-ROS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Float64
from loki_msgs.msg import EspPacket


def pwm_to_int8(pwm: int) -> int:
    """
    Temporary: map PWM (1100-1900) to int8 (-100 to 100).
    Centre (1500) = 0.
    """
    return max(-100, min(100, (pwm - 1500) // 4))


class HwBridge(Node):

    def __init__(self) -> None:
        super().__init__("hw_bridge")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publisher to ESP32
        self._esp_pub = self.create_publisher(EspPacket, "/pc_to_esp_cmd", qos)

        # Internal state
        self._thruster    = 1500
        self._elevator    = 1500
        self._rudder      = 1500
        self._moving_mass = 0.0
        self._seq         = 0

        # Subscribers from loki_control
        self.create_subscription(Int32,   "/cmd/thruster",    self._on_thruster,    qos)
        self.create_subscription(Int32,   "/cmd/elevator",    self._on_elevator,    qos)
        self.create_subscription(Int32,   "/cmd/rudder",      self._on_rudder,      qos)
        self.create_subscription(Float64, "/cmd/moving_mass", self._on_moving_mass, qos)

        # Publish at 20Hz
        self.create_timer(0.05, self._publish)

        self.get_logger().info("HwBridge ready — waiting for ESP32 firmware subscriber")

    def _on_thruster(self,    msg: Int32)   -> None: self._thruster    = msg.data
    def _on_elevator(self,    msg: Int32)   -> None: self._elevator    = msg.data
    def _on_rudder(self,      msg: Int32)   -> None: self._rudder      = msg.data
    def _on_moving_mass(self, msg: Float64) -> None: self._moving_mass = msg.data

    def _publish(self) -> None:
        pkt        = EspPacket()
        pkt.seq    = self._seq
        self._seq += 1

        # cmd[0] = thruster, cmd[1] = elevator, cmd[2] = rudder
        # cmd[3] = moving_mass (0-100), cmd[4]-cmd[5] = reserved
        pkt.cmd[0] = pwm_to_int8(self._thruster)
        pkt.cmd[1] = pwm_to_int8(self._elevator)
        pkt.cmd[2] = pwm_to_int8(self._rudder)
        pkt.cmd[3] = max(0, min(100, int(self._moving_mass * 100)))
        pkt.cmd[4] = 0  # reserved
        pkt.cmd[5] = 0  # reserved

        self._esp_pub.publish(pkt)


def main() -> None:
    rclpy.init()
    rclpy.spin(HwBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
