"""geometry_msgs/Twist -> motion_msgs/MotionCtrl bridge for the DIABLO-lite MCU node.

/cmd_vel (linear.x = forward [m/s], angular.z = left [rad/s]) is turned into a movement command
(mode_mark = false) published on diablo/MotionCmd immediately, and the last command is re-sent every
heartbeat_ms (180 ms) like diablo_ctrl_node's heart_beat_loop, which keeps the MCU's host link alive
(500 ms timeout).  Stop the bridge and the MCU releases control and stops the robot; while the bridge
runs, a silent /cmd_vel publisher is covered by cmd_vel_timeout_ms (the bridge then sends zero velocities).

Optionally a single stand-up mode command (mode_mark = true, stand_mode = true) is sent at startup
so that a robot started in CAR mode stands up (harmless when it is already standing).
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from motion_msgs.msg import MotionCtrl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_bridge")
        self.declare_parameter("heartbeat_ms", 180.0)          # diablo_ctrl_node heart_beat_loop period
        self.declare_parameter("up", 0.0)                       # height command (0 = nominal standing height)
        self.declare_parameter("stand_mode", True)              # mode flag carried by every message
        self.declare_parameter("send_stand_on_start", True)     # one stand-up mode command after start
        self.declare_parameter("stand_delay_s", 1.0)            # delay of that command (discovery)
        self.declare_parameter("cmd_vel_timeout_ms", 500.0)     # zero velocities after this silence (0 = hold forever)

        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_ms").value) * 1e-3
        self._last_cmd_vel: Optional[float] = None

        self._msg = MotionCtrl()
        self._msg.mode_mark = False
        self._msg.value.up = float(self.get_parameter("up").value)
        self._msg.mode.stand_mode = bool(self.get_parameter("stand_mode").value)

        self._pub = self.create_publisher(MotionCtrl, "diablo/MotionCmd", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(float(self.get_parameter("heartbeat_ms").value) * 1e-3, self._heartbeat)
        self._stand_timer = None
        if bool(self.get_parameter("send_stand_on_start").value):
            self._stand_timer = self.create_timer(float(self.get_parameter("stand_delay_s").value), self._send_stand_once)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._msg.mode_mark = False
        self._msg.value.forward = float(msg.linear.x)
        self._msg.value.left = float(msg.angular.z)
        self._last_cmd_vel = time.perf_counter()
        self._pub.publish(self._msg)

    def _heartbeat(self) -> None:
        if self._cmd_vel_timeout > 0.0 and (self._last_cmd_vel is None
                                            or time.perf_counter() - self._last_cmd_vel > self._cmd_vel_timeout):
            self._msg.value.forward = 0.0
            self._msg.value.left = 0.0
        self._msg.mode_mark = False
        self._pub.publish(self._msg)

    def _send_stand_once(self) -> None:
        if self._stand_timer is not None:
            self._stand_timer.cancel()
        stand = MotionCtrl()
        stand.mode_mark = True
        stand.mode.stand_mode = True
        stand.value.up = self._msg.value.up
        self._pub.publish(stand)
        self.get_logger().info("stand-up mode command sent")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
