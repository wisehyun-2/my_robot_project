"""upkie balancer node (Plan 1, external balancer).

Subscribes the spine observation (/upkie/observation) and the velocity targets (/cmd_vel), runs

  balancer = pid : wbr.controllers.upkie.wheel_balancer.WheelBalancer (PI on base pitch and ground
                   position -> ground velocity) at every observation, i.e. at the spine's
                   observation_hz (upkie's wheel_balancer pipeline moved out of the spine), or
  balancer = mpc : wbr.controllers.upkie.mpc_balancer.MPCBalancer (linear MPC on the wheeled
                   inverted pendulum) at mpc_hz, decimated from the observation rate,

maps the ground / yaw velocity to servo commands with wbr.controllers.upkie.gyropod.GyropodMapper
(wheels in velocity mode, legs in position mode) and publishes wbr_msgs/ServoCommands on
/upkie/servo_commands.  Everything runs inside the observation callback (a few microseconds for the
PI, about a millisecond for the MPC QP), so no extra thread is needed.

observation_hz must match the spine's observation_hz (or spine_hz when publish_every_cycle is set):
it fixes the integration period of the PI and of the leg position filter.
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from wbr.controllers.upkie.gyropod import GyropodMapper
from wbr.controllers.upkie.wheel_balancer import WheelBalancer, WheelBalancerParams
from wbr.plans.plan1_upkie import WHEEL_BASE, WHEEL_RADIUS
from wbr_msgs.msg import ServoCommand as ServoCommandMsg
from wbr_msgs.msg import ServoCommands, SpineObservation


class BalancerNode(Node):
    def __init__(self) -> None:
        super().__init__("balancer_node")
        self.declare_parameter("balancer", "pid")            # pid | mpc
        self.declare_parameter("observation_hz", 200.0)      # rate of /upkie/observation
        self.declare_parameter("mpc_hz", 200.0)
        self.declare_parameter("gain_scale", 1.0)            # multiplier on the four PI gains
        self.declare_parameter("mpc_leg_length", 0.58)
        self.declare_parameter("left_wheeled", True)         # +left wheel velocity rolls forward (logged by the spine)
        self.declare_parameter("cmd_vel_timeout_ms", 500.0)  # zero targets after this silence (0 = hold forever)
        self.declare_parameter("yaw_sign", -1.0)             # upkie's yaw is clockwise-positive; -1 -> /cmd_vel angular.z CCW (REP-103)
        self.declare_parameter("reset_jump_m", 0.1)          # odometry jump that signals a spine reset (re-initialise)

        kind = str(self.get_parameter("balancer").value).lower()
        observation_hz = float(self.get_parameter("observation_hz").value)
        mpc_hz = float(self.get_parameter("mpc_hz").value)
        self._dt = 1.0 / observation_hz
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_ms").value) * 1e-3
        left_wheeled = bool(self.get_parameter("left_wheeled").value)
        self._yaw_sign = float(self.get_parameter("yaw_sign").value)
        self._reset_jump = float(self.get_parameter("reset_jump_m").value)
        self._last_position: Optional[float] = None

        # same construction as Plan1Upkie.__init__ (dt = observation period instead of the spine period)
        self._balancer = WheelBalancer(WheelBalancerParams(
            dt=self._dt, wheel_radius=WHEEL_RADIUS, contact_radius=WHEEL_BASE / 2,
            gain_scale=float(self.get_parameter("gain_scale").value)))
        self._mapper = GyropodMapper(WHEEL_RADIUS, WHEEL_BASE, self._dt, left_wheeled=left_wheeled)
        self._mpc = None
        self._mpc_period = 1
        if kind == "mpc":
            from wbr.controllers.upkie.mpc_balancer import MPCBalancer
            self._mpc = MPCBalancer(leg_length=float(self.get_parameter("mpc_leg_length").value))
            self._mpc_period = max(1, int(round(observation_hz / mpc_hz)))
            self.get_logger().info(f"MPC balancer every {self._mpc_period} observation(s) "
                                   f"({observation_hz / self._mpc_period:.0f} Hz)")
        elif kind == "pid":
            self.get_logger().info(f"wheel balancer PI at the observation rate ({observation_hz:.0f} Hz)")
        else:
            raise ValueError(f"unknown balancer '{kind}' (expected pid or mpc)")

        self._ground_velocity = 0.0
        self._k = 0
        self._initialized = False
        self._target_ground_velocity = 0.0
        self._target_yaw_velocity = 0.0
        self._cmd_vel_time: Optional[float] = None

        self._pub = self.create_publisher(ServoCommands, "upkie/servo_commands", qos_profile_sensor_data)
        self.create_subscription(SpineObservation, "upkie/observation", self._on_observation, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

    # ---------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._target_ground_velocity = float(msg.linear.x)
        self._target_yaw_velocity = float(msg.angular.z)
        self._cmd_vel_time = time.perf_counter()

    def _on_observation(self, msg: SpineObservation) -> None:
        servo = {s.name: s for s in msg.servos}
        position = float(msg.ground_position)
        if self._initialized and self._last_position is not None and abs(position - self._last_position) > self._reset_jump:
            # the spine was reset (odometry restarted): forget the integrators and leg filter state
            self.get_logger().info("odometry jump detected: re-initialising the balancer")
            self._balancer.reset()
            if self._mpc is not None:
                self._mpc.reset()
            self._ground_velocity = 0.0
            self._initialized = False
        self._last_position = position
        if not self._initialized:
            try:
                self._mapper.reset({j: float(servo[j].position) for j in self._mapper.upper_leg_joints})
            except KeyError as missing:
                self.get_logger().warning(f"observation without servo {missing}; waiting", throttle_duration_sec=2.0)
                return
            self._initialized = True
            self.get_logger().info("first observation received, balancing")

        observation = {
            "base_orientation": {"pitch": float(msg.base_pitch),
                                 "angular_velocity": [float(x) for x in msg.base_angular_velocity]},
            "wheel_odometry": {"position": float(msg.ground_position), "velocity": float(msg.ground_velocity)},
            "floor_contact": {"contact": bool(msg.floor_contact)},
        }
        v, w = self._target_ground_velocity, self._target_yaw_velocity
        if self._cmd_vel_timeout > 0.0 and (self._cmd_vel_time is None
                                            or time.perf_counter() - self._cmd_vel_time > self._cmd_vel_timeout):
            v = w = 0.0
        w = self._yaw_sign * w                         # as Plan1Upkie.control_step (cfg.yaw_sign)

        if self._mpc is not None:                      # Plan1Upkie.control_step, MPC branch
            if self._k % self._mpc_period == 0:
                self._ground_velocity = self._mpc.step(v, observation, self._mpc_period * self._dt)
            scale = 2.0 + (2.0 if abs(w) > 0.1 else 0.0)   # leg gains stiffen while turning (upkie follow_joystick)
            cmds = self._mapper.servo_commands(self._ground_velocity, w, scale, scale)
        else:                                          # Plan1Upkie.step, PI branch
            out = self._balancer.step(observation, v, w)
            self._ground_velocity = out["ground_velocity"]
            cmds = self._mapper.servo_commands(self._ground_velocity, w, out["leg_kp_scale"], out["leg_kd_scale"])
        self._k += 1

        out_msg = ServoCommands()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.servos = [
            ServoCommandMsg(name=name, position=float(c.position), velocity=float(c.velocity),
                            feedforward_torque=float(c.feedforward_torque), kp_scale=float(c.kp_scale),
                            kd_scale=float(c.kd_scale), maximum_torque=float(c.maximum_torque))
            for name, c in cmds.items()]
        self._pub.publish(out_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalancerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
