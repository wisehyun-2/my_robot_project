"""upkie spine node (Plan 1): MuJoCo simulation + upkie observers behind a ROS 2 interface.

Runtime structure (mirrors upkie's spine):

  background thread @ spine_hz (1 kHz, time.perf_counter pacing)
      servo commands -> MuJoCo (moteus-like servo PD evaluated at every physics substep)
      -> observation (optional sensor noise / latency emulation)
      -> observers: BaseOrientation, FloorContact, WheelOdometry (wbr.observers.upkie_observers)
      -> publish wbr_msgs/SpineObservation (decimated to observation_hz, or every cycle),
         sensor_msgs/Imu, sensor_msgs/JointState and the odom -> base transform
  executor threads: /cmd_vel or /upkie/servo_commands subscriptions, ~/reset service

embedded_balancer = true  : the WheelBalancer PI + GyropodMapper of wbr run inside the 1 kHz loop
                            (upkie's wheel_balancer spine pipeline); /cmd_vel gives the targets
                            (linear.x = ground velocity [m/s], angular.z = yaw velocity [rad/s]).
embedded_balancer = false : servo commands come from /upkie/servo_commands (upkie_balancer node).
                            Watchdog: without a command for command_timeout_ms the spine applies
                            zero-torque ("stop") commands, like upkie's spine when the agent dies.

The simulation, servo properties (kp 20, kd 1; 16 N m legs, 1.7 N m wheels), observers, balancer
and mapper are exactly the objects built by wbr.plans.plan1_upkie.Plan1Upkie.__init__ (wheel
radius 0.05 m, wheel base 0.3048 m, standing keyframe placement).  Only the stepping is done here:
Plan1Upkie.step() is not used because it ties the balancer to the simulation and logs every cycle
into memory.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from wbr import MODELS_DIR
from wbr.common.servo import ServoCommand
from wbr.plans.plan1_upkie import JOINTS, Plan1Config, Plan1Upkie
from wbr_msgs.msg import ServoCommands, ServoState, SpineObservation

# moteus "stop" equivalent: no feedback terms, no feedforward torque, zero torque limit
STOP_COMMAND = ServoCommand(position=math.nan, velocity=0.0, feedforward_torque=0.0,
                            kp_scale=0.0, kd_scale=0.0, maximum_torque=0.0)


def copy_quaternion(q, out) -> None:
    """Copy a MuJoCo (w, x, y, z) quaternion into a geometry_msgs/Quaternion (x, y, z, w)."""
    out.w = float(q[0])
    out.x = float(q[1])
    out.y = float(q[2])
    out.z = float(q[3])


def copy_vector3(v, out) -> None:
    out.x = float(v[0])
    out.y = float(v[1])
    out.z = float(v[2])


class SpineNode(Node):
    """1 kHz spine loop around wbr.plans.plan1_upkie.Plan1Upkie's simulation and observers."""

    def __init__(self) -> None:
        super().__init__("spine_node")

        # ------------------------------------------------------------ parameters
        self.declare_parameter("spine_hz", 1000.0)
        self.declare_parameter("scene", str(MODELS_DIR / "upkie" / "scene.xml"))
        self.declare_parameter("latency_ms", 0.0)
        self.declare_parameter("noise", False)
        self.declare_parameter("seed", 0)
        self.declare_parameter("use_sim_contact", False)
        self.declare_parameter("pitch_offset", "model")   # "model": subtract the model's equilibrium pitch (IMU zero), "none": raw
        self.declare_parameter("embedded_balancer", True)
        self.declare_parameter("gain_scale", 1.0)
        self.declare_parameter("observation_hz", 200.0)
        self.declare_parameter("publish_every_cycle", False)
        self.declare_parameter("imu_hz", 200.0)
        self.declare_parameter("joint_state_hz", 50.0)
        self.declare_parameter("command_timeout_ms", 100.0)
        self.declare_parameter("cmd_vel_timeout_ms", 500.0)   # embedded balancer: zero targets after this silence
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("imu_frame", "base")

        spine_hz = float(self._param("spine_hz"))
        scene = str(self._param("scene")).strip() or str(MODELS_DIR / "upkie" / "scene.xml")
        self._embedded = bool(self._param("embedded_balancer"))
        self._publish_every_cycle = bool(self._param("publish_every_cycle"))
        self._command_timeout = float(self._param("command_timeout_ms")) * 1e-3
        self._cmd_vel_timeout = float(self._param("cmd_vel_timeout_ms")) * 1e-3
        self._odom_frame = str(self._param("odom_frame"))
        self._base_frame = str(self._param("base_frame"))
        self._imu_frame = str(self._param("imu_frame"))

        # ------------------------------------------------------------ simulation
        # Exactly Plan1Upkie.__init__: MuJoCo scene, ServoModel (kp 20 / kd 1, 16 N m legs and
        # 1.7 N m wheels), observers, WheelBalancer, GyropodMapper, noise/delay emulation and the
        # standing placement of the base (reset()).
        cfg = Plan1Config(balancer="pid", spine_hz=spine_hz, latency_ms=float(self._param("latency_ms")),
                          noise=bool(self._param("noise")), seed=int(self._param("seed")),
                          gain_scale=float(self._param("gain_scale")),
                          use_sim_contact=bool(self._param("use_sim_contact")), scene=scene,
                          pitch_offset=str(self._param("pitch_offset")))
        self.get_logger().info(f"loading MuJoCo scene {MODELS_DIR / 'upkie' / scene}")
        self._plan = Plan1Upkie(cfg)
        self._dt = self._plan.dt
        signs = self._plan.sim.wheel_forward_signs()
        self.get_logger().info(
            "wheel forward signs (joint velocity sign that rolls the robot forward): "
            + ", ".join(f"{w}={int(s):+d}" for w, s in signs.items())
            + " -> balancer_node parameter left_wheeled="
            + str(signs["left_wheel"] > 0))

        self._joint_set = frozenset(JOINTS)
        self._stop_commands = {j: STOP_COMMAND for j in JOINTS}
        self._obs_period = self._period_steps(spine_hz, float(self._param("observation_hz")))
        self._imu_period = self._period_steps(spine_hz, float(self._param("imu_hz")))
        self._js_period = self._period_steps(spine_hz, float(self._param("joint_state_hz")))

        # ------------------------------------------------------------ shared state
        self._lock = threading.Lock()
        self._target_ground_velocity = 0.0
        self._target_yaw_velocity = 0.0
        self._cmd_vel_time: Optional[float] = None
        self._servo_commands: Optional[Dict[str, ServoCommand]] = None
        self._servo_commands_time = -math.inf
        self._reset_requested = False
        self._latest_observation = self._plan.observation

        # ------------------------------------------------------------ ROS interface
        self._obs_pub = self.create_publisher(SpineObservation, "upkie/observation", qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self._tf = TransformBroadcaster(self) if bool(self._param("publish_tf")) else None
        if self._embedded:
            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().info("embedded balancer: WheelBalancer PI + GyropodMapper at %.0f Hz, "
                                   "targets from /cmd_vel" % spine_hz)
        else:
            self.create_subscription(ServoCommands, "upkie/servo_commands", self._on_servo_commands,
                                     qos_profile_sensor_data)
            self.get_logger().info("external balancer: servo commands from /upkie/servo_commands "
                                   "(watchdog %.0f ms)" % (self._command_timeout * 1e3))
        self.create_service(Trigger, "~/reset", self._on_reset)

        # ------------------------------------------------------------ 1 kHz loop
        self._stop_event = threading.Event()
        self._overruns = 0
        self._thread = threading.Thread(target=self._loop, name="spine_loop", daemon=True)
        self._thread.start()

    # ---------------------------------------------------------------- helpers
    def _param(self, name: str):
        return self.get_parameter(name).value

    @staticmethod
    def _period_steps(loop_hz: float, rate_hz: float) -> int:
        """Decimation factor for a publisher at rate_hz inside a loop at loop_hz (0 = disabled)."""
        if rate_hz <= 0.0:
            return 0
        return max(1, int(round(loop_hz / rate_hz)))

    # ---------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._target_ground_velocity = float(msg.linear.x)
            self._target_yaw_velocity = float(msg.angular.z)
            self._cmd_vel_time = time.perf_counter()

    def _on_servo_commands(self, msg: ServoCommands) -> None:
        cmds: Dict[str, ServoCommand] = {}
        for s in msg.servos:
            if s.name not in self._joint_set:
                self.get_logger().warning(f"ignoring command for unknown servo '{s.name}'",
                                          throttle_duration_sec=5.0)
                continue
            cmds[s.name] = ServoCommand(position=float(s.position), velocity=float(s.velocity),
                                        feedforward_torque=float(s.feedforward_torque),
                                        kp_scale=float(s.kp_scale), kd_scale=float(s.kd_scale),
                                        maximum_torque=float(s.maximum_torque))
        if not cmds:
            return
        with self._lock:
            # joints absent from a message keep their previous command (MujocoSim.set_commands merges)
            self._servo_commands = cmds
            self._servo_commands_time = time.perf_counter()

    def _on_reset(self, request, response):
        with self._lock:
            self._reset_requested = True
        response.success = True
        response.message = "reset scheduled for the next spine cycle"
        return response

    # ---------------------------------------------------------------- loop
    def _loop(self) -> None:
        period = self._dt
        next_time = time.perf_counter()
        stats_time, stats_k = next_time, 0
        k = 0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._cycle(k)
            except Exception as exc:  # keep the loop alive; ~/reset can recover the simulation
                self.get_logger().error(f"spine cycle failed: {exc!r}", throttle_duration_sec=1.0)
            k += 1
            next_time += period
            now = time.perf_counter()
            if now < next_time:
                time.sleep(next_time - now)
            elif now - next_time > 0.1:
                # more than 100 ms behind: resynchronise instead of running without pauses
                self._overruns += 1
                next_time = now
            if now - stats_time >= 5.0:
                rate = (k - stats_k) / (now - stats_time)
                if rate < 0.9 / period:
                    self.get_logger().warning(
                        f"spine loop at {rate:.0f} Hz (target {1.0 / period:.0f} Hz), {self._overruns} resyncs")
                stats_time, stats_k = now, k

    def _cycle(self, k: int) -> None:
        plan = self._plan
        now = time.perf_counter()
        with self._lock:
            v, w, cmd_vel_time = self._target_ground_velocity, self._target_yaw_velocity, self._cmd_vel_time
            ext_cmds, ext_time = self._servo_commands, self._servo_commands_time
            do_reset, self._reset_requested = self._reset_requested, False
        if do_reset:
            plan.reset()
            self.get_logger().info("simulation reset (standing pose)")

        obs = plan.observation
        if self._embedded:
            if self._cmd_vel_timeout > 0.0 and (cmd_vel_time is None or now - cmd_vel_time > self._cmd_vel_timeout):
                v = w = 0.0
            # same code path as Plan1Upkie.step: balancer + gyropod mapping, yaw convention (+w = CCW)
            cmds, _ = plan.control_step(obs, v, w)
        elif ext_cmds is None:
            # upkie's spine does not drive the robot before an agent connects: hold the simulation
            # (the observation is still published so that the balancer node can initialise)
            cmds = None
        elif now - ext_time > self._command_timeout:
            cmds = self._stop_commands
            self.get_logger().warning("no servo command for %.0f ms: zero-torque commands applied"
                                      % (self._command_timeout * 1e3), throttle_duration_sec=1.0)
        else:
            cmds = ext_cmds

        if cmds is not None:
            plan.sim.set_commands(cmds)
            plan.sim.step()
            plan.k += 1
            plan.observation = plan._observe()   # noise / delay emulation + the three observers (as Plan1Upkie.step)
        obs = plan.observation
        with self._lock:
            self._latest_observation = obs

        stamp = self.get_clock().now().to_msg()
        if self._publish_every_cycle or (self._obs_period and k % self._obs_period == 0):
            self._obs_pub.publish(self._observation_msg(obs, stamp))
        if self._imu_period and k % self._imu_period == 0:
            self._imu_pub.publish(self._imu_msg(obs, stamp))
        if self._js_period and k % self._js_period == 0:
            self._js_pub.publish(self._joint_state_msg(obs, stamp))
            if self._tf is not None:
                self._tf.sendTransform(self._odom_transform(obs, stamp))

    # ---------------------------------------------------------------- messages
    def _imu_msg(self, obs: dict, stamp) -> Imu:
        imu = obs["imu"]
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        copy_quaternion(imu["orientation"], msg.orientation)
        copy_vector3(imu["angular_velocity"], msg.angular_velocity)
        copy_vector3(imu["linear_acceleration"], msg.linear_acceleration)
        return msg

    def _observation_msg(self, obs: dict, stamp) -> SpineObservation:
        msg = SpineObservation()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        bo = obs["base_orientation"]
        av = bo["angular_velocity"]
        msg.base_pitch = float(bo["pitch"])
        msg.base_angular_velocity = [float(av[0]), float(av[1]), float(av[2])]
        wo = obs["wheel_odometry"]
        msg.ground_position = float(wo["position"])
        msg.ground_velocity = float(wo["velocity"])
        msg.floor_contact = bool(obs["floor_contact"]["contact"])
        servo = obs["servo"]
        msg.servos = [ServoState(name=j, position=float(servo[j]["position"]), velocity=float(servo[j]["velocity"]),
                                 torque=float(servo[j]["torque"])) for j in JOINTS]
        msg.imu = self._imu_msg(obs, stamp)
        return msg

    def _joint_state_msg(self, obs: dict, stamp) -> JointState:
        servo = obs["servo"]
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINTS)
        msg.position = [float(servo[j]["position"]) for j in JOINTS]
        msg.velocity = [float(servo[j]["velocity"]) for j in JOINTS]
        msg.effort = [float(servo[j]["torque"]) for j in JOINTS]
        return msg

    def _odom_transform(self, obs: dict, stamp) -> TransformStamped:
        base = obs["sim"]["base"]   # ground truth pose of the MuJoCo base body (same frame as the URDF root)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        copy_vector3(base["position"], t.transform.translation)
        copy_quaternion(base["orientation"], t.transform.rotation)
        return t

    # ---------------------------------------------------------------- lifecycle
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpineNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
