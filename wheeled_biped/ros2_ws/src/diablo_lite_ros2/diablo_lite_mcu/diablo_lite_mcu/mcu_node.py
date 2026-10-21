"""DIABLO-lite MCU node (Plans 2 and 3): the robot's real-time program behind the DIABLO ROS 2 interface.

In simulation the node embeds MuJoCo through the plan assemblies of wbr:

  balancer = lqi6       -> wbr.plans.plan2_diablo_lqi6.Plan2DiabloLQI6   (LQI6 law of the design report)
  balancer = upkie_pid  -> wbr.plans.plan3_diablo_upkie.Plan3DiabloUpkie (upkie wheel balancer PI)
  balancer = upkie_mpc  -> wbr.plans.plan3_diablo_upkie.Plan3DiabloUpkie (upkie MPC balancer)

On the real robot this node is replaced by the firmware (the DiabloMcu loop) plus a serial bridge.

Threads
  mcu loop (background, mcu_hz = 1 kHz, time.perf_counter pacing): plan.step(forward, left)
      = host link poll (timeout) -> DiabloMcu.tick (estimator, mode state machine, balance law,
        leg / yaw / roll control) -> MuJoCo step -> observation
  executor: /diablo/MotionCmd subscription, telemetry timers (IMU + Euler and motors 50 Hz,
      body state 10 Hz, joint states + odom TF 50 Hz), ~/reset service

Host link semantics (wbr.controllers.diablo_arch.host_link)
  Every motion_msgs/MotionCtrl received on /diablo/MotionCmd is one "serial packet" from the DIABLO
  host.  The node keeps the host-side command state like diablo_ctrl_node's ctrl_msg_ -- a message
  with mode_mark = true updates only the mode flags (stand / height / pitch / roll ctrl modes, jump,
  split), otherwise only the movement values (forward, left, up, roll, pitch, leg_split) -- and
  pushes a copy into the MCU link with HostLink.receive(), which obtains control and refreshes the
  timeout.  The plan's own HostEmulator (which would re-send the command at 50 Hz by itself and
  therefore never time out) is disabled, so the MCU releases control and runs its disconnect program
  (zero velocity, hold the posture) when no command arrives within host_timeout_ms (500 ms), exactly
  as the real MCU does when diablo_ctrl_node and its 180 ms heartbeat die.  Keeping the link alive
  is the job of the ROS publishers (diablo_lite_ctrl: cmd_vel_bridge heartbeat, teleop_node at 25 Hz).
"""
from __future__ import annotations

import collections
import copy
import math
import threading
import time
from typing import Optional

import rclpy
from ception_msgs.msg import IMUEuler
from geometry_msgs.msg import TransformStamped
from motion_msgs.msg import LegMotors, MotionCtrl, RobotStatus
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from wbr import MODELS_DIR
from wbr.common.rotations import quat_to_rpy
from wbr.controllers.diablo_arch.host_link import MotionCtrl as HostMotionCtrl
from wbr.controllers.diablo_arch.host_link import RobotMode
from wbr.plans.common import Logger
from wbr.plans.plan2_diablo_lqi6 import JOINTS, Plan2Config, Plan2DiabloLQI6
from wbr.plans.plan3_diablo_upkie import Plan3Config, Plan3DiabloUpkie

# /joint_states order: actuated joints + the passive five-bar joints of the URDF tree
JOINT_STATE_NAMES = ["left_hip", "left_knee", "left_wheel", "left_crank", "left_rod",
                     "right_hip", "right_knee", "right_wheel", "right_crank", "right_rod"]
# DIABLO motor naming -> DIABLO-lite joint: the "knee" motor is the crank motor of the parallelogram
DIABLO_MOTORS = (("hip", "hip"), ("knee", "crank"), ("wheel", "wheel"))
TWO_PI = 2.0 * math.pi


class _NullLogger(Logger):
    """Replaces the plan's in-memory Logger, which would otherwise grow at 1 kHz forever."""

    def log(self, **kw) -> None:
        pass


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


class McuNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_node")

        # ------------------------------------------------------------ parameters
        self.declare_parameter("balancer", "lqi6")          # lqi6 | upkie_pid | upkie_mpc
        self.declare_parameter("mcu_hz", 1000.0)
        self.declare_parameter("scene", "scene.xml")        # file in wbr.MODELS_DIR/diablo_lite or absolute path
        self.declare_parameter("latency_ms", 0.0)
        self.declare_parameter("noise", False)
        self.declare_parameter("seed", 0)
        self.declare_parameter("host_timeout_ms", 500.0)    # HostLink timeout (OSDK obtain_control)
        self.declare_parameter("start_mode", "stand")       # stand | car
        self.declare_parameter("initial_up", 0.0)           # height command before the first MotionCmd
        self.declare_parameter("gain_scale", 1.0)           # Plan 3: multiplier on the wheel balancer PI gains
        self.declare_parameter("leg_gain_scale", 2.0)       # Plan 3: leg position gain scale
        self.declare_parameter("mpc_leg_length", 0.30)      # Plan 3 MPC pendulum length
        self.declare_parameter("use_sim_contact", False)    # Plan 3: ground-truth wheel contact
        self.declare_parameter("imu_hz", 50.0)
        self.declare_parameter("motors_hz", 50.0)
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("joint_state_hz", 50.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sensor_frame_id", "imu_link")   # DIABLO uses "diablo_robot"

        kind = str(self._param("balancer")).lower()
        mcu_hz = float(self._param("mcu_hz"))
        scene = str(self._param("scene")).strip() or "scene.xml"
        start_mode_name = str(self._param("start_mode")).lower()
        if start_mode_name == "car":
            # DIABLO-lite has no auxiliary wheels: a robot in CAR mode lies on the floor without torque and the
            # pitch FAULT latches before any stand-up command can arrive (review pass 2)
            raise ValueError("start_mode 'car' is not supported on DIABLO-lite (no auxiliary wheels); use 'stand'")
        if start_mode_name != "stand":
            raise ValueError(f"start_mode must be 'stand', got '{start_mode_name}'")
        start_mode = RobotMode.STAND if start_mode_name == "stand" else RobotMode.CAR
        self._odom_frame = str(self._param("odom_frame"))
        self._base_frame = str(self._param("base_frame"))
        self._sensor_frame = str(self._param("sensor_frame_id"))

        # ------------------------------------------------------------ host-side command state
        self._lock = threading.Lock()
        # mirror of diablo_ctrl_node's ctrl_msg_: values and modes persist between messages
        self._host_cmd = HostMotionCtrl(forward=0.0, left=0.0, up=float(self._param("initial_up")))
        self._host_cmd.mode.stand_mode = start_mode == RobotMode.STAND
        # every received MotionCtrl is one serial packet; packets are queued in order so that a mode
        # packet (e.g. teleop 'z': stand-up) is never overwritten by the movement packet that follows
        # it within the same MCU cycle (review C3)
        self._packets = collections.deque(maxlen=64)
        self._forward = 0.0
        self._left = 0.0
        self._last_cmd_time: Optional[float] = None
        self._reset_requested = False
        self._n_link_events = 0
        self._n_mcu_events = 0

        # ------------------------------------------------------------ plan (MuJoCo + MCU)
        common = dict(mcu_hz=mcu_hz, latency_ms=float(self._param("latency_ms")), noise=bool(self._param("noise")),
                      seed=int(self._param("seed")), host_timeout_ms=float(self._param("host_timeout_ms")),
                      scene=scene, start_mode=start_mode)
        self.get_logger().info(f"loading MuJoCo scene {MODELS_DIR / 'diablo_lite' / scene}; balancer={kind}")
        if kind == "lqi6":
            self._plan = Plan2DiabloLQI6(Plan2Config(**common))
        elif kind in ("upkie_pid", "upkie_mpc"):
            self._plan = Plan3DiabloUpkie(Plan3Config(
                **common, balancer="pid" if kind == "upkie_pid" else "mpc",
                gain_scale=float(self._param("gain_scale")), leg_gain_scale=float(self._param("leg_gain_scale")),
                mpc_leg_length=float(self._param("mpc_leg_length")), use_sim_contact=bool(self._param("use_sim_contact"))))
        else:
            raise ValueError(f"unknown balancer '{kind}' (expected lqi6, upkie_pid or upkie_mpc)")
        self._dt = self._plan.dt
        self._detach_host_emulator()
        # the real program starts disarmed and arms itself once the robot is held upright (|pitch| < 3 deg)
        self._plan.mcu.disarm("waiting for an upright robot")
        self._snapshot = self._make_snapshot(self._plan.observation)

        # ------------------------------------------------------------ ROS interface (DIABLO topics)
        self.create_subscription(MotionCtrl, "diablo/MotionCmd", self._on_motion_cmd, 10)
        self._imu_pub = self.create_publisher(Imu, "diablo/sensor/Imu", 10)
        self._euler_pub = self.create_publisher(IMUEuler, "diablo/sensor/ImuEuler", 10)
        self._motors_pub = self.create_publisher(LegMotors, "diablo/sensor/Motors", 10)
        self._status_pub = self.create_publisher(RobotStatus, "diablo/sensor/Body_state", 10)
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self._tf = TransformBroadcaster(self) if bool(self._param("publish_tf")) else None
        self._make_timer(float(self._param("imu_hz")), self._publish_imu)
        self._make_timer(float(self._param("motors_hz")), self._publish_motors)
        self._make_timer(float(self._param("status_hz")), self._publish_status)
        self._make_timer(float(self._param("joint_state_hz")), self._publish_joint_states)
        self.create_service(Trigger, "~/reset", self._on_reset)

        # ------------------------------------------------------------ 1 kHz loop
        self._stop_event = threading.Event()
        self._overruns = 0
        self._thread = threading.Thread(target=self._loop, name="mcu_loop", daemon=True)
        self._thread.start()
        self.get_logger().info(f"MCU loop at {mcu_hz:.0f} Hz, host link timeout "
                               f"{self._plan.link.p.timeout_ms:.0f} ms, start mode {start_mode.name}")

    # ---------------------------------------------------------------- helpers
    def _param(self, name: str):
        return self.get_parameter(name).value

    def _make_timer(self, rate_hz: float, callback) -> None:
        if rate_hz > 0.0:
            self.create_timer(1.0 / rate_hz, callback)

    def _detach_host_emulator(self) -> None:
        """ROS replaces the emulated host: no automatic 50 Hz re-send, no in-memory log."""
        plan = self._plan
        plan.host.enabled = False
        plan.log = _NullLogger()
        with self._lock:
            # command the MCU acts on before the host obtains control (disconnect program keeps
            # `up` and the mode flags): the ROS-side state (initial_up parameter)
            plan.link.last_cmd = copy.deepcopy(self._host_cmd)

    def _make_snapshot(self, obs: dict) -> dict:
        plan = self._plan
        est = plan.mcu.est
        return {
            "obs": obs,
            "L0": {"left": float(est.L0.get("left", 0.0)), "right": float(est.L0.get("right", 0.0))},
            "mode": int(plan.mcu.mode),
            "fault": bool(plan.mcu.fault),
            "armed": bool(plan.mcu.armed),
            "ctrl_mode": int(plan.link.ctrl_mode),
            "in_control": bool(plan.link.in_control),
        }

    # ---------------------------------------------------------------- callbacks
    def _on_motion_cmd(self, msg: MotionCtrl) -> None:
        with self._lock:
            c = self._host_cmd
            c.mode_mark = bool(msg.mode_mark)
            if msg.mode_mark:
                # diablo_ctrl_node: transform up/down + mode flags only, movement values untouched
                m = c.mode
                m.stand_mode = bool(msg.mode.stand_mode)
                m.height_ctrl_mode = bool(msg.mode.height_ctrl_mode)
                m.pitch_ctrl_mode = bool(msg.mode.pitch_ctrl_mode)
                m.roll_ctrl_mode = bool(msg.mode.roll_ctrl_mode)
                m.jump_mode = bool(msg.mode.jump_mode)
                m.split_mode = bool(msg.mode.split_mode)
            else:
                c.forward = float(msg.value.forward)
                c.left = float(msg.value.left)
                c.up = float(msg.value.up)
                c.roll = float(msg.value.roll)
                c.pitch = float(msg.value.pitch)
                c.leg_split = float(msg.value.leg_split)
                self._forward, self._left = c.forward, c.left
            self._packets.append(copy.deepcopy(c))   # one serial packet, applied by the MCU loop in order
            self._last_cmd_time = time.perf_counter()

    def _on_reset(self, request, response):
        with self._lock:
            self._reset_requested = True
        response.success = True
        response.message = "reset scheduled for the next MCU cycle"
        return response

    # ---------------------------------------------------------------- loop
    def _loop(self) -> None:
        period = self._dt
        next_time = time.perf_counter()
        stats_time, stats_k = next_time, 0
        k = 0
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._cycle()
            except Exception as exc:  # keep the loop alive; ~/reset can recover the simulation
                self.get_logger().error(f"MCU cycle failed: {exc!r}", throttle_duration_sec=1.0)
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
                        f"MCU loop at {rate:.0f} Hz (target {1.0 / period:.0f} Hz), {self._overruns} resyncs")
                stats_time, stats_k = now, k

    def _cycle(self) -> None:
        plan = self._plan
        with self._lock:
            packets = list(self._packets)
            self._packets.clear()
            forward, left = self._forward, self._left
            do_reset, self._reset_requested = self._reset_requested, False
        if do_reset:
            plan.reset()                    # re-creates plan.host / plan.log and resets the link
            self._detach_host_emulator()
            plan.mcu.disarm("reset: waiting for an upright robot")
            self.get_logger().info("simulation reset (equilibrium standing pose)")
        for pkt in packets:
            # HostLink.receive(): obtains control on the first packet, refreshes the timeout
            plan.link.receive(pkt, plan.mcu.t)
            if pkt.mode_mark:
                plan.mcu.apply_command(pkt, 0.0)   # mode transitions are applied in order, never lost
        if not plan.mcu.armed and not plan.mcu.fault:
            if plan.mcu.arm():
                self.get_logger().info("MCU armed (robot upright)")
        # Plan step: host emulator tick (inert), MCU tick with link.poll() timeout handling,
        # MuJoCo step, observation.  forward / left mirror the last movement command.
        obs = plan.step(forward, left)
        snapshot = self._make_snapshot(obs)
        with self._lock:
            self._snapshot = snapshot

    # ---------------------------------------------------------------- telemetry
    def _latest(self) -> dict:
        with self._lock:
            return self._snapshot

    def _publish_imu(self) -> None:
        imu = self._latest()["obs"]["imu"]
        stamp = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._sensor_frame
        copy_quaternion(imu["orientation"], msg.orientation)
        copy_vector3(imu["angular_velocity"], msg.angular_velocity)
        copy_vector3(imu["linear_acceleration"], msg.linear_acceleration)
        self._imu_pub.publish(msg)

        roll, pitch, yaw = quat_to_rpy(imu["orientation"])
        euler = IMUEuler()
        euler.header.stamp = stamp
        euler.header.frame_id = self._sensor_frame
        euler.roll = float(roll)
        euler.pitch = float(pitch)
        euler.yaw = float(yaw)
        self._euler_pub.publish(euler)

    def _publish_motors(self) -> None:
        snap = self._latest()
        servo = snap["obs"]["servo"]
        msg = LegMotors()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._sensor_frame
        for side in ("left", "right"):
            for motor, joint in DIABLO_MOTORS:
                s = servo[f"{side}_{joint}"]
                position = float(s["position"])
                # DIABLO convention: pos = angle within one turn, enc_rev = number of full turns
                rev = math.floor(position / TWO_PI)
                setattr(msg, f"{side}_{motor}_enc_rev", int(rev))
                setattr(msg, f"{side}_{motor}_pos", position - rev * TWO_PI)
                setattr(msg, f"{side}_{motor}_vel", float(s["velocity"]))
                setattr(msg, f"{side}_{motor}_iq", float(s["torque"]))   # torque [N m] in place of the current
        msg.left_leg_length = snap["L0"]["left"]
        msg.right_leg_length = snap["L0"]["right"]
        self._motors_pub.publish(msg)

    def _publish_status(self) -> None:
        snap = self._latest()
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._sensor_frame
        msg.ctrl_mode_msg = int(snap["ctrl_mode"])      # CtrlMode: 0 RC, 1 MOTION (host), 2 VIRTUAL_RC
        msg.robot_mode_msg = int(snap["mode"])          # RobotMode: 2 CAR, 3 STAND, 4/5 transitions
        msg.error_msg = (1 if snap["fault"] else 0) | (0 if snap["armed"] else 2)   # bit 0: pitch fault, bit 1: disarmed
        msg.warning_msg = 0
        self._status_pub.publish(msg)
        self._n_link_events = self._log_events(self._plan.link.events, self._n_link_events, "host link")
        self._n_mcu_events = self._log_events(self._plan.mcu.events, self._n_mcu_events, "MCU")

    def _log_events(self, events: list, seen: int, source: str) -> int:
        if seen > len(events):        # list recreated by a reset
            seen = 0
        while seen < len(events):
            t, text = events[seen]
            self.get_logger().info(f"[{source} t={t:.3f}] {text}")
            seen += 1
        return seen

    def _publish_joint_states(self) -> None:
        obs = self._latest()["obs"]
        servo = obs["servo"]
        stamp = self.get_clock().now().to_msg()
        pos = {j: float(servo[j]["position"]) for j in JOINTS}
        vel = {j: float(servo[j]["velocity"]) for j in JOINTS}
        eff = {j: float(servo[j]["torque"]) for j in JOINTS}
        for side in ("left", "right"):
            # passive parallelogram joints (see wbr.models.build_diablo_lite / Plan2DiabloLQI6._set_leg_pose)
            pos[f"{side}_knee"] = pos[f"{side}_crank"] - pos[f"{side}_hip"]
            pos[f"{side}_rod"] = pos[f"{side}_hip"] - pos[f"{side}_crank"]
            vel[f"{side}_knee"] = vel[f"{side}_crank"] - vel[f"{side}_hip"]
            vel[f"{side}_rod"] = vel[f"{side}_hip"] - vel[f"{side}_crank"]
            eff[f"{side}_knee"] = 0.0
            eff[f"{side}_rod"] = 0.0
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_STATE_NAMES)
        msg.position = [pos[j] for j in JOINT_STATE_NAMES]
        msg.velocity = [vel[j] for j in JOINT_STATE_NAMES]
        msg.effort = [eff[j] for j in JOINT_STATE_NAMES]
        self._js_pub.publish(msg)

        if self._tf is not None:
            base = obs["sim"]["base"]   # ground-truth pose of the MuJoCo base body (= URDF base_link)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            copy_vector3(base["position"], t.transform.translation)
            copy_quaternion(base["orientation"], t.transform.rotation)
            self._tf.sendTransform(t)

    # ---------------------------------------------------------------- lifecycle
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuNode()
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
