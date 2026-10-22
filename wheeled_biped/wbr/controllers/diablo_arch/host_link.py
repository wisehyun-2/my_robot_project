"""Emulation of the DIABLO host <-> MCU link semantics (diablo_ros2 / OSDK):

* MotionCtrl message (motion_msgs/MotionCtrl): mode_mark, value {forward, left, up, roll, pitch,
  leg_split}, mode {pitch_ctrl_mode, roll_ctrl_mode, height_ctrl_mode, stand_mode, jump_mode, split_mode}
* the host obtains control authority with a timeout (default 500 ms): control is released by the
  MCU when no valid command arrives within the timeout (timeout_act = 0: run the disconnect program,
  no fallback to the RC)
* diablo_ctrl_node re-sends the last movement command every 180 ms (heart_beat_loop)
* telemetry push rates: IMU/motors 50 Hz, status 10 Hz, power 1 Hz (not needed in simulation)
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from enum import IntEnum


class CtrlMode(IntEnum):
    RC = 0
    MOTION = 1
    VIRTUAL_RC = 2


class RobotMode(IntEnum):
    DISCONNECT = 0
    INITIALIZE = 1
    CAR = 2
    STAND = 3
    TRANSITION_DOWN = 4
    TRANSITION_UP = 5


@dataclass
class MovementCtrlMode:
    pitch_ctrl_mode: bool = False     # False: angle, True: angular velocity
    roll_ctrl_mode: bool = False
    height_ctrl_mode: bool = False    # False: height, True: vertical velocity
    stand_mode: bool = False          # same default as the ROS message (motion_msgs/MovementCtrlMode)
    jump_mode: bool = False
    split_mode: bool = False


@dataclass
class MotionCtrl:
    """Mirror of motion_msgs/msg/MotionCtrl."""
    mode_mark: bool = False
    forward: float = 0.0      # m/s
    left: float = 0.0         # rad/s (yaw rate, positive = turn left)
    up: float = 0.0           # height command (position mode, -1..1) or vertical velocity; ROS default 0
    roll: float = 0.0         # rad (angle mode) or rad/s
    pitch: float = 0.0        # rad (angle mode) or rad/s
    leg_split: float = 0.0
    host_shaped: bool = False  # True: the host already shaped the velocity reference (stage-2 MPC): the MCU tracks it
                              # plainly; False (and the disconnect program): the MCU applies its own stage-1 shaping
    mode: MovementCtrlMode = field(default_factory=MovementCtrlMode)


@dataclass
class HostLinkParams:
    timeout_ms: float = 500.0      # obtain_control(timeout_ms)
    heartbeat_ms: float = 180.0    # diablo_ctrl_node heart_beat_loop usleep(180000)
    disconnect_brake_s: float = 2.0  # design report: after 2 s without commands, stop driving


class HostLink:
    """MCU-side view of the serial link: stores the latest command and tracks control authority."""

    def __init__(self, params: HostLinkParams | None = None):
        self.p = params or HostLinkParams()
        self.reset()

    def reset(self) -> None:
        self.ctrl_mode = CtrlMode.RC
        self.in_control = False
        self.last_rx_time = -1e9
        self.last_cmd = MotionCtrl()
        self.events = []

    def obtain_control(self, t: float) -> None:
        self.in_control = True
        self.ctrl_mode = CtrlMode.MOTION
        self.last_rx_time = t
        self.events.append((t, "control obtained"))

    def receive(self, cmd: MotionCtrl, t: float) -> None:
        """Called whenever the host sends a MotionCtrl (teleop at 25-50 Hz or the 180 ms heartbeat)."""
        if not self.in_control:
            self.obtain_control(t)
        self.last_cmd = copy.deepcopy(cmd)   # one serial packet: later edits of the host object must not leak
        self.last_rx_time = t

    def poll(self, t: float) -> MotionCtrl:
        """Return the command the MCU should act on at time t (timeout handling)."""
        if self.in_control and (t - self.last_rx_time) * 1e3 > self.p.timeout_ms:
            self.in_control = False
            self.ctrl_mode = CtrlMode.RC
            self.events.append((t, "control released (timeout)"))
        if not self.in_control:
            # disconnect program: zero velocities, keep the posture; velocity-mode inputs (up / roll / pitch
            # rates) are zeroed too so that the posture stops moving (review B9)
            last = self.last_cmd
            safe = MotionCtrl(up=0.0 if last.mode.height_ctrl_mode else last.up,
                              roll=0.0 if last.mode.roll_ctrl_mode else last.roll,
                              pitch=0.0 if last.mode.pitch_ctrl_mode else last.pitch,
                              mode=copy.deepcopy(last.mode))
            return safe
        return self.last_cmd


class HostEmulator:
    """Host side: publishes MotionCtrl at a fixed rate and keeps the 180 ms heartbeat."""

    def __init__(self, link: HostLink, publish_hz: float = 50.0):
        self.link = link
        self.period = 1.0 / publish_hz
        self.next_pub = 0.0
        self.last_sent = -1e9
        self.cmd = MotionCtrl()
        self.enabled = True

    def set_command(self, **fields) -> None:
        for k, v in fields.items():
            if hasattr(self.cmd, k):
                setattr(self.cmd, k, v)
            elif hasattr(self.cmd.mode, k):
                setattr(self.cmd.mode, k, v)
            else:
                raise KeyError(k)

    def tick(self, t: float) -> None:
        if not self.enabled:
            return
        if t >= self.next_pub or (t - self.last_sent) * 1e3 >= self.link.p.heartbeat_ms:
            self.link.receive(self.cmd, t)
            self.last_sent = t
            self.next_pub = t + self.period
