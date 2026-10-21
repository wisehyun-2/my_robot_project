"""Keyboard teleoperation of the DIABLO-lite MCU node.

Port of DIABLO's diablo_teleop/teleop.py (Direct Drive Technology, Apache-2.0): same key map, same
message semantics, same 25 Hz publication of motion_msgs/MotionCtrl on diablo/MotionCmd.  One key is
consumed per 40 ms tick; when the key queue is empty the movement values are zeroed (so the robot only
moves while keys keep arriving, e.g. with the terminal's key repeat).  Linux terminal only (termios).

    ros2 run diablo_lite_ctrl teleop_node        (run it in its own terminal, not through ros2 launch)

  w / s   forward +1.0 / -1.0 m/s        a / d   turn left / right (left = +1.0 / -1.0 rad/s)
  q / e   roll -0.1 / +0.1   r roll 0    u / i / o  pitch +0.5 / 0 / -0.5
  h j k l height (up) -0.5 / 1.0 / 0.5 / 0.0
  v / b   height velocity mode on / off  n / m   pitch velocity mode on / off
  z       stand up (then up = 1.0)       x       sit down (car mode)
  c       jump                           f / g   split (dance) on / off
  `       quit (Ctrl-C also quits)
"""
from __future__ import annotations

import sys
import threading
from collections import deque

import rclpy
from motion_msgs.msg import MotionCtrl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    import termios
    import tty
except ImportError:   # not a POSIX terminal (e.g. Windows): the node cannot read keys
    termios = None
    tty = None

QUIT_KEYS = ("`", "\x03")   # backtick as in the original, Ctrl-C (delivered as a character in raw mode)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("diablo_teleop_node")
        self.declare_parameter("rate_hz", 25.0)
        self._pub = self.create_publisher(MotionCtrl, "diablo/MotionCmd", 2)
        self._msg = MotionCtrl()
        self._keys = deque()
        self._keys_lock = threading.Lock()
        self.stopped = False
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._tick)

    # ---------------------------------------------------------------- keys
    def push_key(self, key: str) -> None:
        with self._keys_lock:
            self._keys.append(key)

    def _pop_key(self):
        with self._keys_lock:
            return self._keys.popleft() if self._keys else None

    # ---------------------------------------------------------------- message (generMsgs port)
    def _set(self, forward=None, left=None, roll=None, up=None, pitch=None, mode_mark=False,
             height_ctrl_mode=None, pitch_ctrl_mode=None, roll_ctrl_mode=None, stand_mode=None,
             jump_mode=False, dance_mode=None) -> None:
        m = self._msg
        m.mode_mark = mode_mark
        m.mode.jump_mode = jump_mode
        if dance_mode is not None:
            m.mode.split_mode = dance_mode
        if forward is not None:
            m.value.forward = float(forward)
        if left is not None:
            m.value.left = float(left)
        if pitch is not None:
            m.value.pitch = float(pitch)
        if roll is not None:
            m.value.roll = float(roll)
        if up is not None:
            m.value.up = float(up)
        if height_ctrl_mode is not None:
            m.mode.height_ctrl_mode = height_ctrl_mode
        if pitch_ctrl_mode is not None:
            m.mode.pitch_ctrl_mode = pitch_ctrl_mode
        if roll_ctrl_mode is not None:
            m.mode.roll_ctrl_mode = roll_ctrl_mode
        if stand_mode is not None:
            m.mode.stand_mode = stand_mode

    def _publish(self) -> None:
        self._pub.publish(self._msg)

    # ---------------------------------------------------------------- 25 Hz tick (main loop port)
    def _tick(self) -> None:
        if self.stopped:
            return
        key = self._pop_key()
        if key is None:
            self._msg.mode_mark = False
            self._msg.mode.split_mode = False
            self._msg.value.forward = 0.0
            self._msg.value.left = 0.0
        elif key == "w":
            self._set(forward=1.0)
        elif key == "s":
            self._set(forward=-1.0)
        elif key == "a":
            self._set(left=1.0)
        elif key == "d":
            self._set(left=-1.0)
        elif key == "e":
            self._set(roll=0.1)
        elif key == "q":
            self._set(roll=-0.1)
        elif key == "r":
            self._set(roll=0.0)
        elif key == "h":
            self._set(up=-0.5)
        elif key == "j":
            self._set(up=1.0)
        elif key == "k":
            self._set(up=0.5)
        elif key == "l":
            self._set(up=0.0)
        elif key == "u":
            self._set(pitch=0.5)
        elif key == "i":
            self._set(pitch=0.0)
        elif key == "o":
            self._set(pitch=-0.5)
        elif key == "v":
            self._set(mode_mark=True, height_ctrl_mode=True)
        elif key == "b":
            self._set(mode_mark=True, height_ctrl_mode=False)
        elif key == "n":
            self._set(mode_mark=True, pitch_ctrl_mode=True)
        elif key == "m":
            self._set(mode_mark=True, pitch_ctrl_mode=False)
        elif key == "z":
            self._set(mode_mark=True, stand_mode=True)
            self._publish()
            self._set(up=1.0)
            self._publish()
        elif key == "x":
            self._set(mode_mark=True, stand_mode=False)
            self._publish()
        elif key == "c":
            self._set(mode_mark=True, jump_mode=True)
            self._publish()
        elif key == "f":
            self._set(mode_mark=True, dance_mode=True)
            self._publish()
        elif key == "g":
            self._set(mode_mark=True, dance_mode=False)
            self._publish()
        elif key in QUIT_KEYS:
            self.stopped = True
            return
        self._publish()


def read_char(fd: int) -> str:
    """Blocking single-key read in raw mode, restoring the terminal afterwards (as in the original)."""
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def key_reader(node: TeleopNode, fd: int) -> None:
    while rclpy.ok() and not node.stopped:
        key = read_char(fd)
        node.push_key(key)
        if key in QUIT_KEYS:
            break


def main(args=None) -> None:
    if termios is None or not sys.stdin.isatty():
        print("teleop_node needs an interactive POSIX terminal (run it with `ros2 run`, not `ros2 launch`)",
              file=sys.stderr)
        sys.exit(1)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(__doc__)
    print("Teleop start now!  Press '`' to exit.")

    rclpy.init(args=args)
    node = TeleopNode()
    reader = threading.Thread(target=key_reader, args=(node, fd), name="keyboard", daemon=True)
    reader.start()
    try:
        while rclpy.ok() and not node.stopped:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.try_shutdown()
        print("exit!")


if __name__ == "__main__":
    main()
