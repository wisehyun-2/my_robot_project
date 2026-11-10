# ros2_ws — ROS 2 workspace (empty for now)

The packages that were here were written for Plans 1-3 (2026-10-21/22): `upkie_ros2` (Plan 1, upkie spine and
balancer nodes), `diablo_lite_ros2` (Plans 2 / 3, DIABLO-lite MCU, teleop and `cmd_vel` bridge) and the message
packages `wbr_msgs`, `motion_msgs`, `ception_msgs`.  They were never built or run on Ubuntu, and on 2026-11-11 they were
removed together with Plans 1-3 (they are in the git history before that date).

The ROS 2 side of the real robot (v5: Raspberry Pi 4B + pi3hat + moteus, AKE60-8 × 6) will follow the TITA
`ros2_control` package layout on ROS 2 Jazzy (decision of 2026-11-08): the TITA packages are reused and only the
missing balance controller is replaced by a port of `wbr` (`wbr/controllers/diablo_arch/mcu.py`, `wbr/controllers/lqi8`,
`wbr/observers/body_attitude.py`), with a pi3hat / moteus hardware interface and the v5 URDF
(`models/wheelbiped_v5`).  The reference repositories are under `../../../references/` (TITA_ROS2_Control_Sim,
TITA-SDK-ROS2, TITA_CAN_Inerface).
