# ros2_ws -- ROS 2 packages of the wheeled biped project

ROS 2 Jazzy (Ubuntu 24.04; the code is Humble-compatible) glue around the ROS-independent core
package `wbr` (`../wbr`): MuJoCo simulation, upkie observers/controllers, the DIABLO-style MCU and
the LQI6 law all live in `wbr`; the nodes here only translate between ROS messages and `wbr`.

| Plan | Robot / architecture | Packages | Nodes (executables) |
|------|----------------------|----------|---------------------|
| 1 | Upkie (`upkie_description`), upkie control stack re-architected: 1 kHz spine, balancer in or out of the spine | `upkie_ros2/upkie_spine`, `upkie_ros2/upkie_balancer`, `upkie_ros2/upkie_bringup`, `upkie_ros2/upkie_description` (wrapper), `wbr_msgs` | `spine_node` (MuJoCo + observers, optional embedded WheelBalancer PI), `balancer_node` (WheelBalancer PI or MPC) |
| 2 | DIABLO-lite (6 kg five-bar), DIABLO architecture + LQI6 law of the design report | `diablo_lite_ros2/diablo_lite_mcu`, `diablo_lite_ros2/diablo_lite_ctrl`, `diablo_lite_ros2/diablo_lite_bringup`, `diablo_lite_ros2/diablo_lite_description`, `motion_msgs`, `ception_msgs` | `mcu_node` (`balancer:=lqi6`), `cmd_vel_bridge`, `teleop_node` |
| 3 | DIABLO-lite, DIABLO architecture + upkie control code (wheel balancer PI or MPC) | same as Plan 2 | `mcu_node` (`balancer:=upkie_pid` or `upkie_mpc`), `cmd_vel_bridge`, `teleop_node` |

Message packages:

* `wbr_msgs` -- `ServoCommand`, `ServoCommands`, `ServoState`, `SpineObservation` (Plan 1 spine interface).
* `motion_msgs`, `ception_msgs` -- verbatim copies of the DIABLO interface packages
  (`references/diablo_ros2/diablo_interfaces`), so DIABLO tools (teleop, rviz plugin, ...) stay compatible.

## 1. Prerequisites

```bash
# ROS 2 Jazzy desktop (ros-jazzy-desktop brings rviz2, robot_state_publisher, xacro, ...)
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep \
                 ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher-gui xterm
```

### Install the `wbr` core into the Python used by ROS

`wbr` (and its dependencies: `mujoco`, `numpy`, `scipy`, `qpsolvers`, `qpmpc`, `daqp`, `osqp`, ...)
must be importable by the **same interpreter** that runs the ROS 2 nodes (`/usr/bin/python3`, 3.12).
Two options:

```bash
cd <project>/wheeled_biped          # the directory that contains pyproject.toml and wbr/

# (a) recommended: a virtual environment that can still see the ROS Python packages
python3 -m venv --system-site-packages ~/wbr_venv
source ~/wbr_venv/bin/activate
pip install -e .
# keep the venv activated when running colcon build and the nodes below

# (b) system Python (PEP 668 requires the flag on Ubuntu 24.04); pip may then replace the apt numpy
#     1.x by numpy 2.x and break ROS packages built against it (cv_bridge, ...)
pip install -e . --break-system-packages
```

(`pip install -e ../` from inside `ros2_ws/` is the same thing.)  The editable install means that
`wbr.MODELS_DIR` points at `<project>/wheeled_biped/models`, where the MuJoCo scenes
(`models/upkie/scene.xml`, `models/diablo_lite/scene.xml`) are loaded from.

## 2. Build

```bash
source /opt/ros/jazzy/setup.bash
cd <project>/wheeled_biped/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y     # ROS dependencies (pip packages are not rosdep keys)
colcon build --symlink-install
source install/setup.bash
```

Notes:

* `upkie_ros2/upkie_description` is a thin ament_cmake wrapper: it installs the meshes / URDF / xacro
  of the vendored upstream package `models/upkie_description` (a catkin package, left untouched).
  Its CMake variable `UPKIE_DESCRIPTION_SOURCE_DIR` defaults to `../../../../models/upkie_description`
  relative to the wrapper; override it with `colcon build --cmake-args -DUPKIE_DESCRIPTION_SOURCE_DIR=<path>`
  if the layout changes.  Do **not** symlink `models/upkie_description` into `src/` (duplicate package).
* `upkie.xacro` includes its parts relative to the package root (`xacro/utils/...`), which ROS
  `xacro` cannot resolve from `xacro/upkie.xacro`; the wrapper therefore also installs a copy at the
  share root and the launch file processes `$(ros2 pkg prefix --share upkie_description)/upkie.xacro`.
  `use_xacro:=false` falls back to the pre-generated `urdf/upkie.urdf`.
* With `--symlink-install`, edits to the Python nodes, launch and YAML files are live (no rebuild).

## 3. Run

### Plan 1 -- Upkie

```bash
# spine with the embedded wheel-balancer PI (upkie's spine pipeline), robot_state_publisher, rviz
ros2 launch upkie_bringup sim.launch.py                       # = balancer:=pid embedded:=true rviz:=true

# spine + external balancer node (PI or MPC) over /upkie/observation and /upkie/servo_commands
ros2 launch upkie_bringup sim.launch.py embedded:=false balancer:=pid
ros2 launch upkie_bringup sim.launch.py embedded:=false balancer:=mpc   # the launch file switches the spine to
                                                                        # ground-truth wheel contact: upkie's MPC
                                                                        # needs a contact flag and the apparent-inertia
                                                                        # observer does not trigger on a robot at rest

# drive it (teleop_twist_keyboard in an xterm; or run it yourself in another terminal)
ros2 launch upkie_bringup teleop.launch.py speed:=0.3 turn:=0.6
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.6

# after a fall: put the robot back on its wheels
ros2 service call /spine_node/reset std_srvs/srv/Trigger
```

Parameters: `upkie_bringup/config/spine.yaml` (`spine_hz`, `scene`, `latency_ms`, `noise`,
`observation_hz`, `publish_every_cycle`, `command_timeout_ms`, ...) and `config/balancer.yaml`
(`balancer`, `observation_hz` -- must equal the spine's observation rate --, `mpc_hz`, `gain_scale`,
`mpc_leg_length`, `left_wheeled`).  `spine_node` logs the wheel direction signs of the model at
start-up; `left_wheeled` of the balancer must agree with them.

Topics / services of Plan 1:

| Name | Type | Direction | Rate | Notes |
|------|------|-----------|------|-------|
| `/upkie/observation` | `wbr_msgs/SpineObservation` | spine -> balancer | `observation_hz` (200 Hz) or every cycle | base pitch, body angular velocity, wheel odometry, floor contact, servo states, IMU |
| `/upkie/servo_commands` | `wbr_msgs/ServoCommands` | balancer -> spine | per observation | moteus-style commands; 100 ms watchdog -> zero torque |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop -> spine (embedded) or balancer | on change | `linear.x` ground velocity [m/s], `angular.z` yaw velocity [rad/s] |
| `/joint_states` | `sensor_msgs/JointState` | spine -> robot_state_publisher | 50 Hz | left/right hip, knee, wheel |
| `/imu/data` | `sensor_msgs/Imu` | spine | 200 Hz | IMU on the `base` site |
| `/tf` | `odom -> base` | spine | 50 Hz | ground-truth base pose (`publish_tf`) |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | latched | from `upkie.xacro` |
| `/spine_node/reset` | `std_srvs/Trigger` | service | | reset to the standing pose |

### Plans 2 and 3 -- DIABLO-lite

```bash
ros2 launch diablo_lite_bringup sim.launch.py balancer:=lqi6        # Plan 2
ros2 launch diablo_lite_bringup sim.launch.py balancer:=upkie_pid   # Plan 3, wheel balancer PI
ros2 launch diablo_lite_bringup sim.launch.py balancer:=upkie_mpc   # Plan 3, MPC balancer

# drive it through /cmd_vel (cmd_vel_bridge is started by the launch file, bridge:=true) ...
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.6
# ... or with the DIABLO key map (port of diablo_teleop; needs its own terminal, publishes at 25 Hz)
ros2 launch diablo_lite_bringup sim.launch.py balancer:=lqi6 bridge:=false
ros2 run diablo_lite_ctrl teleop_node

# URDF only (sliders)
ros2 launch diablo_lite_description display.launch.py

# after a fall
ros2 service call /mcu_node/reset std_srvs/srv/Trigger
```

Parameters: `diablo_lite_bringup/config/mcu.yaml` (`balancer`, `mcu_hz`, `latency_ms`, `noise`,
`host_timeout_ms`, `start_mode`, `initial_up`, Plan 3 gains, publish rates, frames).

Topics / services of Plans 2 and 3 (DIABLO topic structure):

| Name | Type | Direction | Rate | Notes |
|------|------|-----------|------|-------|
| `/diablo/MotionCmd` | `motion_msgs/MotionCtrl` | host -> MCU | 25 Hz teleop / 180 ms heartbeat | `value.forward` [m/s], `value.left` [rad/s], `value.up` height, `value.roll/pitch`; `mode_mark=true` carries only the mode flags |
| `/diablo/sensor/Imu` | `sensor_msgs/Imu` | MCU | 50 Hz | |
| `/diablo/sensor/ImuEuler` | `ception_msgs/IMUEuler` | MCU | 50 Hz | roll, pitch, yaw |
| `/diablo/sensor/Motors` | `motion_msgs/LegMotors` | MCU | 50 Hz | hip / knee (= crank motor) / wheel position, velocity, `iq` (torque in simulation), leg lengths |
| `/diablo/sensor/Body_state` | `motion_msgs/RobotStatus` | MCU | 10 Hz | `ctrl_mode_msg` (0 RC, 1 host), `robot_mode_msg` (2 CAR, 3 STAND, 4/5 transitions), `error_msg` bit 0 = pitch fault |
| `/joint_states` | `sensor_msgs/JointState` | MCU | 50 Hz | 10 joints incl. passive `*_knee = crank - hip`, `*_rod = hip - crank` |
| `/tf` | `odom -> base_link` | MCU | 50 Hz | ground-truth base pose |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop -> `cmd_vel_bridge` | on change | converted to `MotionCtrl`, re-sent every 180 ms |
| `/mcu_node/reset` | `std_srvs/Trigger` | service | | reset to the equilibrium standing pose |

Host link semantics: each `/diablo/MotionCmd` message is one serial packet for the MCU
(`wbr.controllers.diablo_arch.host_link.HostLink.receive`).  Without a packet for
`host_timeout_ms` (500 ms) the MCU releases control and runs its disconnect program (zero velocity,
posture held) -- kill `cmd_vel_bridge` / `teleop_node` and the robot stops within 0.5 s.  While the
bridge is alive its 180 ms heartbeat keeps the link up, so a silent `/cmd_vel` publisher is covered by
the bridge's own `cmd_vel_timeout_ms` (500 ms, then zero velocities).  Packets are queued and applied
in order (a mode packet followed immediately by a movement packet is never lost).  The plan's internal
`HostEmulator` is disabled in the node because it would keep the link alive by itself.

Safety behaviour of the MCU program (DIABLO-lite has no auxiliary wheels): `mode_mark=true,
stand_mode=false` (DIABLO teleop keys `v b n m c f g` before `z`) is ignored instead of sitting down;
the MCU starts disarmed and arms itself once |pitch| < 3 deg; |pitch| > 34 deg is a latched FAULT with
every joint torque-free (`error_msg` bit 0; bit 1 = disarmed); `jump_mode`, `split_mode` and
`value.leg_split` are accepted but not implemented.

## 4. Real-time behaviour

The 1 kHz loops (`spine_node`, `mcu_node`) run in a background thread paced with
`time.perf_counter()`; the ROS callbacks and telemetry timers run in a `MultiThreadedExecutor`.
If the loop falls more than 100 ms behind (slow machine, MPC solve) it resynchronises and a warning
with the achieved rate is printed every 5 s.  For a steadier loop on Linux:
`sudo setcap cap_sys_nice+ep` is not needed, but `chrt -f 50 ros2 run ...` or isolating a core helps.

## 5. Layout

```
ros2_ws/src
  wbr_msgs/                      ament_cmake, Plan 1 spine messages
  motion_msgs/ ception_msgs/     ament_cmake, DIABLO interface copies
  upkie_ros2/
    upkie_description/           ament_cmake wrapper installing ../../../../models/upkie_description
    upkie_spine/                 spine_node
    upkie_balancer/              balancer_node
    upkie_bringup/               sim.launch.py, teleop.launch.py, config/*.yaml, rviz/upkie.rviz
  diablo_lite_ros2/
    diablo_lite_description/     urdf/diablo_lite.urdf (copy of models/diablo_lite), rviz, display.launch.py
    diablo_lite_mcu/             mcu_node
    diablo_lite_ctrl/            teleop_node, cmd_vel_bridge
    diablo_lite_bringup/         sim.launch.py, config/mcu.yaml
```
