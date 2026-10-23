# wheeled_biped — 6-motor wheeled biped: three plans compared in MuJoCo

Everything that does not need ROS 2 lives in the Python package `wbr` and runs on Windows/Linux/macOS
with MuJoCo. The ROS 2 packages in `ros2_ws/src` wrap the same code for Ubuntu 24.04 + ROS 2 Jazzy
(see `ros2_ws/README.md`); they cannot be built on Windows.

| plan | robot model | control stack | entry point |
|---|---|---|---|
| 1 | Upkie (`models/upkie_description` → `models/upkie/upkie.xml`, built from the URDF) | upkie spine/agent structure: observers (base orientation, floor contact, wheel odometry) + WheelBalancer PI (1 kHz) or MPC balancer (200 Hz), wheels in velocity mode, legs in position mode | `wbr.plans.plan1_upkie` |
| 2 | DIABLO-lite, 6 kg (`models/diablo_lite`, parallelogram five-bar legs like DIABLO) | DIABLO architecture (1 kHz MCU program with modes, MotionCtrl commands, 500 ms timeout / 180 ms heartbeat) + the LQI6 controller of the design report (online dLQR on the three-body plant, VMC leg force, anti-split PD) | `wbr.plans.plan2_diablo_lqi6` |
| 3 | DIABLO-lite, 6 kg | DIABLO architecture + upkie's control code (WheelBalancer PI / MPC, upkie observers, velocity-mode wheels, position-mode hips) | `wbr.plans.plan3_diablo_upkie` |

## Install

```bash
python -m venv .venv && .venv/Scripts/activate      # or source .venv/bin/activate on Linux
pip install -e .
python -m wbr.models.build_upkie                    # URDF -> MJCF (models/upkie)
python -m wbr.models.build_diablo_lite              # parametric MJCF + URDF (models/diablo_lite)
```

## Run

```bash
python -m wbr.viewer.live --plan 1 [--balancer pid|mpc]   # live MuJoCo window, arrow keys drive, P pushes, R resets
python -m wbr.viewer.live --plan 2
python -m wbr.viewer.live --plan 3
python -m wbr.bench.run_bench                       # full test battery -> results/bench/summary.md (+ json)
python -m wbr.bench.plots                           # figures -> results/figures
python -m wbr.bench.video --plan 2                  # demo video -> results/videos
python -m wbr.bench.torque_sizing                   # Plan 2 motor torque sizing at 15 kg -> results/torque_sizing (docs/plan2_15kg_torque_sizing.md)
```

## Layout

```
wbr/common        filters, rotations (upkie pitch convention), moteus-like servo model
wbr/sim           MuJoCo backend (servo PD at every physics substep, IMU sensors, pushes, contacts)
wbr/models        model builders (upkie URDF->MJCF, DIABLO-lite five-bar MJCF/URDF)
wbr/observers     ports of the upkie observers
wbr/controllers   upkie (wheel balancer, MPC, gyropod mapping), lqi6 (plant6, LQI6, five-bar VMC),
                  diablo_arch (host link semantics, MCU state machine, upkie law plug-in)
wbr/plans         plan assemblies with a common API: reset(pitch), step(v, w), push(F, dt), set_height_offset(dh)
wbr/bench         test battery, runner, plots, video
wbr/viewer        live viewer
models/           robot descriptions (upkie_description is Apache-2.0, upkie project)
ros2_ws/src       ROS 2 packages (Jazzy): wbr_msgs, motion_msgs/ception_msgs (DIABLO-compatible), upkie_*, diablo_lite_*
results/          benchmark outputs, figures, videos
docs/             comparison report
```
