# wheeled_biped — 6-motor wheeled biped: controller, models and MuJoCo checks

Everything lives in the ROS-free Python package `wbr` and runs on Windows/Linux/macOS with MuJoCo. The ROS 2 side of the
real robot will follow the TITA `ros2_control` layout on Jazzy (`ros2_ws/README.md`; nothing is built there yet).

| what | robot model | control stack | entry point |
|---|---|---|---|
| v5 robot | the user's robot from Fusion (`models/wheelbiped_v5`, AKE60-8 × 6, links 25 cm) with the real IMU chain (pi3hat filter port + MCU attitude filter) | DIABLO-style MCU (1 kHz or 200 Hz, modes, MotionCtrl, 500 ms timeout), height / yaw / roll / split loops, serial-leg VMC; balance law LQI8 (`v5_lqi/`) or an MPC in the MCU (`v5_mpc/`) | `wbr.plans.plan_v5` (`PlanV5`, on the shared runtime `wbr.plans.plan4_lqi8`), staged tests `v5_lqi/TEST`, `v5_mpc/TEST` |

Removed (all in the git history):
- 2026-11-11: Plans 1-3 (upkie stack, DIABLO-lite + LQI6, DIABLO-lite + upkie code), their benchmark, models and ROS 2
  packages; the comparison is kept in `docs/비교보고서.md` and `results/bench`.
- 2026-11-14 (user: the robot is new hardware, upkie is not needed): Plan 4's upkie model (`models/upkie_description`,
  `upkie_plan4`, `upkie_step`, `wbr/models/build_upkie.py`), its acceptance battery runner, its stage-2 MPC host
  (`wbr/controllers/mpc_host`, `wbr/bench/acceptance_mpc.py`), its live viewer and video, the motor / wheel-diameter study on the
  upkie legs (`python -m wbr.bench.ake_sizing`) and the upkie route of `wbr.bench.ake_cad`. Renamed: `wbr/models/upkie_params.py`
  → `plant_params.py`, `wbr/observers/upkie_observers.py` → `base_orientation.py`; `tests/test_plan4*.py` →
  `tests/test_leg_plant.py` (no MuJoCo) and `tests/test_runtime_v5.py` (on the v5 model).
- 2026-11-14, second step (user: remove what the v5 robot does not use): in the MCU the CAR / transition modes, the
  motors-off after a host-link loss, the leg-length-difference roll, the F_ff / cos(lean) leg force and the on / off switches
  of the v5 features (compliant roll, wheel-reaction feed-forward, suspension, mean length following the range, uneven-ground
  integrator hold, attitude filter: always on now); in the LQI8 the state predictor, the plain tracking of a host-shaped
  reference, a_ff, the second-order pre-filter and quiet_boost; in the plans the mass / friction scales, the ideal-IMU option
  and the base-class methods every robot replaces; `ake_cad --wheel-reaction / --roll / --yaw-wheel`.  Plan 4's records went
  too: `docs/plan4_mpc_host.md`, the review documents, `docs/ake_motor_sizing.md`, `results/acceptance`, `results/plan4_*`,
  `results/ake_sizing`, `results/ake_cad*` (v2-v4) and the Plan 4 videos.  Kept because v5 builds on them:
  `docs/plan4_model.md` (the plant derivation) and `docs/plan4_lqi8.md` (the LQI8 design).  The name `plan4_lqi8` /
  `Plan4LQI8` stays for the runtime the v5 plan is built on.

## Install

```bash
python -m venv .venv && .venv/Scripts/activate      # or source .venv/bin/activate on Linux
pip install -e .
python -m wbr.models.build_v5                       # v5 URDF / MJCF / plant parameters from the Fusion export (models/wheelbiped_v5)
```

## Run

```bash
python -m unittest discover -s tests                # unit tests (tests/test_core.py and tests/test_leg_plant.py need no MuJoCo)
python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # v5 staged tests, LQI8 (report v5_lqi/TEST/results)
python v5_mpc/TEST/run_stages.py --stage 4 --keep-going   # the same with the MPC balance law
python -m wbr.viewer.course_v5                      # v5 robot over speed bumps under one wheel: Up/Down speed, Space stop, R restart, P/O push
python -m wbr.models.verify_plan4_plant             # v5 plant against MuJoCo -> results/wheelbiped_v5_model
python -m wbr.bench.ake_cad                         # AKE motor check of robot_cad_v5 (with and without 5 kg) -> results/ake_cad_v5
```

## Layout

```
wbr/common        filters, rotations, moteus-like servo model (ports of upkie code, Apache-2.0)
wbr/sim           MuJoCo backend (servo PD at every physics substep, IMU sensors, pushes, contacts)
wbr/models        v5 model builder (Fusion -> URDF -> MJCF), plant parameters, plant check against MuJoCo
wbr/observers     pi3hat firmware attitude filter port, MCU attitude filter, IMU frame adapter (BaseOrientation)
wbr/controllers   lqi8 (serial-leg plant, LQI8, serial-leg VMC), diablo_arch (host link, MCU state machine)
wbr/plans         plan4_lqi8 (closed-loop runtime), plan_v5 (v5 robot), common (delays, noise, logs)
wbr/bench         scenarios and metrics of the acceptance criteria, staged tests, CAD robot motor check
wbr/viewer        the v5 course viewer
v5_lqi/, v5_mpc/  the v5 robot's two balance controllers and their staged tests
models/           robot description (wheelbiped_v5)
results/          outputs (staged tests, plant check, motor checks) and the records of removed plans, figures, videos
docs/             reports and reviews
```
