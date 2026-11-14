# wheeled_biped — the v5 wheeled biped: controller, model and MuJoCo checks

Everything lives in the ROS-free Python package `wbr` and runs on Windows/Linux/macOS with MuJoCo. The ROS 2 side of the
real robot will follow the TITA `ros2_control` layout on Jazzy (`ros2_ws/README.md`; nothing is built there yet).
Only the v5 robot is here (since 2026-11-14; what came before is at the end of this file).

| what | model | control stack | entry point |
|---|---|---|---|
| v5 robot | the user's robot from Fusion (`models/wheelbiped_v5`, AKE60-8 × 6, links 25 cm, wheel radius 0.101 m) with the real IMU chain | DIABLO-style MCU (1 kHz or 200 Hz) with the balance law LQI8 (`v5_lqi/`, in use) or an MPC in the MCU (`v5_mpc/`) | `wbr.plans.plan_v5` (`PlanV5`, on the shared runtime `wbr.plans.plan4_lqi8.Plan4LQI8`), staged tests `v5_lqi/TEST`, `v5_mpc/TEST` |

## The closed loop

Every part below is always on (the on / off switches were removed on 2026-11-14); the gains come from the robot's plant
(`wbr.bench.ake_cad.v5_gains`, written to `models/wheelbiped_v5/description.json`).

- **IMU** (`wbr/plans/plan_v5.py`): the MuJoCo site at the chip on the pi3hat -> sensor noise -> port of the pi3hat firmware
  filter (`wbr/observers/pi3hat_ars.py`) -> quaternion in the ARS frame, bias-corrected rate, bias -> sample delay -> MCU.
- **MCU estimator** (`wbr/controllers/diablo_arch/mcu.py`): the board's frame adapter (`wbr/observers/base_orientation.py`,
  `R_base_to_imu` = diag(-1, -1, 1)), the MCU's own attitude filter with the IMU position in it (`wbr/observers/body_attitude.py`),
  yaw rate from the raw gyro with an offset learned against the wheels, joint velocities from 5 ms position differences,
  NaN / stale sample guards (hold, then FAULT).
- **Balance law**: LQI8 (`wbr/controllers/lqi8/`: 8 states, gain table over the leg length, 2 m/s^2 ramp + 0.3 s pre-filter,
  integrators frozen while the reference moves) or the MPC (`v5_mpc/mpc_law.py`), in the slot `Plan4LQI8._make_law`.
- **Outer loops**: leg length (PD on the longer leg = suspension, the mean reference following the range, static leg force
  from the plant at each leg's length and lean, each leg's own damper, soft walls at the range ends), compliant roll
  (differential leg force from the IMU roll), yaw (PI on the gyro + the wheels' speed difference, ramped reference), anti-split
  PD, the knee's wheel-reaction feed-forward, speed limit and integrator hold over uneven ground, stiction compensation.
- **Safety**: FAULT at 34 deg pitch or roll, persistent NaN / stale data, a non-finite balance-law input or torque command
  (all joints torque-free); `arm()` refuses without a measurement, tilted, or with legs that do not read like legs; a lost
  host link (0.5 s) stops driving and turning, the robot keeps balancing; always in STAND, a transform-down request is refused.

## Install

```bash
python -m venv .venv && .venv/Scripts/activate      # or source .venv/bin/activate on Linux
pip install -e .
python -m wbr.models.build_v5                       # v5 URDF / MJCF / plant parameters from the Fusion export (models/wheelbiped_v5)
```

## Run

```bash
python -m unittest discover -s tests                # unit tests
python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # v5 staged tests, LQI8 (report v5_lqi/TEST/results)
python v5_mpc/TEST/run_stages.py --stage 4 --keep-going   # the same with the MPC balance law
python -m wbr.viewer.course_v5                      # v5 robot over speed bumps under one wheel: Up/Down speed, Space stop, R restart, P/O push
python -m wbr.models.verify_plan4_plant             # v5 plant against MuJoCo -> results/wheelbiped_v5_model
python -m wbr.bench.ake_cad                         # AKE motor check of robot_cad_v5 (with and without 5 kg) -> results/ake_cad_v5
```

Unit tests: `tests/test_core.py` (filters, servo, LQI8, host link) and `tests/test_leg_plant.py` (serial leg, plant, LQI8
reference shaping, acceptance metrics) need no MuJoCo; `tests/test_robot_v5.py` (model = CAD, IMU chain), `tests/test_runtime_v5.py`
(plant against MuJoCo, gain table, smoke runs, MCU guards, outer-loop mechanisms) and `tests/test_v5_mpc.py` do.
On the Windows PC Smart App Control blocks the mujoco DLLs (2026-11-14): run the MuJoCo tests on the Ubuntu laptop there.

## Layout

```
wbr/common        filters, rotations, moteus-like servo model (ports of upkie code, Apache-2.0)
wbr/sim           MuJoCo backend (servo PD at every physics substep, IMU sensors, pushes, contacts)
wbr/models        v5 model builder (Fusion -> URDF -> MJCF), plant parameters (plant_params), plant check against MuJoCo
wbr/observers     pi3hat firmware attitude filter port, MCU attitude filter, IMU frame adapter (BaseOrientation)
wbr/controllers   lqi8 (serial-leg plant, LQI8, serial-leg VMC, MCU adapter), diablo_arch (host link, MCU program)
wbr/plans         plan4_lqi8 (closed-loop runtime), plan_v5 (the v5 robot on it), common (delays, noise, logs)
wbr/bench         scenarios and metrics of the acceptance criteria, staged tests, CAD robot motor check (ake_cad, ake_sizing)
wbr/viewer        the v5 course viewer
v5_lqi/, v5_mpc/  the v5 robot's two balance controllers and their staged tests
tests/            unit tests
models/           robot description (wheelbiped_v5)
ros2_ws/          plan of the ROS 2 packages (README only)
docs/             see below
results/          see below
```

Documents (`docs/`):
- `robot_v5_model.md` — the v5 model, IMU chain and outer loops: records and open items. `bringup_step0.md` — the real
  robot's set-up order, signs, zeros, IMU test. `합격기준.md` — the acceptance criteria the staged tests judge against.
- `plan4_model.md` (plant derivation and its check against MuJoCo) and `plan4_lqi8.md` (LQI8 design): written for the upkie
  model of Plan 4 with its numbers; the method is what v5 uses.
- Records: `HANDOFF.md` (hand-over 10-22 to 11-11), `비교보고서.md` (Plans 1-3), `plan2_15kg_torque_sizing.md` (Plan 2),
  `session_log_2026-10-21.md`, `balancing_robot_design/` (the two-wheel robot of 10-21).

Results (`results/`): `wheelbiped_v5_model` (plant check), `ake_cad_v5` (motor check); the staged-test reports are in
`v5_lqi/TEST/results` and `v5_mpc/TEST/results`. Records of Plans 1-3: `bench`, `torque_sizing`, `figures`, `videos`.

## History (all in the git history)

- 2026-11-11: Plans 1-3 (upkie stack, DIABLO-lite + LQI6, DIABLO-lite + upkie code), their benchmark, models and ROS 2
  packages were removed.
- 2026-11-14 (user: the robot is new hardware, upkie is not needed): the upkie model (`models/upkie_description`,
  `upkie_plan4`, `upkie_step`, `wbr/models/build_upkie.py`), Plan 4's acceptance battery runner, its stage-2 MPC host, live
  viewer and video, the motor / wheel-diameter study on the upkie legs and the upkie route of `ake_cad` were removed.
  Renamed: `wbr/models/upkie_params.py` -> `plant_params.py`, `wbr/observers/upkie_observers.py` -> `base_orientation.py`,
  `tests/test_plan4*.py` -> `tests/test_leg_plant.py` and `tests/test_runtime_v5.py`.
- 2026-11-14, second step (user: remove what the v5 robot does not use): the MCU's CAR / transition modes, motors-off after a
  host-link loss, leg-length-difference roll and `F_ff / cos(lean)` leg force, the on / off switches of the v5 features
  (`roll_compliant`, `wheel_reaction_ff`, `exact_leg_force`, `suspension`, `mean_follows_range`, `uneven_hold_int`, attitude filter
  off, `imu_model`), the LQI8's state predictor, plain tracking of a host-shaped reference, `a_ff`, second-order pre-filter and
  `quiet_boost`, the mass / friction scales, `ake_cad --wheel-reaction / --roll / --yaw-wheel`, and Plan 4's other records
  (reviews, `plan4_mpc_host.md`, `ake_motor_sizing.md`, `results/acceptance`, `plan4_*`, `ake_sizing`, `ake_cad` v2-v4, videos).
  The MCU + LQI8 and `MpcLaw.limits` gave bit-identical outputs before and after on a 3000-step synthetic input.
  The "switched off" rows of `v5_lqi/TEST/README.md` (section 5) cannot be reproduced any more.
