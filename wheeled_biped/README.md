# wheeled_biped — 6-motor wheeled biped: controller, models and MuJoCo checks

Everything lives in the ROS-free Python package `wbr` and runs on Windows/Linux/macOS with MuJoCo. The ROS 2 side of the
real robot will follow the TITA `ros2_control` layout on Jazzy (`ros2_ws/README.md`; nothing is built there yet).

| what | robot model | control stack | entry point |
|---|---|---|---|
| Plan 4 harness | upkie hardware (`models/upkie_plan4`: upkie_description with 112 mm tyres, qdd100 / mj5208 limits) | DIABLO-style MCU (1 kHz or 200 Hz, modes, MotionCtrl, 500 ms timeout) + LQI8 (8 states, gain table over leg length) + serial-leg VMC; stage 2: host MPC reference generator | `wbr.plans.plan4_lqi8` (`Plan4LQI8`), acceptance battery `wbr.bench.acceptance` |
| v5 robot | the user's robot from Fusion (`models/wheelbiped_v5`, AKE60-8 × 6, links 25 cm) with the real IMU chain (pi3hat filter port + MCU attitude filter) | the same MCU and outer loops; balance law LQI8 (`v5_lqi/`) or an MPC in the MCU (`v5_mpc/`) | `wbr.plans.plan_v5` (`PlanV5`), staged tests `v5_lqi/TEST`, `v5_mpc/TEST` |

Plans 1-3 (upkie stack, DIABLO-lite + LQI6, DIABLO-lite + upkie code) and their benchmark, models and ROS 2 packages
were removed on 2026-11-11; the comparison is kept in `docs/비교보고서.md` and `results/bench`, the code in the git history.

## Install

```bash
python -m venv .venv && .venv/Scripts/activate      # or source .venv/bin/activate on Linux
pip install -e .
python -m wbr.models.build_upkie --out models/upkie_plan4 --tire-radius 0.056 --tire-shape ellipsoid --tire-condim 4
python -m wbr.models.upkie_params                   # Plan 4 plant parameters (models/upkie_plan4/plan4_params.json)
python -m wbr.models.build_v5                       # v5 URDF / MJCF / plant parameters from the Fusion export (models/wheelbiped_v5)
```

## Run

```bash
python -m unittest discover -s tests                # unit tests
python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # v5 staged tests, LQI8 (report v5_lqi/TEST/results)
python v5_mpc/TEST/run_stages.py --stage 4 --keep-going   # the same with the MPC balance law
python -m wbr.viewer.course_v5                      # v5 robot over speed bumps under one wheel: Up/Down speed, Space stop, R restart, P/O push
python -m wbr.viewer.live --plan 4 [--balancer ramp|mpc]   # Plan 4 live window, arrow keys drive, P pushes, R resets
python -m wbr.bench.acceptance --rate both --matrix full --seeds 5   # Plan 4 acceptance (docs/합격기준.md) -> results/acceptance
python -m wbr.bench.video --plan 4                  # Plan 4 demo video -> results/videos
python -m wbr.bench.ake_sizing                      # AKE motor / wheel sizing on the upkie geometry (docs/ake_motor_sizing.md)
python -m wbr.bench.ake_cad --cad ../robot_cad_v5/calc/mass_properties.json --out results/ake_cad_v5   # motor re-check of a CAD version
```

## Layout

```
wbr/common        filters, rotations (upkie pitch convention), moteus-like servo model
wbr/sim           MuJoCo backend (servo PD at every physics substep, IMU sensors, pushes, contacts)
wbr/models        model builders (upkie URDF->MJCF, v5 from Fusion), plant parameters, plant check against MuJoCo
wbr/observers     pi3hat firmware attitude filter port, MCU attitude filter, upkie IMU frame adapter
wbr/controllers   lqi8 (serial-leg plant, LQI8, serial-leg VMC), diablo_arch (host link, MCU state machine), mpc_host
wbr/plans         plan4_lqi8 (closed-loop harness), plan_v5 (v5 robot), common (delays, noise, logs)
wbr/bench         acceptance battery, staged tests, motor sizing, video
wbr/viewer        live viewers
v5_lqi/, v5_mpc/  the v5 robot's two balance controllers and their staged tests
models/           robot descriptions (upkie_description is Apache-2.0, upkie project; upkie_plan4; wheelbiped_v5)
results/          outputs (acceptance, staged tests, motor sizing, the old Plan 1-3 benchmark), figures, videos
docs/             reports and reviews
```
