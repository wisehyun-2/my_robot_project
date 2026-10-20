# balancing_robot_design

Self-balancing robot (ROS 2 + Teensy, direct-drive wheels) design study.

| path | what |
|---|---|
| `설계보고서.md` | **main report (Korean)**: reference-project analysis, ROS 2 latency fact-check, hardware architecture, controller selection with numbers |
| `제어기_비교보고서.md` | **controller comparison report (Korean)**: survey of all open-source wheel-leg balance code, SIL benchmark of the 9 Ascento-like (4-motor) projects + upkie (§6) + the 9 five-bar/serial six-motor projects (§8) against my controller |
| `시뮬레이션_환경조사.md` | **simulation-environment survey (Korean)**: which surveyed repos ship a physics simulator (MuJoCo / PyBullet / Webots / Simscape), what is missing, what runs on Ubuntu 24.04 (verified runs of fern, WheelLeg_LQR, kuafu, ATWheelLeg core) and install recipes |
| `sil/` | 4-motor SIL harness: every ported balance law as C (`controllers/*.c`, one `.so` each) or Python (upkie MPC), `sil_harness.py`, `make_report.py`, `results_{small,large}.json` |
| `sim6/` | six-motor (five-bar leg) SIL: three-body wheel-leg plant `plant6.py` (validated against USTB's published gains), ported gain tables `controllers6.py`, `sil6_harness.py`, `make_report6.py`, `results6_<plant>.json` |
| `sim/twip_model.py` | two-wheeled inverted pendulum model (nonlinear + linearised), three parameter sets |
| `sim/analysis_controllers.py` | LQR/LQI design, cascade-PID equivalence, loop-rate x latency stability map, time responses, torque sizing |
| `sim/mpc_compare.py` | OSQP linear MPC vs saturated LQR (tilt, push, speed-limited velocity steps), Teensy cost estimate |
| `sim/sizing_options.py` | per-candidate-motor recovery limits for the 5 kg and 10 kg robot classes (`python3 sizing_options.py small|large`) |
| `sim/gen_gains.py` | prints LQI gains + C constants for any robot parameters (`python3 gen_gains.py --M_b 7 --l 0.25 ...`) |
| `firmware/teensy_balance/teensy_balance.ino` | Teensy 4.1 balance-loop skeleton (1 kHz LQI, complementary filter, CAN torque adapters, watchdogs) |
| `figures/` | plots referenced by the report |
| `results/` | JSON outputs of the simulations (gains, margins, metrics) |

Re-run everything:

```bash
pip install numpy scipy matplotlib control osqp sympy qpmpc proxsuite
cd sim && python3 analysis_controllers.py && python3 mpc_compare.py && python3 gen_gains.py
cd ../sil && python3 sil_harness.py --robot small --out results_small.json && python3 sil_harness.py --robot large --out results_large.json && python3 make_report.py
cd ../sim6 && for p in rm liu sky small large; do python3 sil6_harness.py --plant $p --scale --out results6_$p.json; done && python3 make_report6.py
```
