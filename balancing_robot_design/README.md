# balancing_robot_design

Self-balancing robot (ROS 2 + Teensy, direct-drive wheels) design study.

| path | what |
|---|---|
| `설계보고서.md` | **main report (Korean)**: reference-project analysis, ROS 2 latency fact-check, hardware architecture, controller selection with numbers |
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
pip install numpy scipy matplotlib control osqp
cd sim && python3 analysis_controllers.py && python3 mpc_compare.py && python3 gen_gains.py
```
