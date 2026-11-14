"""wbr -- wheeled biped robot core (ROS-independent).

Sub-packages:
  common      filters, rotations, servo model
  sim         MuJoCo simulation backend
  models      the v5 robot's model from its Fusion export (build_v5), plant parameters (plant_params), plant check
  observers   pi3hat firmware attitude filter port, the MCU attitude filter, the IMU frame adapter (BaseOrientation)
  controllers lqi8 (serial-leg plant + LQI8 gain tables + serial-leg VMC), diablo_arch (MCU state machine / host link
              emulation)
  plans       plan4_lqi8 (closed-loop runtime shared by the robots), plan_v5 (the v5 robot on it)
  bench       scenarios and metrics of the acceptance criteria, staged tests, CAD robot runner
  viewer      the v5 course viewer
(Plans 1-3 -- upkie stack, DIABLO-lite + LQI6, DIABLO-lite + upkie code -- were removed on 2026-11-11; Plan 4's upkie model,
its acceptance battery and its stage-2 MPC host on 2026-11-14; git history.)
"""
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
MODELS_DIR = PROJECT_ROOT / "models"
RESULTS_DIR = PROJECT_ROOT / "results"
