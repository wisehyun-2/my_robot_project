"""wbr -- wheeled biped robot core (ROS-independent).

Sub-packages:
  common      filters, rotations, servo model
  sim         MuJoCo simulation backend
  models      model builders (upkie URDF -> MJCF for the Plan 4 model, the v5 robot from its Fusion export) + plant parameters
  observers   pi3hat firmware attitude filter port, the MCU attitude filter, upkie's IMU frame adapter (BaseOrientation)
  controllers lqi8 (serial-leg plant + LQI8 gain tables + serial-leg VMC), diablo_arch (MCU state machine / host link
              emulation), mpc_host (Plan 4 stage-2 reference MPC)
  plans       plan4_lqi8 (closed-loop harness), plan_v5 (the v5 robot on it)
  bench       acceptance battery, staged tests, motor sizing, video
  viewer      live MuJoCo viewers
(Plans 1-3 -- upkie stack, DIABLO-lite + LQI6, DIABLO-lite + upkie code -- were removed on 2026-11-11; git history.)
"""
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
MODELS_DIR = PROJECT_ROOT / "models"
RESULTS_DIR = PROJECT_ROOT / "results"
