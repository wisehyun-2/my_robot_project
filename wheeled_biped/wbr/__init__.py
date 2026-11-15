"""wbr -- wheeled biped robot core (ROS-independent).

Sub-packages:
  common      filters, rotations, servo model
  sim         MuJoCo simulation backend
  models      the v5 robot's model from its Fusion export (build_v5), plant parameters (plant_params), plant check
  observers   pi3hat firmware attitude filter port, the MCU attitude filter, the IMU frame adapter (BaseOrientation)
  controllers lqi8 (serial-leg plant + LQI8 gain tables + serial-leg VMC), diablo_arch (MCU state machine / host link
              emulation)
  plans       closed_loop (the closed-loop runtime: simulator + MCU + host), plan_v5 (the v5 robot on it)
  bench       scenarios and metrics of the staged tests, CAD robot runner
  viewer      the v5 course viewer
"""
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
MODELS_DIR = PROJECT_ROOT / "models"
RESULTS_DIR = PROJECT_ROOT / "results"
