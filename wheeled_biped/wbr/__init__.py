"""wbr -- wheeled biped robot core (ROS-independent).

Sub-packages:
  common      filters, rotations, servo model
  sim         MuJoCo simulation backend
  models      MJCF builders (upkie from URDF, diablo_lite parametric five-bar)
  observers   ports of upkie observers (base orientation, floor contact, wheel odometry) + IMU estimator
  controllers upkie (wheel balancer PI, MPC, gyropod), lqi6 (plant6 + online dLQR + five-bar VMC),
              diablo_arch (MCU state machine / host link emulation)
  plans       plan1_upkie, plan2_diablo_lqi6, plan3_diablo_upkie assemblies
  bench       test battery and reports
  viewer      live MuJoCo viewer
"""
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
MODELS_DIR = PROJECT_ROOT / "models"
RESULTS_DIR = PROJECT_ROOT / "results"
