"""Staged tests of the LQI8 controller (v5_lqi): LQR first, integrators later (README.md).

The scenarios, metrics, limits and the report live in wbr/bench/stage_tests.py and are the same for v5_mpc/TEST;
this file only registers the LQI stages (v5_lqi/controller.py) and re-exports the shared names for the test modules.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))        # wheeled_biped/: the v5_lqi package

from wbr.bench import stage_tests as _ST         # noqa: E402
from wbr.bench.stage_tests import *              # noqa: E402,F401,F403
from v5_lqi.controller import NAME, STAGES as _STAGES   # noqa: E402

_ST.register(*_STAGES)
CONTROLLER = NAME
