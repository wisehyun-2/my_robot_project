"""Staged tests of the MPC controller (v5_mpc): the same order as v5_lqi -- the MPC alone, the outer loops, then the
integral states (README.md).

The scenarios, metrics, limits and the report live in wbr/bench/stage_tests.py and are the same as v5_lqi/TEST's; this
file only registers the MPC stages (v5_mpc/controller.py) and re-exports the shared names for the test modules.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))        # wheeled_biped/: the v5_mpc package

from wbr.bench import stage_tests as _ST         # noqa: E402
from wbr.bench.stage_tests import *              # noqa: E402,F401,F403
from v5_mpc.controller import NAME, STAGES as _STAGES   # noqa: E402

_ST.register(*_STAGES)
CONTROLLER = NAME
