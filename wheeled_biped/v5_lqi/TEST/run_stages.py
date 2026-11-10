"""Run the LQI8 staged tests in order and write v5_lqi/TEST/results/stage_report.md (+ .json).

    python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # the full controller (the one in use)
    python v5_lqi/TEST/run_stages.py                 # stages 1 -> 4, stops at the first stage that fails
    python v5_lqi/TEST/run_stages.py --stage 1 2     # only these stages
    python v5_lqi/TEST/run_stages.py --keep-going    # run every stage even after a failure
    set WBR_TEST_SEEDS=0,1,2  (PowerShell: $env:WBR_TEST_SEEDS = "0,1,2")   # more noise seeds

A stage builds on the one before it, so a failure stops the run by default: fix that stage first.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import stage_common as SC  # noqa: E402

MODULES = {1: "test_stage1_lqr", 2: "test_stage2_outer_loops", 3: "test_stage3_int_position", 4: "test_stage4_int_tilt"}
TITLES = {1: "순수 LQR", 2: "바깥 루프(높이·요·롤·벌어짐)", 3: "+ ∫위치", 4: "+ ∫기울기"}


def main(argv=None) -> int:
    return SC.run_stages(MODULES, TITLES, HERE / "results", SC.CONTROLLER, __doc__, argv)


if __name__ == "__main__":
    sys.exit(main())
