"""Run the MPC staged tests and write v5_mpc/TEST/results/stage_report.md (+ .json).

    python v5_mpc/TEST/run_stages.py --stage 4 --keep-going   # the full controller (MPC + both integral states): the only stage
    WBR_TEST_SEEDS=0,1,2 python v5_mpc/TEST/run_stages.py     # more noise seeds; --out <folder> writes the report elsewhere

Same scenarios and limits as v5_lqi/TEST/run_stages.py: the two reports compare line by line.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import stage_common as SC  # noqa: E402

MODULES = {4: "test_stage4_int_tilt"}
TITLES = {4: "전체 제어기(MPC + ∫위치 + ∫기울기)"}


def main(argv=None) -> int:
    return SC.run_stages(MODULES, TITLES, HERE / "results", SC.CONTROLLER, __doc__, argv)


if __name__ == "__main__":
    sys.exit(main())
