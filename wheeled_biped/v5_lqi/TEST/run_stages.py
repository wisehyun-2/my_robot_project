"""Run the LQI8 staged tests and write v5_lqi/TEST/results/stage_report.md (+ .json).

    python v5_lqi/TEST/run_stages.py --stage 4 --keep-going   # the full controller: the only stage (number 4 is kept from the
                                                              # time the pure LQR and the single integrators were stages 1 to 3)
    WBR_TEST_SEEDS=0,1,2 python v5_lqi/TEST/run_stages.py     # more noise seeds; --out <folder> writes the report elsewhere
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import stage_common as SC  # noqa: E402

MODULES = {4: "test_stage4_int_tilt"}
TITLES = {4: "전체 제어기(LQR + ∫위치 + ∫기울기)"}


def main(argv=None) -> int:
    return SC.run_stages(MODULES, TITLES, HERE / "results", SC.CONTROLLER, __doc__, argv)


if __name__ == "__main__":
    sys.exit(main())
