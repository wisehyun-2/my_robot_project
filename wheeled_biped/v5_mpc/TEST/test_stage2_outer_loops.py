"""Stage 2 - the other four controllers of the paper's Fig. 6 around the MPC: height (leg-length PD), yaw (PI on the
yaw rate), roll (compliant) and split angle (anti-split PD).  They are the MCU's and the same as in v5_lqi.

    python -m unittest discover -s v5_mpc/TEST -p "test_stage2*.py" -v

Pass = each loop does its job without disturbing the balance (same tests and limits as v5_lqi stage 2).
"""
from stage_common import OuterLoopTests, StageTestCase


class Stage2OuterLoops(OuterLoopTests, StageTestCase):
    STAGE = "mpc"
    GROUP = "단계 2: MPC + 바깥 루프(높이·요·롤·벌어짐)"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
