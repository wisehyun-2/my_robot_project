"""Stage 2 - the other four controllers of the paper's Fig. 6 on top of the pure LQR: height (leg-length PD), yaw
(PI on the yaw rate), roll (leg-length difference) and split angle (anti-split PD).

    python -m unittest discover -s TEST -p "test_stage2*.py" -v        (about 15 s)

Pass = each loop does its job without disturbing the balance: the leg length follows a 4 cm crouch, the yaw rate and
the roll angle follow their references, and the legs stay together while driving through a turn.
"""
from stage_common import OuterLoopTests, StageTestCase


class Stage2OuterLoops(OuterLoopTests, StageTestCase):
    STAGE = "lqr"
    GROUP = "단계 2: 순수 LQR + 바깥 루프(높이·요·롤·벌어짐)"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
