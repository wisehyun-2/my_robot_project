"""Stage 2 - the other four controllers of the paper's Fig. 6 on top of the pure LQR: height (leg-length PD), yaw
(PI on the yaw rate), roll (compliant: IMU roll to a differential leg force) and split angle (anti-split PD).

    python -m unittest discover -s v5_lqi/TEST -p "test_stage2*.py" -v        (about 40 s)

Pass = each loop does its job without disturbing the balance: the leg length follows a 4 cm crouch, the yaw rate and
the roll angle follow their references, and the legs stay together while driving through a turn.  The knee's coupling
compensation keeps the hip and wheel torques out of the leg length (2-5, 2-6).  A leg pressed by a bump under one wheel
yields and the body stays level (2-7, 2-8), inside each leg's range (2-9) and under a one-sided load (2-10).  A wheel
stopped by a sharp step does not turn the robot (2-11 to 2-13).  The attitude and the yaw rate stay true while driving and turning,
with the IMU chain of the real board (2-14 to 2-16).
"""
from stage_common import OuterLoopTests, StageTestCase


class Stage2OuterLoops(OuterLoopTests, StageTestCase):
    STAGE = "lqr"
    GROUP = "단계 2: 순수 LQR + 바깥 루프(높이·요·롤·벌어짐)"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
