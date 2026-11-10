"""Stage 3 - LQR + position integral (z3 = integral of the position error, q_pos).

    python -m unittest discover -s v5_lqi/TEST -p "test_stage3*.py" -v        (about 1 minute)

Pass = (a) under a constant disturbance the controller does not know (5 deg slope, +1 deg IMU pitch bias, a payload
ahead of the axle) the robot ends within 3 cm of where it started and never strays more than 10 cm; (b) every test of
stages 1 and 2 still passes with the integrator on.  The body tilt is not judged yet: that is stage 4.
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase


class Stage3IntPosition(PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "lqr_ipos"
    GROUP = "단계 3: LQR + ∫위치"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
