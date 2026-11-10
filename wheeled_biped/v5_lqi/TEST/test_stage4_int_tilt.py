"""Stage 4 - LQR + position integral + static-tilt integral (z2, q_att): the full balance controller.

    python -m unittest discover -s v5_lqi/TEST -p "test_stage4*.py" -v        (about 1 minute)

Pass = (a) under the same three constant disturbances the TRUE body pitch ends within 0.5 deg of level; (b) the
position tests of stage 3 and every test of stages 1 and 2 still pass.
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase, TiltIntegralTests


class Stage4IntTilt(TiltIntegralTests, PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "lqr_ipos_itilt"
    GROUP = "단계 4: LQR + ∫위치 + ∫기울기"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
