"""Stage 4 - MPC + position integral + static-tilt integral states (z2, q_att): the full MPC controller.

    python -m unittest discover -s v5_mpc/TEST -p "test_stage4*.py" -v

Pass = under the same three constant disturbances the TRUE body pitch ends within 0.5 deg of level, and every test of
stages 1 to 3 still passes (same tests and limits as v5_lqi stage 4).
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase, TiltIntegralTests


class Stage4IntTilt(TiltIntegralTests, PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "mpc_ipos_itilt"
    GROUP = "단계 4: MPC + ∫위치 + ∫기울기"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
