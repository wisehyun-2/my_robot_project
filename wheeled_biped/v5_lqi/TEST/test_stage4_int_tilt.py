"""The full balance controller: LQR + position integral (z3, q_pos) + static-tilt integral (z2, q_att).

    python v5_lqi/TEST/run_stages.py --stage 4 --keep-going        (from wheeled_biped/)

The tests: balance (1-x), outer loops (2-x), position under constant disturbances (3-x) and the TRUE body pitch under the same
disturbances (4-x).
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase, TiltIntegralTests


class Stage4IntTilt(TiltIntegralTests, PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "lqr_ipos_itilt"
    GROUP = "단계 4: LQR + ∫위치 + ∫기울기"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
