"""The full MPC controller: MPC + position integral + static-tilt integral states.

    python v5_mpc/TEST/run_stages.py --stage 4 --keep-going        (from wheeled_biped/)

Same tests and limits as v5_lqi/TEST.
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase, TiltIntegralTests


class Stage4IntTilt(TiltIntegralTests, PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "mpc_ipos_itilt"
    GROUP = "단계 4: MPC + ∫위치 + ∫기울기"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
