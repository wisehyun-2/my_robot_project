"""Stage 3 - MPC + position integral state (z3 = integral of the position error in the prediction model, q_pos).

    python -m unittest discover -s v5_mpc/TEST -p "test_stage3*.py" -v

Pass = under a constant disturbance the robot ends within 3 cm of where it started, never strays more than 10 cm, and
every test of stages 1 and 2 still passes (same tests and limits as v5_lqi stage 3).
"""
from stage_common import BalanceTests, OuterLoopTests, PositionIntegralTests, StageTestCase


class Stage3IntPosition(PositionIntegralTests, BalanceTests, OuterLoopTests, StageTestCase):
    STAGE = "mpc_ipos"
    GROUP = "단계 3: MPC + ∫위치"


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
