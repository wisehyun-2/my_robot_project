"""Stage 1 - pure LQR: 6 states [theta, theta_dot, v - v_ref, phi, phi_dot, x - x_ref], no integral action.

    python -m unittest discover -s TEST -p "test_stage1*.py" -v        (from wheeled_biped/, about 1 minute)

Pass = the robot balances with state feedback alone: it stands, recovers from a 10 deg tilt and a push of half its
weight, and follows a velocity step.  The last test records what a constant disturbance leaves behind without
integrators (position offset, body tilt): the numbers stages 3 and 4 have to remove.
"""
import stage_common as SC
from stage_common import BalanceTests, Check, StageTestCase


class Stage1LQR(BalanceTests, StageTestCase):
    STAGE = "lqr"
    GROUP = "단계 1: 순수 LQR (6상태)"

    def test_1_5_residual_errors_recorded(self):
        """Not a performance requirement: the robot must stay up under every disturbance, and the offsets it keeps
        are recorded (sanity bounds only).  Expect several cm and about 1 deg here; that is what the integrators fix."""
        for i, (kind, label) in enumerate(SC.DISTURBANCES, 1):
            m = self.check(f"1-5{'abc'[i - 1]}", f"{label} 정지: 적분 없이 남는 오차(기록)", SC.disturbed_stand,
                           [Check("offset_end_cm", "<=", 15.0, "cm", "기록: 남는 위치 오프셋(∫위치 기준은 3 cm)"),
                            Check("pitch_end_deg", "<=", 2.5, "deg", "기록: 남는 몸통 기울기(∫기울기 기준은 0.5°)")], kind=kind)
            print(f"  → 위치 {m['offset_end_signed_cm']:+.1f} cm, 기울기 {m['pitch_end_signed_deg']:+.2f}°, 최대 이탈 {m['excursion_max_cm']:.1f} cm", end="")


if __name__ == "__main__":
    import unittest
    unittest.main(verbosity=2)
