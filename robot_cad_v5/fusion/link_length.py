"""Link length of the v5 robot (user 11-07: thigh = shank = 250 mm, was 175; the choice: 25 cm of the 25-30 cm study).

The v4 params stay as they are (v4 keeps 175 mm).  v5 lengthens them right after they are loaded and before the modules that read them
at import.  The nominal pose keeps the angle PHI between link and leg line, so L0 scales with the links: every part near a joint keeps
its orientation and only moves in a straight line, nothing is re-clocked.  L0 is the pose the model is drawn in; a standing height is a
joint pose (`v5run.pose`).
"""
import math

LINK = 250.0
BASE = 175.0                # the link the joint ends were designed on
SCALE = LINK / BASE


def lengthen(P):
    """params: L1 = L2 = LINK, L0 scaled, the joint positions that follow.  Idempotent."""
    if P.L1 != LINK:
        P.L1, P.L2 = LINK, LINK
        P.L0 = 2.0 * LINK * math.cos(P.PHI)
        P.L0_MIN, P.L0_MAX = P.L0_MIN * SCALE, P.L0_MAX * SCALE
        P.Z_HIP = P.L0 + P.R_WHEEL
        P.KNEE_X = LINK * math.sin(P.PHI)
        P.KNEE_Z = P.Z_HIP - LINK * math.cos(P.PHI)
    return P
