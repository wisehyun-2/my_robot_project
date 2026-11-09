"""v5 rocker switch on the top of the torso (11-09, user: "스위치는 앞이 아니라 윗면 ... 정중앙"), instead of the switch-side face.

The exact top centre is the payload plate, with the battery 16.6 mm under it (the switch body is 24 mm deep), so the user chose the
top of the hip shell section just behind the payload plate, on the robot's centre line: x +78 (knee side, midway between the payload
plate opening at x 61 and the hip / knee-side shell seam at x 95), y 0.  Nothing is under it down to the shell floor (the rails end at
x 59.5, the battery at x 38.8; the top splice tabs sit at y +-60).  The 19 x 13 cut-out lies on the seam of shell_mid_L / _R, half in
each piece; the long side runs across the robot (y), as on the face before.  The front face keeps no switch hole.

The v4 files stay as they are: v5run.load() calls patch(M), which swaps shell.openings (the v4 face hole falls outside the shell, the
top hole is cut) and assembly.place_switch.  v5run.reswitch() applies it to the hub file.  No Fusion import at module level.
"""
X, Y = 78.0, 0.0             # robot mm, hole centre on the top surface (z = Z_HIP + TORSO_TOP)
HOLE = (19.0, 13.0)          # panel cut-out (vendor.rocker_switch): along y, along x
FAR = 1.0e4                  # SWITCH_Z while the v4 openings() runs: its face hole box lands far outside the shell
NAME = 'rocker switch'       # vendor.rocker_switch master name prefix


def frame(M):
    """Switch frame (vendor.rocker_switch: panel front face Z = 0, body towards -Z): on the top surface, X across the robot."""
    C, P = M['cadlib'], M['params']
    return C.frame((X, Y, P.Z_HIP + P.TORSO_TOP), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def patch(M):
    P, SH, A, C = M['params'], M['shell'], M['assembly'], M['cadlib']
    v4_openings = SH.openings

    def openings(h):
        z = P.SWITCH_Z
        P.SWITCH_Z = FAR
        try:
            v4_openings(h)
        finally:
            P.SWITCH_Z = z
        B = C.Builder()
        top = P.Z_HIP + P.TORSO_TOP
        C.cut(h, B.abox(X - HOLE[1] / 2, X + HOLE[1] / 2, Y - HOLE[0] / 2, Y + HOLE[0] / 2, top - 6.0, top + 2.0))
        return h
    SH.openings = openings                              # shell.build_shell -> hollow -> openings (module global, read at call)
    A.place_switch = lambda parts: A.place(A.links()['torso'], parts['switch'], frame(M))
    return M
