"""v5-only values over the v4 params (../v4/fusion/params.py keeps the v4 ones).  No Fusion import: applied right
after the params import by v5run.load() and by the scripts that read the geometry outside Fusion (part drawings,
sections, strength), next to link_length.lengthen().

11-08 (user): the wheel hub flange dowel holes, then the axial stack-up review ("문제 2건 + 쉬운 것")."""

HUB_DOWEL_HOLE = 8.0        # wheel hub flange dowel holes (v4 7.0; 7.5, then 8.0 for the print tolerance): Ø3 x 10 pins
                            # 2.0 proud into the AKE output's 3 mm holes
V5_PARAMS = dict(
    HUB_DOWEL_HOLE=HUB_DOWEL_HOLE,
    HUB_SPIGOT=(5.9, 1.0),  # wheel hub flange spigot 1.3 -> 1.0 into the Ø12 x 1.5 recess (0.5 off the floor; the dowels locate)
    RIM_REG_GAP=(0.1, 0.6),  # rim register recess Ø11.2 x 3.6 (was 3.2): the rim prints hub side down, its recess floor
                            # bridges and sags, and the Ø11 x 3 spigot must not seat before the faces meet
    SHELL_PAD=1.6, SHELL_SCREW_L=8,   # shell M3 screws: Ø8 pads from the wall down to the hip plate web, M3 x 8 (5.0 engaged)
    BOARD_SCREW_L=5,        # moteus board DIN 7984 M2.5 x 5: the tip stays in the 4 mm standoff insert
    TOP_CBORE=4.3,          # top plate M4 x 25 head counterbores (head k 4.0)
)


def apply(P):
    for k, v in V5_PARAMS.items():
        assert hasattr(P, k), k
        setattr(P, k, v)
    return P
