"""Rebuild the whole robot in the active Fusion design (from an empty or an old build).

Run inside Fusion (e.g. through the MCP script runner):
    import build_all; build_all.main()

Order: links and motors (build_setup) -> upkie adaptation and leg parts (build_links)
-> torso (build_torso) -> right leg and joints (build_assembly, switches to parametric).
The battery position in params.py keeps the torso COM over the hip-axis midpoint;
check it with suggest_battery_offset() in a separate script after a build.
"""
import adsk.core
import adsk.fusion

import build_assembly
import build_links
import build_setup
import build_torso
import cadlib as C
import check
import params as P


def suggest_battery_offset():
    """Battery offset that puts the torso COM over the hip-axis midpoint.

    Run it as a separate (read-only) script after a build: physical properties read
    in the same script that created the bodies came out wrong (stale placement)."""
    _txt, ct = check.report()
    items = []
    check.leaf_items(C.root_occ('torso:1'), items, '')
    m_t = sum(i[1] for i in items)
    tray = sum(i[1] for i in items if 'battery_tray' in i[0])
    bx = P.BATTERY_X - ct[0] * m_t / (P.BATTERY_MASS + tray)     # the tray follows the battery in x
    by = P.BATTERY_Y - ct[1] * m_t / P.BATTERY_MASS
    return ct[0], ct[1], bx, by


def main():
    d = C.design()
    d.designType = adsk.fusion.DesignTypes.DirectDesignType       # drops the old timeline and joints
    root = d.rootComponent
    while root.occurrences.count:
        root.occurrences.item(0).deleteMe()
    build_setup.main(new_doc=False)
    build_links.main()
    build_torso.main()
    build_assembly.main()
    print('built; run suggest_battery_offset() in a separate script to check the torso COM')
