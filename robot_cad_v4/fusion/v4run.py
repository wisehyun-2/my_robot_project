"""Module loader for scripts run through the Fusion MCP: reloads the v4 modules in dependency order and drops
same-named modules of other folders (robot_cad and robot_cad_v2 use the same names)."""
import importlib
import os
import sys

PATH = os.path.dirname(os.path.abspath(__file__))
MODS = ['params', 'cadlib', 'layout', 'vendor', 'legs', 'torso', 'shell', 'harness', 'assembly', 'fasteners', 'view',
        'check', 'poses', 'gaps', 'finalize']


def load(names=None):
    if PATH not in sys.path:
        sys.path.insert(0, PATH)
    out = {}
    for n in MODS:
        m = sys.modules.get(n)
        if m is not None and PATH.lower() not in (getattr(m, '__file__', '') or '').lower():
            del sys.modules[n]
    for n in MODS:
        if not os.path.exists(os.path.join(PATH, n + '.py')):
            continue
        out[n] = importlib.reload(importlib.import_module(n))
    return out


def build(with_harness=False, rear_round=False, centre_imu=False):
    """No wiring in the model by default (user 11-02); with_harness=True adds the v3 / v4 harness (harness.py) - the
    links have no cable openings any more, so it would run through closed walls.  rear_round: the shell's rear plan
    corners rounded instead of chamfered (v5, shell.REAR_ROUND).  centre_imu: the electronics layout of v5 (torso.centre_imu)."""
    M = load()
    A, V = M['assembly'], M['vendor']
    M['shell'].REAR_ROUND = rear_round
    if centre_imu:
        M['torso'].centre_imu()
    A.reset()
    parts = V.build_all()
    A.build_torso(parts)
    A.build_shell()
    A.build_leg(1, parts)
    A.build_leg(-1, parts)
    st = A.build_harness(parts) if with_harness else []
    A.place_switch(parts)
    n, bad = M['fasteners'].build_all()            # every screw / insert / nut as geometry (10-30)
    if bad:
        raise RuntimeError('fastener counts differ from fasteners.specs(): %r' % bad)
    return M, st
