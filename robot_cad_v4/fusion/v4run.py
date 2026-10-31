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


def build(with_harness=True):
    M = load()
    A, V = M['assembly'], M['vendor']
    A.reset()
    parts = V.build_all()
    A.build_torso(parts)
    A.build_shell()
    A.build_leg(1, parts)
    A.build_leg(-1, parts)
    st = A.build_harness(parts) if with_harness else []
    n, bad = M['fasteners'].build_all()            # every screw / insert / nut as geometry (10-30)
    if bad:
        raise RuntimeError('fastener counts differ from fasteners.specs(): %r' % bad)
    return M, st
