"""Build the scratch document of a job (python fusion_rpc.py f_ax_build.py 600 <job>)."""
import sys
HERE = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v2\manufacturing\explain\assembly\scripts'


def run(_context: str):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    sys.modules.pop('axlib', None)
    import axlib as X
    parts, nf, missing = X.build(JOB)  # noqa: F821  (JOB is prepended by fusion_rpc.py)
    print(JOB, 'parts', parts, 'fasteners', nf, 'missing', missing)  # noqa: F821
