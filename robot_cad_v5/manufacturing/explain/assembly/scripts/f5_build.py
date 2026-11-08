"""Build the scratch document of a job (python fusion_rpc.py f5_build.py 900 <job>).
Imports export/WheelBiped_v5.f3d into an unsaved document once (kept open for the next job, tagged ax5/src), copies
the job's parts into a new unsaved direct-design document and builds the fasteners the model lacks."""
import sys
HERE = r'C:\Users\User\Desktop\myrobot\my_robot_project\robot_cad_v5\manufacturing\explain\assembly\scripts'


def run(_context: str):
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    sys.modules.pop('axlib5', None)
    import axlib5 as X
    counts, missing = X.build(JOB)  # noqa: F821  (JOB is prepended by fusion_rpc.py)
    print(JOB, 'groups', counts, 'missing', missing)  # noqa: F821
