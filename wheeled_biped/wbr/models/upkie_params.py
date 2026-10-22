"""Plan 4 plant parameters of the upkie MuJoCo model: lumped leg table over the virtual leg length,
body / wheel parameters, standing posture (docs/plan4_model.md, section 2).

    python -m wbr.models.upkie_params [--model upkie_plan4]   # writes models/<model>/plan4_params.json

Bodies are grouped by the last joint on their path from the base: base group (torso, battery,
handle, hip stators) = body; hip group (hip rotor, horn, femur, knee stator) = thigh; knee group
(knee rotor, horn, tibia, ankle, mj5208 stator) = shank; wheel group (mj5208 rotor, hub, tire).
For each L0 the legs are set to the IK posture with the virtual leg vertical (theta_v = 0), so the
leg frame coincides with the world frame: c = COM height above the axle, d = COM forward offset.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict

import mujoco
import numpy as np

from wbr import MODELS_DIR
from wbr.controllers.lqi6.plant_serial import LegTable, SerialPlantParams
from wbr.controllers.lqi6.serial_leg import UPKIE_JOINT_SIGNS, SerialLeg, SerialLegGeometry

L_STAND = 0.31          # standing virtual leg length (knee ~51 deg; straight leg = 0.343 is singular)
L_MIN, L_MAX = 0.24, 0.335
TABLE_GRID = np.round(np.arange(0.22, 0.3401, 0.005), 4)


def _group_bodies(m: mujoco.MjModel) -> Dict[str, list]:
    def last_joint(b):
        while b != 0:
            if m.body_jntnum[b] > 0:
                j = m.body_jntadr[b] + m.body_jntnum[b] - 1
                if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    return "base"
                return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            b = m.body_parentid[b]
        return "world"
    groups: Dict[str, list] = {}
    for b in range(1, m.nbody):
        groups.setdefault(last_joint(b), []).append(b)
    return groups


def _planar_inertia(m: mujoco.MjModel, d: mujoco.MjData, bodies, about=None):
    """Mass, COM (x, z) and inertia about +y of a set of bodies (about their common COM or a point)."""
    mass = float(sum(m.body_mass[b] for b in bodies))
    com = sum(m.body_mass[b] * d.xipos[b] for b in bodies) / mass
    ref = com if about is None else np.asarray(about)
    I = 0.0
    for b in bodies:
        Rm = d.ximat[b].reshape(3, 3)
        Iw = Rm @ np.diag(m.body_inertia[b]) @ Rm.T
        r = d.xipos[b] - ref
        I += Iw[1, 1] + m.body_mass[b] * (r[0] ** 2 + r[2] ** 2)
    return mass, com, float(I)


def set_leg_posture(m, d, leg: SerialLeg, L0: float, theta_v: float, pitch: float = 0.0, wheel_radius: float = 0.05):
    """Set both legs to IK(L0, theta_v) (u-convention -> upkie joint signs) and place the base so that
    the axle is at x = 0, z = wheel_radius and the base is pitched by `pitch` (about +y)."""
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(m, d, key)
    qh, qk = leg.inverse(L0, theta_v)
    for side in ("left", "right"):
        for jn, q in ((f"{side}_hip", qh), (f"{side}_knee", qk)):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            d.qpos[m.jnt_qposadr[j]] = q * UPKIE_JOINT_SIGNS[jn]
    mujoco.mj_forward(m, d)
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    jl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel")
    jr = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel")
    axle = 0.5 * (d.xanchor[jl] + d.xanchor[jr])
    off = axle - d.xpos[base]
    off[1] = 0.0
    c, s = math.cos(pitch), math.sin(pitch)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    off_w = Ry @ off
    fq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_freejoint")]
    d.qpos[fq:fq + 3] = np.array([-off_w[0], 0.0, -off_w[2] + wheel_radius])
    d.qpos[fq + 3:fq + 7] = np.array([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    mujoco.mj_forward(m, d)
    return qh, qk


def build_params(xml_path: Path | None = None, grid=TABLE_GRID, verbose: bool = True) -> SerialPlantParams:
    xml_path = xml_path or MODELS_DIR / "upkie_plan4" / "scene.xml"
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    leg = SerialLeg(SerialLegGeometry(knee_sign=-1.0))
    groups = _group_bodies(m)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)

    # geometry check: link lengths from the straight-leg keyframe
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(m, d, key)
    mujoco.mj_forward(m, d)
    hip, knee, axle = d.xanchor[jid("left_hip")], d.xanchor[jid("left_knee")], d.xanchor[jid("left_wheel")]
    L1 = float(np.linalg.norm((hip - knee)[[0, 2]]))
    L2 = float(np.linalg.norm((knee - axle)[[0, 2]]))
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    hip_above_base = float(d.xpos[base][2] - hip[2])
    wheel_radius = float(m.geom_size[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_tire")][0])
    leg = SerialLeg(SerialLegGeometry(L1=L1, L2=L2, knee_sign=-1.0))

    # body group (base level): COM from the hip, pitch inertia
    m_b, com_b, I_b = _planar_inertia(m, d, groups["base"])
    b_x, b_z = float(com_b[0] - hip[0]), float(com_b[2] - hip[2])
    # wheels (both): mass and inertia about the axle (+ rotor armature)
    wheel_bodies = groups["left_wheel"] + groups["right_wheel"]
    m_w, _, _ = _planar_inertia(m, d, wheel_bodies)
    I_w, J_w = 0.0, 0.0
    for side in ("left", "right"):
        _, _, I_side = _planar_inertia(m, d, groups[f"{side}_wheel"], about=d.xanchor[jid(f"{side}_wheel")])
        I_w += I_side                                                        # rigid wheel bodies about the axle
        J_w += float(m.dof_armature[m.jnt_dofadr[jid(f"{side}_wheel")]])    # rotor armature: on the joint rate (m9)
    J_h = float(sum(m.dof_armature[m.jnt_dofadr[jid(f"{s}_hip")]] for s in ("left", "right")))
    c_h = float(sum(m.dof_damping[m.jnt_dofadr[jid(f"{s}_hip")]] for s in ("left", "right")))
    c_w = float(sum(m.dof_damping[m.jnt_dofadr[jid(f"{s}_wheel")]] for s in ("left", "right")))

    # leg table: legs vertical (theta_v = 0) for each L0
    rows = []
    leg_bodies = groups["left_hip"] + groups["left_knee"] + groups["right_hip"] + groups["right_knee"]
    for L0 in grid:
        if L0 >= leg.g.L0_max - 1e-3:
            continue
        set_leg_posture(m, d, leg, float(L0), 0.0, wheel_radius=wheel_radius)
        axle_w = 0.5 * (d.xanchor[jid("left_wheel")] + d.xanchor[jid("right_wheel")])
        m_l, com_l, I_l = _planar_inertia(m, d, leg_bodies)
        rows.append((float(L0), m_l, float(com_l[2] - axle_w[2]), float(com_l[0] - axle_w[0]), I_l))
    rows = np.array(rows)
    table = LegTable(L0=rows[:, 0], m_l=rows[:, 1], c=rows[:, 2], d=rows[:, 3], I_l=rows[:, 4])
    params = SerialPlantParams(R=wheel_radius, m_w=m_w, I_w=I_w, m_b=m_b, b_x=b_x, b_z=b_z, I_b=I_b, J_h=J_h, c_h=c_h,
                               c_w=c_w, J_w=J_w, hip_above_base=hip_above_base, leg=table, L0=L_STAND, L_min=L_MIN, L_max=L_MAX,
                               tau_wheel_max=1.7, tau_hip_max=16.0, name="upkie_serial",
                               extra=dict(L1=L1, L2=L2, knee_sign=-1.0, total_mass=float(m.body_subtreemass[base]),
                                          hip_joint_z_straight=float(hip[2])))
    if verbose:
        print(f"L1={L1:.4f} L2={L2:.4f}  body m_b={m_b:.4f} b_x={b_x:+.4f} b_z={b_z:.4f} I_b={I_b:.5f}")
        print(f"wheels m_w={m_w:.4f} I_w={I_w:.6f} J_w={J_w:.6f}  J_h={J_h} c_h={c_h} c_w={c_w}  hip above base {hip_above_base:.4f}")
        print("  L0     m_l      c        d        I_l")
        for r in rows:
            print("  %.3f  %.4f  %.4f  %+.4f  %.5f" % tuple(r))
    return params


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="upkie_plan4", help="model directory under models/")
    args = ap.parse_args()
    params = build_params(MODELS_DIR / args.model / "scene.xml")
    out = MODELS_DIR / args.model / "plan4_params.json"
    params.to_json(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
