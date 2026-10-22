"""Verify the serial-leg three-body plant against the MuJoCo upkie model (docs/plan4_model.md, sec. 5).

    python -m wbr.models.verify_plan4_plant [--L0 0.31 0.25 0.33] [--out results/plan4_model]

For every leg length L0 (knee locked = virtual leg rigid, the assumption of the reduced plant):
  1. equilibrium: posture IK(L0, theta_eq), body level; vertical settling; static joint torques from
     MuJoCo inverse dynamics (base generalized forces must vanish)
  2. B check : initial accelerations (x, theta, phi) for a wheel torque step and a hip torque step,
               mj_forward vs. M^-1 E u of the derived plant
  3. A check : continuous eigenvalues of mjd_transitionFD (log(A_d)/dt) whose eigenvectors lie in the
               symmetric planar subspace, vs. eig(A) of the plant
  4. response: 3 deg rigid tilt, open loop (static torques held), 0.3 s, MuJoCo vs. plant (RK4)
Outputs results/plan4_model/linearization.json and openloop_3deg.png.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import scipy.linalg as la

from wbr import MODELS_DIR, RESULTS_DIR
from wbr.controllers.lqi6.plant_serial import SerialPlant, SerialPlantParams
from wbr.controllers.lqi6.serial_leg import UPKIE_JOINT_SIGNS, SerialLeg, SerialLegGeometry
from wbr.models.upkie_params import set_leg_posture

KNEE_LOCK_ARMATURE = 1.0e3     # kg m^2 on the knee dofs: the knee cannot move (rigid virtual leg)


class MjPlanar:
    """Helpers mapping the planar plant coordinates to the MuJoCo upkie model."""

    def __init__(self, params: SerialPlantParams, xml_path: Path | None = None):
        self.p = params
        self.m = mujoco.MjModel.from_xml_path(str(xml_path or MODELS_DIR / "upkie_plan4" / "scene.xml"))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.leg = SerialLeg(SerialLegGeometry(L1=params.extra["L1"], L2=params.extra["L2"], knee_sign=params.extra["knee_sign"]))
        self.jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in
                    ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel", "base_freejoint")}
        self.dof = {n: int(m.jnt_dofadr[j]) for n, j in self.jid.items()}
        self.qadr = {n: int(m.jnt_qposadr[j]) for n, j in self.jid.items()}
        self.aid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.jid if n != "base_freejoint"}
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.armature0 = m.dof_armature.copy()
        # The linear plant has no Coulomb friction: the joint frictionloss of the model (hip/knee 0.1 Nm,
        # wheel 0.02 Nm) would hold the joints against the tiny finite-difference perturbations and make
        # the hips look locked.  It is switched off for the comparison (documented simplification).
        self.frictionloss0 = m.dof_frictionloss.copy()
        m.dof_frictionloss[:] = 0.0
        # the noslip post-pass of the forward solver is not part of mj_inverse: switch it off so that the
        # static torques from inverse dynamics hold the posture exactly in forward simulation
        m.opt.noslip_iterations = 0

    # ---------------------------------------------------------------- posture / locking
    def lock(self, joints, armature=KNEE_LOCK_ARMATURE):
        self.m.dof_armature[:] = self.armature0
        for n in joints:
            self.m.dof_armature[self.dof[n]] = armature

    def set_equilibrium(self, L0: float, plant: SerialPlant, tilt: float = 0.0, dz: float = 0.0):
        """Equilibrium posture, optionally rigidly tilted about the axle by `tilt` (theta += tilt,
        pitch += tilt) and lowered by dz (settled contact penetration).  Resets the keyframe, i.e. ctrl."""
        th_eq = plant.theta_eq(L0)
        set_leg_posture(self.m, self.d, self.leg, L0, th_eq, pitch=tilt, wheel_radius=self.p.R)
        self.d.qpos[2] += dz
        mujoco.mj_forward(self.m, self.d)
        return th_eq

    def settle(self, seconds: float = 0.1) -> float:
        """Let the robot sink into the soft contact with every joint locked (only valid AT the equilibrium:
        a tilted rigid robot would tip over), then zero the velocities.  Returns the height change."""
        z0 = float(self.d.qpos[2])
        self.lock(("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"))
        self.d.ctrl[:] = 0.0
        for _ in range(int(round(seconds / self.m.opt.timestep))):
            mujoco.mj_step(self.m, self.d)
        self.d.qvel[:] = 0.0
        self.d.qacc[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        return float(self.d.qpos[2]) - z0

    def static_torques(self) -> dict:
        """Joint torques holding the current posture (inverse dynamics with qacc = 0)."""
        self.d.qacc[:] = 0.0
        self.d.qvel[:] = 0.0
        mujoco.mj_inverse(self.m, self.d)
        f = self.d.qfrc_inverse.copy()
        return {n: float(f[self.dof[n]]) for n in self.aid} | {"base_residual": float(np.abs(f[:6]).max())}

    def apply_static(self, torques: dict):
        for n, a in self.aid.items():
            self.d.ctrl[a] = torques.get(n, 0.0)

    # ---------------------------------------------------------------- planar coordinates
    def planar_state(self) -> np.ndarray:
        """[theta, theta_dot, x, x_dot, phi, phi_dot] of the current MuJoCo state (plant conventions)."""
        d, m = self.d, self.m
        Rm = d.xmat[self.base].reshape(3, 3)
        pitch = math.atan2(-Rm[2, 0], Rm[0, 0])            # rotation about +y (positive = leaning forward)
        gyro = Rm @ d.qvel[3:6]                            # free joint angular velocity is body-local
        pitch_rate = float(gyro[1])
        s = UPKIE_JOINT_SIGNS
        qh = d.qpos[self.qadr["left_hip"]] * s["left_hip"]
        qk = d.qpos[self.qadr["left_knee"]] * s["left_knee"]
        qdh = d.qvel[self.dof["left_hip"]] * s["left_hip"]
        qdk = d.qvel[self.dof["left_knee"]] * s["left_knee"]
        _, L0, th_b = self.leg.forward(qh, qk)
        _, th_b_dot = self.leg.virtual_rates(qh, qk, qdh, qdk)
        jl, jr = self.jid["left_wheel"], self.jid["right_wheel"]
        axle = 0.5 * (d.xanchor[jl] + d.xanchor[jr])
        # axle velocity: hip velocity minus leg rotation (rigid leg) -- use the wheel body velocity instead
        v = np.zeros(6)
        bid = m.jnt_bodyid[jl]
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, bid, v, 0)
        v2 = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[jr], v2, 0)
        xd = 0.5 * (v[3] + v2[3])
        return np.array([th_b + pitch, th_b_dot + pitch_rate, axle[0], xd, -pitch, -pitch_rate])

    def P_matrix(self, L0: float, th_eq: float) -> np.ndarray:
        """Planar (dx, dtheta, dphi) -> MuJoCo tangent coordinates (nv)."""
        m, R, h = self.m, self.p.R, self.p.hip_above_base
        P = np.zeros((m.nv, 3))
        dof = self.dof
        # dx: base x, wheels roll
        P[0, 0] = 1.0
        P[dof["left_wheel"], 0] = 1.0 / R
        P[dof["right_wheel"], 0] = -1.0 / R
        # dtheta: hip moves on the circle, leg (and shank -> wheel joints) rotates, hip joints q_hip = theta + phi
        P[0, 1] = L0 * math.cos(th_eq)
        P[2, 1] = -L0 * math.sin(th_eq)
        P[dof["left_hip"], 1] = -1.0
        P[dof["right_hip"], 1] = +1.0
        P[dof["left_wheel"], 1] = -1.0
        P[dof["right_wheel"], 1] = +1.0
        # dphi: body rotates about -y about the hip (base origin h above the hip)
        P[0, 2] = -h
        P[4, 2] = -1.0
        P[dof["left_hip"], 2] = -1.0
        P[dof["right_hip"], 2] = +1.0
        return P

    def ctrl_from_inputs(self, T: float, Tp: float) -> np.ndarray:
        """Plant inputs -> actuator torques (knee locked: a hip torque is a pure hip joint torque)."""
        u = np.zeros(self.m.nu)
        s = UPKIE_JOINT_SIGNS
        for side in ("left", "right"):
            u[self.aid[f"{side}_wheel"]] = 0.5 * T * s[f"{side}_wheel"]
            u[self.aid[f"{side}_hip"]] = 0.5 * Tp * s[f"{side}_hip"]
        return u


def check_L0(mp: MjPlanar, plant: SerialPlant, L0: float, response_T: float = 0.3, tilt_deg: float = 3.0) -> dict:
    m, d = mp.m, mp.d
    dt = m.opt.timestep
    out = {"L0": L0}
    th_eq = mp.set_equilibrium(L0, plant)
    out["theta_eq_deg"] = math.degrees(th_eq)
    dz = mp.settle()
    mp.lock(("left_knee", "right_knee"))
    stat = mp.static_torques()
    mp.apply_static(stat)
    mujoco.mj_forward(m, d)
    out["static_check_qacc"] = dict(base=np.abs(d.qacc[:6]).max(), hips=float(abs(d.qacc[mp.dof["left_hip"]])))
    out["static_torques"] = stat
    out["Tp_eq_plant"] = plant.Tp_eq(L0)
    out["Tp_eq_mujoco"] = float(stat["left_hip"] * UPKIE_JOINT_SIGNS["left_hip"] + stat["right_hip"] * UPKIE_JOINT_SIGNS["right_hip"])
    mp.apply_static(stat)
    q0, v0 = d.qpos.copy(), d.qvel.copy()
    ctrl0 = d.ctrl.copy()

    # ---- B check: initial accelerations ------------------------------------------------
    P = mp.P_matrix(L0, th_eq)
    Qv = np.linalg.pinv(P)
    mujoco.mj_forward(m, d)
    acc0 = d.qacc.copy()
    A_pl, B_pl = plant.linearize(L0)
    bcheck = {}
    for name, (T, Tp) in (("wheel_T", (0.2, 0.0)), ("hip_Tp", (0.0, 1.0))):
        d.qpos[:] = q0; d.qvel[:] = v0
        d.ctrl[:] = ctrl0 + mp.ctrl_from_inputs(T, Tp)
        mujoco.mj_forward(m, d)
        dacc = d.qacc - acc0
        a_mj = Qv @ dacc                                   # (xdd, thdd, phdd)
        u = np.array([T, Tp])
        a_pl = B_pl @ u                                    # rows [thd, thdd, xd, xdd, phd, phdd]
        a_pl = np.array([a_pl[3], a_pl[1], a_pl[5]])
        bcheck[name] = dict(mujoco=a_mj.tolist(), plant=a_pl.tolist(),
                            rel_err=float(np.linalg.norm(a_mj - a_pl) / max(1e-9, np.linalg.norm(a_pl))))
    out["B_check"] = bcheck
    d.ctrl[:] = ctrl0

    # ---- A check: eigenvalues of the FD linearisation projected on the planar subspace ----
    d.qpos[:] = q0; d.qvel[:] = v0
    mujoco.mj_forward(m, d)
    n = 2 * m.nv + m.na
    A_d = np.zeros((n, n)); B_d = np.zeros((n, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, 1, A_d, B_d, None, None)
    A_c = np.real(la.logm(A_d)) / dt
    Pst = np.zeros((n, 6)); Pst[:m.nv, :3] = P; Pst[m.nv:2 * m.nv, 3:] = P
    proj = Pst @ np.linalg.pinv(Pst)
    w, V = np.linalg.eig(A_c)
    frac = np.array([np.linalg.norm(proj @ V[:, i]) / np.linalg.norm(V[:, i]) for i in range(n)])
    planar = [(complex(w[i]), float(frac[i])) for i in range(n) if frac[i] > 0.5]
    planar.sort(key=lambda z: z[0].real)
    eig_plant = np.sort_complex(np.linalg.eigvals(A_pl))
    out["eig_plant"] = [complex(z).real for z in eig_plant]
    out["eig_mujoco_planar"] = [dict(re=z.real, im=z.imag, planar_fraction=f) for z, f in planar]
    # match the unstable poles
    unst_pl = sorted([z.real for z in eig_plant if z.real > 0.5])
    unst_mj = sorted([z.real for z, f in planar if z.real > 0.5])
    out["unstable_poles"] = dict(plant=unst_pl, mujoco=unst_mj,
                                 rel_err=[abs(a - b) / abs(a) for a, b in zip(unst_pl, unst_mj)] if len(unst_pl) == len(unst_mj) else None)
    # also the projected A (least squares) for the doc
    A_proj = np.linalg.pinv(Pst) @ A_c @ Pst
    idx = [1, 4, 0, 3, 2, 5]
    out["A_proj_mujoco"] = A_proj[np.ix_(idx, idx)].tolist()
    out["A_plant"] = A_pl.tolist()

    # ---- open-loop response to a rigid 3 deg tilt ----------------------------------------
    tilt = math.radians(tilt_deg)
    mp.set_equilibrium(L0, plant, tilt=tilt, dz=dz)      # no second settling: the tilted robot would tip
    mp.lock(("left_knee", "right_knee"))
    mp.apply_static(stat)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    X = np.array([th_eq + tilt, 0.0, 0.0, 0.0, -tilt, 0.0])
    x0_mj = mp.planar_state()
    X[2] = x0_mj[2]
    T_eq, Tp_eq = 0.0, plant.Tp_eq(L0)
    traj_mj, traj_pl, ts = [], [], []
    nsteps = int(round(response_T / dt))
    for k in range(nsteps + 1):
        ts.append(k * dt)
        traj_mj.append(mp.planar_state())
        traj_pl.append(X.copy())
        mujoco.mj_step(m, d)
        # RK4 on the plant
        f = lambda Y: plant.deriv(Y, T_eq, Tp_eq, L0)
        k1 = f(X); k2 = f(X + 0.5 * dt * k1); k3 = f(X + 0.5 * dt * k2); k4 = f(X + dt * k3)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    traj_mj, traj_pl = np.array(traj_mj), np.array(traj_pl)
    err = traj_mj - traj_pl
    out["response"] = dict(t=ts, mujoco=traj_mj.tolist(), plant=traj_pl.tolist(),
                           rms_err_theta_deg=float(math.degrees(np.sqrt(np.mean(err[:, 0] ** 2)))),
                           rms_err_phi_deg=float(math.degrees(np.sqrt(np.mean(err[:, 4] ** 2)))),
                           rms_err_x_mm=float(1e3 * np.sqrt(np.mean(err[:, 2] ** 2))),
                           final_theta_deg=dict(mujoco=math.degrees(traj_mj[-1, 0]), plant=math.degrees(traj_pl[-1, 0])),
                           final_phi_deg=dict(mujoco=math.degrees(traj_mj[-1, 4]), plant=math.degrees(traj_pl[-1, 4])))
    mp.lock(())
    return out


def plot_responses(results, out_png: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
    for r in results:
        resp = r["response"]
        t = np.array(resp["t"]); mj = np.array(resp["mujoco"]); pl = np.array(resp["plant"])
        lab = f"L0={r['L0']:.2f}"
        axes[0].plot(t, np.degrees(mj[:, 0]), label=f"MuJoCo {lab}"); axes[0].plot(t, np.degrees(pl[:, 0]), "--", label=f"plant {lab}")
        axes[1].plot(t, np.degrees(mj[:, 4])); axes[1].plot(t, np.degrees(pl[:, 4]), "--")
        axes[2].plot(t, 1e3 * (mj[:, 2] - mj[0, 2])); axes[2].plot(t, 1e3 * (pl[:, 2] - pl[0, 2]), "--")
    axes[0].set_title("virtual leg angle theta [deg]"); axes[1].set_title("body phi = -pitch [deg]"); axes[2].set_title("wheel x [mm]")
    for a in axes:
        a.set_xlabel("t [s]"); a.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.suptitle("Open-loop response to a 3 deg rigid tilt (knee locked, static torques held): MuJoCo vs derived plant")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--L0", type=float, nargs="*", default=[0.31, 0.25, 0.28, 0.33])
    ap.add_argument("--out", default=str(RESULTS_DIR / "plan4_model"))
    args = ap.parse_args()
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    params = SerialPlantParams.from_json(MODELS_DIR / "upkie_plan4" / "plan4_params.json")
    plant = SerialPlant(params)
    mp = MjPlanar(params)
    results = []
    for L0 in args.L0:
        r = check_L0(mp, plant, L0)
        results.append(r)
        up = r["unstable_poles"]
        print(f"L0={L0:.2f}: theta_eq={r['theta_eq_deg']:+.2f} deg | static hip torque plant {r['Tp_eq_plant']:+.3f} vs mujoco "
              f"{r['Tp_eq_mujoco']:+.3f} Nm, base residual {r['static_torques']['base_residual']:.2e}")
        print("   static torques:", {k: round(v, 4) for k, v in r["static_torques"].items()},
              "| forward qacc with static ctrl: base %.3f hips %.3f" % (r["static_check_qacc"]["base"], r["static_check_qacc"]["hips"]))
        print(f"   unstable poles plant {np.round(up['plant'], 3)} mujoco {np.round(up['mujoco'], 3)} rel err {np.round(up['rel_err'], 4) if up['rel_err'] else None}")
        for k, v in r["B_check"].items():
            print(f"   B {k}: mujoco {np.round(v['mujoco'], 4)} plant {np.round(v['plant'], 4)} rel err {v['rel_err']:.4f}")
        rp = r["response"]
        print(f"   3 deg open loop 0.3 s: rms err theta {rp['rms_err_theta_deg']:.3f} deg, phi {rp['rms_err_phi_deg']:.3f} deg, x {rp['rms_err_x_mm']:.2f} mm;"
              f" final theta {rp['final_theta_deg']['mujoco']:.2f}/{rp['final_theta_deg']['plant']:.2f} deg")
    slim = []
    for r in results:
        rr = dict(r); rr["response"] = {k: v for k, v in r["response"].items() if k not in ("t", "mujoco", "plant")}
        slim.append(rr)
    (out_dir / "linearization.json").write_text(json.dumps(slim, indent=1), encoding="utf-8")
    plot_responses(results, out_dir / "openloop_3deg.png")
    print("wrote", out_dir)


if __name__ == "__main__":
    main()
