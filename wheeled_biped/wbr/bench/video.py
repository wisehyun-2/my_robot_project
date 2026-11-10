"""Render a demonstration sequence of the Plan 4 closed loop (upkie-hardware model) to MP4 (offscreen MuJoCo renderer).

    python -m wbr.bench.video [--plan 4] [--balancer ramp|mpc] [--out results/videos/plan4_lqi8.mp4]

Sequence: stand 1 s -> push 0.6 W at 1.0 s -> drive 0.6 m/s from 2.5 s to 5.5 s with a 0.8 rad/s
turn between 3.5 and 4.5 s -> crouch 6 cm at 6 s -> back up at 7.5 s -> stop at 9 s.
(Plans 1-3 were removed on 2026-11-11; their videos stay in results/videos.)
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from wbr import RESULTS_DIR


def make_plan(balancer: str):
    from wbr.plans.plan4_lqi8 import Plan4Config, Plan4LQI8
    return Plan4LQI8(Plan4Config(noise=True, latency_ms=10.0, host="mpc" if balancer == "mpc" else "ramp"))


def command_at(t: float):
    v = 0.6 if 2.5 <= t < 5.5 else 0.0
    w = 0.8 if 3.5 <= t < 4.5 else 0.0
    dh = -0.06 if 6.0 <= t < 7.5 else 0.0
    return v, w, dh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", type=int, default=4, choices=(4,), help="kept for old command lines; only Plan 4 is left")
    ap.add_argument("--balancer", default="ramp", choices=("ramp", "mpc"), help="host: ramp (stage 1) or the MPC host (stage 2)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--duration", type=float, default=10.0)
    args = ap.parse_args()
    plan = make_plan(args.balancer)
    tag = "lqi8_mpc" if args.balancer == "mpc" else "lqi8"
    out = Path(args.out) if args.out else RESULTS_DIR / "videos" / f"plan4_{tag}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    sim = plan.sim
    renderer = mujoco.Renderer(sim.model, height=720, width=1280)
    cam = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, "track")
    writer = imageio.get_writer(str(out), fps=args.fps, codec="libx264", quality=8, macro_block_size=None)
    W = sim.total_mass() * 9.81
    frame_period = 1.0 / args.fps
    next_frame = 0.0
    pushed = False
    n = int(round(args.duration / plan.dt))
    label_opt = mujoco.MjvOption()
    for k in range(n):
        t = k * plan.dt
        v, w, dh = command_at(t)
        plan.set_height_offset(dh)
        if t >= 1.0 and not pushed:
            plan.push(0.6 * W, 0.1)
            pushed = True
        obs = plan.step(v, w)
        if plan.fallen:
            print(f"fell at t={t:.2f} s")
            break
        if t >= next_frame:
            renderer.update_scene(sim.data, camera=cam, scene_option=label_opt)
            frame = renderer.render()
            writer.append_data(frame)
            next_frame += frame_period
    writer.close()
    renderer.close()
    print("wrote", out)


if __name__ == "__main__":
    main()
