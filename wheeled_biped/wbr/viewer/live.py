"""Live MuJoCo viewer running one of the plans in real time.

    python -m wbr.viewer.live --plan 1 [--balancer pid|mpc] [--speed 1.0] [--noise] [--latency-ms 0]

Keys (viewer window must have focus):
  Up/Down     forward velocity command +-0.1 m/s        Left/Right  yaw rate +-0.2 rad/s
  Space       zero the commands                          P           push the torso (20 N for 0.1 s)
  O           push backward                              R           reset the robot
  H / J       height command +-0.02 m (plans 2/3)        T           10 deg tilt and release
"""
from __future__ import annotations

import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np


def make_plan(args):
    if args.plan == 1:
        from wbr.plans.plan1_upkie import Plan1Config, Plan1Upkie
        return Plan1Upkie(Plan1Config(balancer=args.balancer, noise=args.noise, latency_ms=args.latency_ms))
    if args.plan == 2:
        from wbr.plans.plan2_diablo_lqi6 import Plan2Config, Plan2DiabloLQI6
        return Plan2DiabloLQI6(Plan2Config(noise=args.noise, latency_ms=args.latency_ms))
    if args.plan == 3:
        from wbr.plans.plan3_diablo_upkie import Plan3Config, Plan3DiabloUpkie
        return Plan3DiabloUpkie(Plan3Config(noise=args.noise, latency_ms=args.latency_ms))
    raise ValueError(args.plan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", type=int, default=1, choices=(1, 2, 3))
    ap.add_argument("--balancer", default="pid", choices=("pid", "mpc"))
    ap.add_argument("--speed", type=float, default=1.0, help="real-time factor")
    ap.add_argument("--noise", action="store_true")
    ap.add_argument("--latency-ms", type=float, default=0.0)
    args = ap.parse_args()
    plan = make_plan(args)
    cmd = {"v": 0.0, "w": 0.0, "h": 0.0, "push": 0.0, "reset": False, "tilt": False}

    def key_cb(keycode):
        if keycode == 265:
            cmd["v"] = min(cmd["v"] + 0.1, 1.5)
        elif keycode == 264:
            cmd["v"] = max(cmd["v"] - 0.1, -1.5)
        elif keycode == 263:
            cmd["w"] = min(cmd["w"] + 0.2, 1.5)
        elif keycode == 262:
            cmd["w"] = max(cmd["w"] - 0.2, -1.5)
        elif keycode == 32:
            cmd["v"] = 0.0; cmd["w"] = 0.0
        elif keycode == ord("P"):
            cmd["push"] = 20.0
        elif keycode == ord("O"):
            cmd["push"] = -20.0
        elif keycode == ord("R"):
            cmd["reset"] = True
        elif keycode == ord("T"):
            cmd["tilt"] = True
        elif keycode == ord("H"):
            cmd["h"] = min(cmd["h"] + 0.02, 0.12)
        elif keycode == ord("J"):
            cmd["h"] = max(cmd["h"] - 0.02, -0.12)
        print(f"cmd: v={cmd['v']:+.2f} m/s  yaw={cmd['w']:+.2f} rad/s  dh={cmd['h']:+.2f} m")

    sim = plan.sim
    print(__doc__)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key_cb, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = sim.base_bid
        viewer.cam.distance = 1.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -15
        wall0 = time.perf_counter()
        sim_t0 = sim.time
        last_print = 0.0
        n_sync = max(1, int(round(0.01 / plan.dt)))
        k = 0
        while viewer.is_running():
            if cmd["reset"]:
                plan.reset(); cmd["reset"] = False; cmd["v"] = cmd["w"] = 0.0
                wall0 = time.perf_counter(); sim_t0 = sim.time
            if cmd["tilt"]:
                plan.reset(pitch=math.radians(10)); cmd["tilt"] = False
                wall0 = time.perf_counter(); sim_t0 = sim.time
            if cmd["push"] != 0.0:
                plan.push(cmd["push"], 0.1); cmd["push"] = 0.0
            if hasattr(plan, "set_height_offset"):
                plan.set_height_offset(cmd["h"])
            obs = plan.step(cmd["v"], cmd["w"])
            if plan.fallen:
                print("fell -> reset")
                plan.reset(); cmd["v"] = cmd["w"] = 0.0
                wall0 = time.perf_counter(); sim_t0 = sim.time
            k += 1
            if k % n_sync == 0:
                pitch = obs["base_orientation"]["pitch"] if "base_orientation" in obs else obs.get("pitch", 0.0)
                vx = obs["sim"]["base"]["linear_velocity"][0]
                with viewer.lock():
                    scn = viewer.user_scn
                    scn.ngeom = 1
                    g = scn.geoms[0]
                    pos = sim.data.xpos[sim.base_bid] + np.array([0, 0, 0.35])
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.005, 0, 0]), pos, np.eye(3).flatten(),
                                        np.array([1, 1, 1, 0.0], dtype=np.float32))
                    g.label = f"plan{args.plan} pitch {math.degrees(pitch):+5.1f} deg  v {vx:+.2f} m/s  cmd v {cmd['v']:+.1f} w {cmd['w']:+.1f}"
                viewer.sync()
            # real-time pacing
            target_wall = wall0 + (sim.time - sim_t0) / args.speed
            now = time.perf_counter()
            if target_wall > now:
                time.sleep(target_wall - now)
            if sim.time - last_print > 2.0:
                last_print = sim.time
                pitch = obs["base_orientation"]["pitch"] if "base_orientation" in obs else obs.get("pitch", 0.0)
                print(f"t={sim.time:6.1f}s pitch={math.degrees(pitch):+6.2f} deg x={obs['sim']['base']['position'][0]:+.2f} m "
                      f"rt-factor={(sim.time - sim_t0) / max(1e-6, time.perf_counter() - wall0):.2f}")


if __name__ == "__main__":
    main()
