"""Live MuJoCo window: the v5 robot (full controller, real IMU chain) drives over curved speed bumps under one wheel.

    python -m wbr.viewer.course_v5 [--humps 0.06,0.10,0.14,0.20] [--v 0.8] [--speed 1.0] [--both-wheels] [--once]

The bumps are in the robot's path, alternating left and right wheel, length max(0.8 m, 10 h).  The robot drives by itself, slows down while its
legs differ in length, stops after the last bump and starts again.

Keys (viewer window must have focus):
  Up/Down     cruise speed +-0.1 m/s        Space   stop / go        R   back to the start        P / O   push forward / backward
"""
from __future__ import annotations

import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np


def course(heights, both_wheels: bool = False, half_width: float = 0.15):
    humps, x, side = [], 1.0, 1.0
    for h in heights:
        length = max(0.8, 10.0 * h)
        humps.append((h, 0.0 if both_wheels else side, x, length, 0.5 if both_wheels else half_width))
        x += length + 1.0
        side = -side
    return tuple(humps), x


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--humps", default="0.06,0.10,0.14,0.20", help="bump heights [m]")
    ap.add_argument("--v", type=float, default=0.8, help="cruise speed command [m/s]")
    ap.add_argument("--speed", type=float, default=1.0, help="real-time factor")
    ap.add_argument("--both-wheels", action="store_true")
    ap.add_argument("--once", action="store_true", help="stay at the end of the course")
    args = ap.parse_args()

    from wbr.plans.plan_v5 import PlanV5, PlanV5Config
    humps, x_end = course([float(h) for h in args.humps.split(",")], args.both_wheels)
    plan = PlanV5(PlanV5Config.from_model(floor_humps=humps, noise=True, seed=0, latency_ms=10.0))
    sim = plan.sim
    cmd = {"v": args.v, "go": True, "push": 0.0, "reset": False}

    def key_cb(keycode):
        if keycode == 265:
            cmd["v"] = min(cmd["v"] + 0.1, 1.0)
        elif keycode == 264:
            cmd["v"] = max(cmd["v"] - 0.1, 0.1)
        elif keycode == 32:
            cmd["go"] = not cmd["go"]
        elif keycode == ord("R"):
            cmd["reset"] = True
        elif keycode == ord("P"):
            cmd["push"] = 30.0
        elif keycode == ord("O"):
            cmd["push"] = -30.0
        print(f"cmd: v={cmd['v']:+.1f} m/s  go={cmd['go']}", flush=True)

    print(__doc__)
    print("course:", ", ".join(f"{1000 * h[0]:.0f} mm {'both' if h[1] == 0 else 'left' if h[1] > 0 else 'right'} wheel at x {h[2]:.1f} m, {h[3]:.1f} m long" for h in humps), flush=True)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key_cb, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = sim.base_bid
        viewer.cam.distance = 2.2
        viewer.cam.azimuth = 150
        viewer.cam.elevation = -15
        wall0, sim_t0 = time.perf_counter(), sim.time
        n_sync = max(1, int(round(0.01 / plan.dt)))
        k = 0
        last_print = 0.0
        t_stop = None
        roll_peak = 0.0
        while viewer.is_running():
            x = float(plan.observation["sim"]["base"]["position"][0])
            if cmd["reset"] or (t_stop is not None and not args.once and sim.time - t_stop > 3.0):
                plan.reset()
                cmd["reset"] = False
                t_stop = None
                roll_peak = 0.0
                wall0, sim_t0 = time.perf_counter(), sim.time
                last_print = sim.time
                x = float(plan.observation["sim"]["base"]["position"][0])
            if cmd["push"] != 0.0:
                plan.push(cmd["push"], 0.1)
                cmd["push"] = 0.0
            if x > x_end and t_stop is None:
                t_stop = sim.time
                print(f"end of the course: largest roll {roll_peak:.1f} deg", flush=True)
            start = sim.time - sim_t0 > 1.0
            obs = plan.step(cmd["v"] if cmd["go"] and start and t_stop is None else 0.0, 0.0)
            if plan.fallen:
                print("fell -> start again", flush=True)
                plan.reset()
                t_stop = None
                wall0, sim_t0 = time.perf_counter(), sim.time
            k += 1
            e = plan.mcu.est
            roll_peak = max(roll_peak, abs(math.degrees(e.roll)))
            if k % n_sync == 0:
                vx = obs["sim"]["base"]["linear_velocity"][0]
                with viewer.lock():
                    scn = viewer.user_scn
                    scn.ngeom = 1
                    g = scn.geoms[0]
                    pos = sim.data.xpos[sim.base_bid] + np.array([0, 0, 0.35])
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.005, 0, 0]), pos, np.eye(3).flatten(), np.array([1, 1, 1, 0.0], dtype=np.float32))
                    g.label = (f"roll {math.degrees(e.roll):+5.1f} deg  pitch {math.degrees(e.pitch):+5.1f} deg  v {vx:+.2f} m/s  "
                               f"L {1000 * e.L0['left']:.0f} / R {1000 * e.L0['right']:.0f} mm")
                viewer.sync()
            target_wall = wall0 + (sim.time - sim_t0) / args.speed
            now = time.perf_counter()
            if target_wall > now:
                time.sleep(target_wall - now)
            if sim.time - last_print > 2.0:
                last_print = sim.time
                print(f"t={sim.time:6.1f}s x={x:+.2f} m v={obs['sim']['base']['linear_velocity'][0]:+.2f} m/s roll={math.degrees(e.roll):+5.1f} deg pitch={math.degrees(e.pitch):+5.1f} deg "
                      f"legs {1000 * e.L0['left']:.0f}/{1000 * e.L0['right']:.0f} mm  rt-factor={(sim.time - sim_t0) / max(1e-6, time.perf_counter() - wall0):.2f}", flush=True)


if __name__ == "__main__":
    main()
