"""Prepare a map for LMPC: waypoint csv + recorded initial safe set.

Mirrors src/record_initial_safe_set.cpp: drive the centerline with a simple
path follower (pure pursuit + P speed control) for two laps in f1tenth_gym,
recording rows in the exact format init_SS_from_data expects:

    time, x, y, yaw, vel, accel_cmd, steer_cmd, s

`time` resets at each start-line crossing (the loader splits laps on that),
and recording stops after the second crossing — same as the C++ recorder.

Also writes <Map>_waypoints.csv (plain "x,y" rows, no header) because the
LMPC CSVReader calls stof on every row and would choke on the
f1tenth_racetracks header line.

Usage:
    ../.venv/bin/python record_initial_ss.py --map barc_oval
"""

import argparse
import csv
import math
import os

import numpy as np
import yaml

import gym
from f110_gym.envs.base_classes import Integrator

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
MAPS_DIR = os.path.join(REPO, "data", "maps")
PARAMS_YAML = os.path.join(REPO, "Lmpc_params.yaml")

CONTROL_DT = 0.05
PHYSICS_DT = 0.01
STEPS = int(round(CONTROL_DT / PHYSICS_DT))
LOOKAHEAD = 0.9         # pure pursuit lookahead [m]
KP_SPEED = 2.0          # speed P gain -> accel command


def load_centerline(map_name):
    path = os.path.join(MAPS_DIR, map_name, f"{map_name}_centerline.csv")
    pts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            pts.append([float(row[0]), float(row[1])])
    return np.array(pts)


def densify(pts, spacing=0.05):
    """Closed-loop resample at fixed spacing (matches Track's 0.05 m grid)."""
    closed = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    length = s[-1]
    u = np.arange(0, length, spacing)
    dense = np.column_stack([np.interp(u, s, closed[:, 0]),
                             np.interp(u, s, closed[:, 1])])
    return dense, length


def gym_car_params(p):
    return {
        "mu": p["friction_coeff"], "C_Sf": p["C_S_front"], "C_Sr": p["C_S_rear"],
        "lf": p["l_cg2front"], "lr": p["l_cg2rear"], "h": p["height_cg"],
        "m": p["mass"], "I": p["moment_inertia"],
        "s_min": -0.4189, "s_max": 0.4189, "sv_min": -3.2, "sv_max": 3.2,
        "v_switch": 7.319, "a_max": 9.51, "v_min": -5.0, "v_max": 20.0,
        "width": 0.31, "length": 0.58,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True,
                    help="folder name under data/maps, or 'levinelobby_track' "
                         "for the repo's original map")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--mu", type=float, default=None,
                    help="plant friction while recording (match the experiment)")
    ap.add_argument("--v-target", type=float, default=1.5,
                    help="path-follower cruise speed [m/s]; lower it for low mu")
    ap.add_argument("--out-ss", default=None,
                    help="output csv path override (default: map folder)")
    ap.add_argument("--out-dyn", default=None,
                    help="also write one-step dynamics pairs "
                         "(vx,vy,yawdot,accel,steer,vx1,vy1,yawdot1) for "
                         "warm-starting the residual regression (paper: a few "
                         "slow pure-pursuit laps seed the error model)")
    args = ap.parse_args()
    V_TARGET = args.v_target

    with open(PARAMS_YAML) as f:
        p = {k: float(v) for k, v in yaml.safe_load(f).items()
             if isinstance(v, (int, float))}
    if args.mu is not None:
        p["friction_coeff"] = args.mu
        print(f"recording at plant mu = {args.mu}, v_target = {V_TARGET}")
    m = args.map

    if m == "levinelobby_track":
        map_stem = os.path.join(REPO, "data", "levinelobby_track")
        center_path = os.path.join(REPO, "data", "centerline_waypoints.csv")
        pts = []
        with open(center_path) as f:
            for row in csv.reader(f):
                if row and not row[0].strip().startswith("#"):
                    pts.append([float(row[0]), float(row[1])])
        import numpy as _np
        center = _np.array(pts)
        write_wp = False       # LMPC already uses centerline_waypoints.csv directly
    else:
        map_stem = os.path.join(MAPS_DIR, m, f"{m}_map")
        center = load_centerline(m)
        write_wp = True
    dense, length = densify(center)
    print(f"{m}: {len(center)} centerline pts, track length {length:.1f} m")

    # plain x,y waypoints for the LMPC Track constructor (no header)
    if write_wp:
        wp_out = os.path.join(MAPS_DIR, m, f"{m}_waypoints.csv")
        with open(wp_out, "w", newline="") as f:
            w = csv.writer(f)
            for x, y in center:
                w.writerow([f"{x:.6f}", f"{y:.6f}"])
    else:
        wp_out = "(existing centerline_waypoints.csv, unchanged)"

    # start pose: first centerline point, heading along the track
    d0 = dense[1] - dense[0]
    sx, sy, syaw = dense[0, 0], dense[0, 1], math.atan2(d0[1], d0[0])

    env = gym.make("f110_gym:f110-v0", map=map_stem, map_ext=".png",
                   num_agents=1, timestep=PHYSICS_DT, integrator=Integrator.RK4,
                   params=gym_car_params(p))
    env.reset(np.array([[sx, sy, syaw]]))

    def s_of(x, y):
        i = np.argmin((dense[:, 0] - x) ** 2 + (dense[:, 1] - y) ** 2)
        return i * 0.05

    ss_out = args.out_ss or os.path.join(MAPS_DIR, m, f"{m}_initial_safe_set.csv")
    fout = open(ss_out, "w", newline="")
    writer = csv.writer(fout)

    t, lap, s_prev = 0, 0, s_of(sx, sy)
    sim_time, max_time = 0.0, 60.0 + args.laps * length / V_TARGET * 2.5
    dyn_rows, prev_dyn = [], None   # (vx, vy, yawdot, accel, steer) of step k
    DYN_V_GATE = 0.3                # skip standstill/launch transients
    while True:
        st = env.sim.agents[0].state
        x, y, v, yaw = st[0], st[1], st[3], st[4]
        yawdot, slip = st[5], st[6]
        vx, vy = v * math.cos(slip), v * math.sin(slip)
        s_curr = s_of(x, y)

        # lap crossing: same wrap test as the C++ recorder
        if s_curr - s_prev < -length / 2:
            lap += 1
            t = 0
            if lap > args.laps - 1:
                break
        s_prev = s_curr

        # pure pursuit on the dense centerline
        i_look = int(round((s_curr + LOOKAHEAD) / 0.05)) % len(dense)
        gx, gy = dense[i_look]
        dx, dy = gx - x, gy - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        ld = math.hypot(dx, dy)
        steer = math.atan2(2.0 * p["wheelbase"] * math.sin(alpha), max(ld, 1e-3))
        steer = max(-0.41, min(0.41, steer))
        accel = max(-2.0, min(2.0, KP_SPEED * (V_TARGET - v)))

        writer.writerow([t, f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}", f"{v:.6f}",
                         f"{accel:.6f}", f"{steer:.6f}", f"{s_curr:.6f}"])
        t += 1

        # one-step dynamics pair: (state_k, action_k) -> state_{k+1}
        if args.out_dyn:
            if prev_dyn is not None and \
               math.hypot(prev_dyn[0], prev_dyn[1]) > DYN_V_GATE and \
               math.hypot(vx, vy) > DYN_V_GATE:
                dyn_rows.append(list(prev_dyn) + [vx, vy, yawdot])
            prev_dyn = (vx, vy, yawdot, accel, steer)

        action = np.array([[steer, accel]])
        for _ in range(STEPS):
            obs, _, _, _ = env.step(action)
            sim_time += PHYSICS_DT
        if obs["collisions"][0]:
            raise RuntimeError(f"path follower crashed at ({x:.2f},{y:.2f}) — "
                               f"tune LOOKAHEAD/V_TARGET for map {m}")
        if sim_time > max_time:
            raise RuntimeError("recorder timed out before completing laps")

    fout.close()
    print(f"wrote {wp_out}")
    print(f"wrote {ss_out} ({lap} laps, {sim_time:.1f} s sim)")
    if args.out_dyn:
        with open(args.out_dyn, "w", newline="") as f:
            w = csv.writer(f)
            for r in dyn_rows:
                w.writerow([f"{v_:.6f}" for v_ in r])
        print(f"wrote {args.out_dyn} ({len(dyn_rows)} dynamics pairs)")


if __name__ == "__main__":
    main()
