"""Run the original C++ LearningMPC controller on f1tenth_gym.

The controller is the UNMODIFIED logic of src/LMPC.cpp compiled into the
lmpc_core python extension (see cpp/lmpc_core.cpp and README.md). This script
only does what ROS did around it: feed state in, apply the returned
(acceleration, steering angle) to the simulator at 20 Hz, and log laps.

Usage:
    ../.venv/bin/python lmpc_gym.py [--render] [--laps 30] [--out results]
"""

import argparse
import csv
import math
import os
import time

import numpy as np
import yaml
from PIL import Image

import gym
from f110_gym.envs.base_classes import Integrator

import matplotlib
matplotlib.use("Agg")  # file-only plotting: no window, no clash with pyglet render
import matplotlib.pyplot as plt

import lmpc_core

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
PARAMS_YAML = os.path.join(REPO, "Lmpc_params.yaml")
MAP_STEM = os.path.join(REPO, "data", "levinelobby_track")  # + .png/.yaml
WAYPOINT_CSV = os.path.join(REPO, "data", "centerline_waypoints.csv")
INIT_SS_CSV = os.path.join(REPO, "data", "initial_safe_set.csv")

CONTROL_DT = 0.05      # LMPC Ts (20 Hz), matches ros::Rate(20) in main()
PHYSICS_DT = 0.01      # gym physics timestep
STEPS_PER_CONTROL = int(round(CONTROL_DT / PHYSICS_DT))


def load_occupancy_grid(map_stem):
    """Convert the map png+yaml into a ROS-map_server-equivalent occupancy grid.

    map_server rule (negate=0): p = (255 - value)/255; p > occupied_thresh ->
    100, p < free_thresh -> 0, else -1. Grid data is row-major with row 0 at
    the map origin (bottom of the image), exactly like nav_msgs/OccupancyGrid.
    """
    with open(map_stem + ".yaml") as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(map_stem + ".png").convert("L"), dtype=np.float64)
    p = (255.0 - img) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p > meta["occupied_thresh"]] = 100
    grid[p < meta["free_thresh"]] = 0
    grid = np.flipud(grid)  # image row 0 is top; grid row 0 is origin (bottom)
    h, w = grid.shape
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return grid.flatten(), w, h, float(meta["resolution"]), ox, oy


def numeric_params(yaml_path):
    with open(yaml_path) as f:
        raw = yaml.safe_load(f)
    return {k: float(v) for k, v in raw.items() if isinstance(v, (int, float))}


def gym_car_params(p):
    """f1tenth_gym vehicle params, with the physical constants set to the SAME
    values Lmpc_params.yaml gives the controller (plant == model, as in the
    original racecar_simulator setup). Remaining keys keep gym defaults."""
    return {
        "mu": p["friction_coeff"],
        "C_Sf": p["C_S_front"],
        "C_Sr": p["C_S_rear"],
        "lf": p["l_cg2front"],
        "lr": p["l_cg2rear"],
        "h": p["height_cg"],
        "m": p["mass"],
        "I": p["moment_inertia"],
        # gym defaults below
        "s_min": -0.4189, "s_max": 0.4189,
        "sv_min": -3.2, "sv_max": 3.2,
        "v_switch": 7.319, "a_max": 9.51,
        "v_min": -5.0, "v_max": 20.0,
        "width": 0.31, "length": 0.58,
    }


def first_safe_set_pose(path):
    with open(path) as f:
        row = next(csv.reader(f))
    return float(row[1]), float(row[2]), float(row[3])  # x, y, yaw


BLUE, INK, MUTED, SURFACE = "#2a78d6", "#1a1a19", "#6b6a63", "#fcfcfb"


def write_lap_plot(lap_indices, lap_times, out_png, crashed=False):
    """Regenerate the lap-time curve. Called after every completed lap, so an
    image viewer (e.g. the VSCode image tab) shows it updating live."""
    fig, ax = plt.subplots(figsize=(8, 5), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.plot(lap_indices, lap_times, color=BLUE, lw=2, marker="o", ms=4.5,
            mec="white", mew=0.8)
    if lap_times:
        ax.annotate(f"{lap_times[-1]:.2f} s", (lap_indices[-1], lap_times[-1]),
                    xytext=(-4, 10), textcoords="offset points",
                    color=INK, fontsize=10, ha="right")
    ax.set_xlabel("lap (LMPC iteration)", color=MUTED)
    ax.set_ylabel("lap time [s]", color=MUTED)
    tag = " — CRASHED" if crashed else ""
    ax.set_title(f"LearningMPC on f1tenth_gym — {len(lap_times)} laps{tag}",
                 color=INK, fontsize=12, pad=10)
    ax.grid(True, color="#e4e3dc", lw=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for s in ax.spines.values():
        s.set_color("#d8d7d0")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", action="store_true", help="open the pyglet view")
    ap.add_argument("--laps", type=int, default=30, help="number of LMPC laps to run")
    ap.add_argument("--max-sim-time", type=float, default=900.0)
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    ap.add_argument("--rd", type=float, default=None,
                    help="override control-rate weight: sets r_d_accel = r_d_steer = RD")
    ap.add_argument("--map", default=None,
                    help="map folder under data/maps (expects <map>_map.png/.yaml, "
                         "<map>_waypoints.csv, <map>_initial_safe_set.csv from "
                         "record_initial_ss.py). Default: levinelobby_track.")
    ap.add_argument("--speed-max", type=float, default=None,
                    help="override SPEED_MAX (QP velocity upper bound) for this run")
    ap.add_argument("--map-margin", type=float, default=None,
                    help="override MAP_MARGIN (occupancy inflation) for this run; "
                         "needed on narrow tracks where the default would seal the corridor")
    ap.add_argument("--mu", type=float, default=None,
                    help="override friction for BOTH the gym plant and the LMPC "
                         "dynamics model (sets friction_coeff; plant mu derives from it)")
    ap.add_argument("--ss-file", default=None,
                    help="override the initial safe set csv (e.g. one recorded "
                         "at the experiment's mu)")
    ap.add_argument("--qs", type=float, default=None,
                    help="override q_s (track-slack weight) for this run")
    ap.add_argument("--r-accel", type=float, default=None,
                    help="override r_accel (input effort weight on acceleration)")
    ap.add_argument("--r-steer", type=float, default=None,
                    help="override r_steer (input effort weight on steering)")
    args = ap.parse_args()

    global MAP_STEM, WAYPOINT_CSV, INIT_SS_CSV
    if args.map:
        d = os.path.join(REPO, "data", "maps", args.map)
        MAP_STEM = os.path.join(d, f"{args.map}_map")
        WAYPOINT_CSV = os.path.join(d, f"{args.map}_waypoints.csv")
        INIT_SS_CSV = os.path.join(d, f"{args.map}_initial_safe_set.csv")
    if args.ss_file:
        INIT_SS_CSV = args.ss_file
        print(f"initial safe set override: {args.ss_file}")

    params = numeric_params(PARAMS_YAML)
    if args.rd is not None:
        params["r_d_accel"] = args.rd
        params["r_d_steer"] = args.rd
        print(f"control-rate cost override: r_d = {args.rd}")
    if args.speed_max is not None:
        params["SPEED_MAX"] = args.speed_max
        print(f"SPEED_MAX override: {args.speed_max} m/s")
    if args.map_margin is not None:
        params["MAP_MARGIN"] = args.map_margin
        print(f"MAP_MARGIN override: {args.map_margin} m")
    if args.mu is not None:
        params["friction_coeff"] = args.mu   # controller model AND plant (see below)
        print(f"friction override: mu = {args.mu} (controller model + gym plant)")
    if args.qs is not None:
        params["q_s"] = args.qs
        print(f"q_s override: {args.qs}")
    if args.r_accel is not None:
        params["r_accel"] = args.r_accel
        print(f"r_accel override: {args.r_accel}")
    if args.r_steer is not None:
        params["r_steer"] = args.r_steer
        print(f"r_steer override: {args.r_steer}")
    grid, w, h, res, ox, oy = load_occupancy_grid(MAP_STEM)
    sx, sy, syaw = first_safe_set_pose(INIT_SS_CSV)

    core = lmpc_core.LMPCCore(
        params=params, grid_data=grid, width=w, height=h,
        resolution=res, origin_x=ox, origin_y=oy,
        waypoint_file=WAYPOINT_CSV, init_data_file=INIT_SS_CSV,
        x0=sx, y0=sy, yaw0=syaw)
    print(f"track length: {core.track_length():.2f} m")

    env = gym.make("f110_gym:f110-v0", map=MAP_STEM, map_ext=".png",
                   num_agents=1, timestep=PHYSICS_DT, integrator=Integrator.RK4,
                   params=gym_car_params(params))  # plant mu = params['friction_coeff']

    if args.render:
        def render_callback(e):
            x = e.cars[0].vertices[::2]
            y = e.cars[0].vertices[1::2]
            top, bottom, left, right = max(y), min(y), min(x), max(x)
            e.score_label.x = left
            e.score_label.y = top - 700
            e.left = left - 800
            e.right = right + 800
            e.top = top + 800
            e.bottom = bottom - 800
        env.add_render_callback(render_callback)

    obs, _, done, _ = env.reset(np.array([[sx, sy, syaw]]))
    if args.render:
        env.render()

    os.makedirs(args.out, exist_ok=True)
    run_tag = time.strftime("%Y%m%d_%H%M%S")
    step_log = open(os.path.join(args.out, f"steps_{run_tag}.csv"), "w", newline="")
    step_writer = csv.writer(step_log)
    step_writer.writerow(["sim_time", "x", "y", "yaw", "v", "s", "accel_cmd",
                          "steer_cmd", "use_dyn", "solved", "iter",
                          # model-prediction audit: QP's predicted state for the
                          # NEXT control instant vs what the sim actually did
                          "pred_x", "pred_y", "pred_yaw", "pred_v",
                          "pred_yawdot", "pred_slip"])
    lap_log_path = os.path.join(args.out, f"laps_{run_tag}.csv")
    lap_log = open(lap_log_path, "w", newline="")
    lap_writer = csv.writer(lap_log)
    lap_writer.writerow(["lap_index(iter)", "lap_time_s", "mean_speed_mps"])

    sim_time = 0.0
    lap_start = 0.0
    last_iter = core.iter()
    lap_times = []
    lap_indices = []
    plot_png = os.path.join(args.out, "lap_times.png")
    crashed = False
    solve_fails = 0
    t_wall = time.time()
    print(f"live lap-time plot: {plot_png} (updates after every lap)")

    while True:
        st = env.sim.agents[0].state  # [x, y, steer, v, yaw, yaw_rate, slip]
        x, y, v, yaw, yawdot, slip = st[0], st[1], st[3], st[4], st[5], st[6]
        core.set_state(x=x, y=y, yaw=yaw,
                       vx=v * math.cos(slip), vy=v * math.sin(slip),
                       yawdot=yawdot)

        accel, steer, solved = core.step()
        if not solved:
            solve_fails += 1

        if core.iter() != last_iter:  # crossed the start line -> lap done
            lap_time = sim_time - lap_start
            dist = core.track_length()
            lap_times.append(lap_time)
            lap_indices.append(last_iter)
            lap_writer.writerow([last_iter, f"{lap_time:.3f}",
                                 f"{dist / lap_time:.3f}"])
            lap_log.flush()
            print(f"lap {last_iter} (iter): {lap_time:.2f} s   "
                  f"mean {dist / lap_time:.2f} m/s")
            write_lap_plot(lap_indices, lap_times, plot_png)
            lap_start = sim_time
            last_iter = core.iter()
            if len(lap_times) >= args.laps:
                break

        pred = core.predicted_states()[1]  # x_1 = model's state 0.05 s ahead

        action = np.array([[steer, accel]])  # gym order; accel via modified base_classes
        for _ in range(STEPS_PER_CONTROL):
            obs, _, done, _ = env.step(action)
            sim_time += PHYSICS_DT
            if obs["collisions"][0]:
                crashed = True
                break
        step_writer.writerow([f"{sim_time:.2f}", f"{x:.4f}", f"{y:.4f}",
                              f"{yaw:.4f}", f"{v:.3f}", f"{core.s_curr():.3f}",
                              f"{accel:.4f}", f"{steer:.4f}",
                              int(core.use_dyn()), int(solved), core.iter(),
                              f"{pred[0]:.4f}", f"{pred[1]:.4f}", f"{pred[2]:.4f}",
                              f"{pred[3]:.4f}", f"{pred[4]:.4f}", f"{pred[5]:.4f}"])
        if args.render:
            env.render(mode="human")
        if crashed:
            print(f"CRASH at sim_time {sim_time:.2f}s, lap {core.iter()}, "
                  f"pos ({x:.2f},{y:.2f}), v {v:.2f}")
            break
        if sim_time > args.max_sim_time:
            print("sim time limit reached")
            break

    step_log.close()
    lap_log.close()
    write_lap_plot(lap_indices, lap_times, plot_png, crashed=crashed)
    if args.map:
        try:
            from plot_lap_trajectories import make_plot
            traj_png = make_plot(args.map, args.out)
            if traj_png:
                print(f"lap trajectory plot: {traj_png}")
        except Exception as e:  # plotting must never kill a finished run
            print(f"trajectory plot skipped: {e}")
    print(f"\n==== summary ====")
    print(f"laps completed: {len(lap_times)}  crashed: {crashed}  "
          f"solver failures: {solve_fails}")
    if lap_times:
        print("lap times:", " ".join(f"{t:.1f}" for t in lap_times))
        print(f"first: {lap_times[0]:.2f}s  best: {min(lap_times):.2f}s")
    print(f"wall time: {time.time() - t_wall:.1f}s, lap log: {lap_log_path}")


if __name__ == "__main__":
    main()
