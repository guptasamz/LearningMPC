# lmpc_ros2

ROS2 (rclcpp) wrapper around `LMPCCore` (`../src_gym/cpp/lmpc_core.cpp`), same controller as
`src/LMPC.cpp`, plus the `online_training/` residual-regression extension. No Python in the
control loop — `lmpc_node` is a plain C++ executable linking Eigen/OSQP/osqp-eigen directly.

Two executables, meant to run as separate steps:
- **`lmpc_node`** — the LMPC controller. Needs a pre-recorded initial safe set to run at all.
- **`pure_pursuit_node`** — drives a centerline via pure pursuit and *produces* that initial safe
  set (see Section 5). No `LMPCCore`/Eigen/OSQP dependency; much lighter. Run this first for any
  track that doesn't already have one.

Targets `f1tenth_gym_ros` first, then the real car. Same nodes either way — only
`pose_topic`/`drive_topic`/`map_topic` differ.

**Docker required for everything below except Section 5 (real car).**

## Not included

- No particle filter — `pose_topic` must already publish `nav_msgs/msg/Odometry`.
- No VESC/ackermann hardware bridge.
- No safety/e-stop layer beyond the controller's own "keep previous solution" fallback.
- No live occupancy updates — map is read once at startup.

## 1. Prerequisites

- Docker + Compose v2 (`docker compose version`).
- Optional: NVIDIA Container Toolkit for GPU-accelerated rviz2 (drop the `deploy.resources...`
  block in `docker/docker-compose.yml` to skip it).
- Optional: X11 display for the rviz2 GUI (works out of the box on WSLg/native Linux).

Everything else (ROS2 Humble, `f1tenth_gym_ros`, Eigen/OSQP/osqp-eigen) is built into the image.

`f1tenth_gym_ros` officially pins Foxy; this image builds it from source against Humble instead,
to match `lmpc_ros2`'s rclcpp usage.

## 2. Build

From the **repo root**:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml build
```

First build takes several minutes. Any repo file change invalidates the cache from `COPY .`
onward, since the build context is the whole repo.

## 3. Run

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml up
```

Starts two containers:
- **`sim`** — `f1tenth_gym_ros` (simulator, rviz2, map_server). Publishes `/ego_racecar/odom`, `/map`.
- **`lmpc`** — `lmpc_node`, default track `barc_oval`, `pose_topic:=/ego_racecar/odom`. Publishes `/drive`.

Background: `up -d`, then `logs -f`. Stop: `down`.

Different track:
```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml run --rm lmpc \
  ros2 launch lmpc_ros2 lmpc.launch.py track_dir:=/path/to/your/track_dir track_name:=my_track
```
(`track_dir` must contain `<track_name>_waypoints.csv` / `<track_name>_initial_safe_set.csv`, and
be visible inside the container. `track_name` defaults to `barc_oval`, matching the bundled data.
If you don't have an `_initial_safe_set.csv` for this track yet, see Section 5.)

**Verify:**
```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml exec lmpc bash -lc "ros2 topic hz /drive"
```
~20 Hz (`Ts: 0.05`), with changing `steering_angle`/`speed` (`topic echo` instead of `hz` to see
values). In the `lmpc` logs:
- `"LMPCCore initialized ..."` — map arrived, controller is up. Missing → check `/map` durability.
- `"control step took ... over Ts budget"` — occasional is fine; frequent means re-tune (Section 4).
- `"QP solve failed -- reapplying previous control"` — fallback firing; frequent means track/map mismatch.

**Default map/track mismatch:** `f1tenth_gym_ros` defaults to its own **Levine** map, `lmpc_ros2`
defaults to the bundled **`barc_oval`** waypoints — different tracks, so you'll see frequent QP
warnings above out of the box. To fix: point `f1tenth_gym_ros`'s `config/sim.yaml` `map_path` at
`data/barc_oval/barc_oval_map` (drop `.png`) and set start pose `sx/sy/stheta: 0.0`, then rebuild
(or bind-mount the edited `sim.yaml` over the installed one for faster iteration).

## 4. Tuning

Params live in `config/lmpc_params.yaml` (mirrors `../Lmpc_params.yaml`). Notable:
- `r_accel`/`r_steer`/`r_d_accel`/`r_d_steer` — cost weights.
- `osqp_max_iter`/`osqp_time_limit` — don't shrink `osqp_max_iter` below 20000 (large tracks need
  it — 102 solver failures at 4000, 0 at 20000). Bound worst-case latency via `osqp_time_limit`
  instead.
- `dynamics_model: 1` — kinematic-nominal + online residual regression (`../online_training/`)
  instead of the known-dynamics model.

Changes need an image rebuild (Section 2) to take effect. Re-check timing after any change.

Tuned around `Ts: 0.05` (20Hz). Dev-machine measurement on `barc_oval`: mean ≈4.5ms, p95 ≈7.7ms,
max ≈21-31ms (bounded by `osqp_time_limit`). Real car compute is likely slower than a dev laptop,
and other tracks solve differently — always re-check the overrun log on your actual target
hardware/track, don't assume these numbers transfer. Docker itself adds negligible overhead.

## 5. Generating a fresh initial safe set for a new track

`lmpc_node` can't run without an `_initial_safe_set.csv` for whatever track it's pointed at — the
bundled one only covers `barc_oval`. `pure_pursuit_node` produces a fresh one: it drives a
`_centerline.csv` via pure pursuit + a capped-speed P-controller and records the same CSV format
`LMPCCore` reads, stopping (and exiting) after `laps` laps (default 2). Same node either way, sim
or real car — only `pose_topic`/`drive_topic` differ, same as `lmpc_node`.

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml run --rm lmpc \
  ros2 launch lmpc_ros2 pure_pursuit.launch.py \
  track_dir:=/path/to/your/track_dir track_name:=my_track max_speed:=2.0
```

`max_speed` has **no default — it's required**, both at the launch-argument level and inside the
node itself (throws if `<= 0`). Keep it low, especially the first time on a new track or on the
real car. It's used both as the P-controller's cruise target and as a hard clamp on the published
`drive.speed`, so the car is never commanded faster than it by construction.

Requires `<track_dir>/<track_name>_centerline.csv` to already exist (raw waypoints, format:
`# x_m, y_m, w_tr_right_m, w_tr_left_m` header comment then comma-separated rows — see the
bundled `data/barc_oval/barc_oval_centerline.csv` for a concrete example, also used by the
default sim command below). **This repo has no
tool to generate that file from a new venue's map** — you'll need to produce it some other way
(e.g. mapping the track and extracting a centerline externally) before this section applies. The
node writes `<track_dir>/<track_name>_initial_safe_set.csv` — the same path `lmpc.launch.py`
reads when given the same `track_dir`/`track_name` — so the natural flow is: run this section,
wait for it to exit ("wrote ... -- shutting down" in the log), then run Section 3's `lmpc_node`
command with the same `track_dir`/`track_name`.

On the real car, run the same command with `pose_topic:=/pf/pose/odom` (or whatever your
localization publishes) — see Section 6 first for the general real-car checklist (bench test
before track test applies here too, since this node drives the car).

## 6. Real car deployment

Docker not required here — build natively on the car's own compute.

**Prerequisites:** ROS2 w/ `colcon` (Humble/Jazzy rclcpp), `rclcpp nav_msgs ackermann_msgs
ament_index_cpp launch launch_ros`, and Eigen 3.4/OSQP v0.6.3/osqp-eigen v0.8.1 built into
`../src_gym/deps/` via `../src_gym/build.sh` (only the C++ deps are needed here, not the pybind
module — `build.sh`'s venv step is optional for this package).

**Build:**
```bash
colcon build --packages-select lmpc_ros2
source install/setup.bash
```
(CMake finds Eigen/osqp/OsqpEigen via `CMAKE_PREFIX_PATH` → `../src_gym/deps` automatically.)

**Before deploying**, after a clean sim pass (Section 3):
1. Point `pose_topic` at your real localization output, e.g. `pose_topic:=/pf/pose/odom` — confirm
   the actual topic name first (`ros2 topic list`).
2. Odometry: node reads `twist.linear.{x,y}` as body velocity, `twist.angular.z` as yaw rate. If
   your source only gives `vx`, `vy` just reads near-zero (an approximation, not a guarantee).
3. Bench test first — wheels off the ground, confirm `/drive` values look physically sane.
4. `drive.speed` is an open-loop integration of commanded accel (`speed_cmd += accel * Ts`).
   Confirm your VESC/ackermann bridge expects that, not `drive.acceleration` directly — if not,
   it's a small change in `control_tick()` (`src/lmpc_node.cpp`).
5. Point `map_topic`/`track_dir` at your venue's map and waypoints, not `barc_oval` — if you don't
   have an `_initial_safe_set.csv` for this venue yet, generate one first (Section 5).
6. Have a physical e-stop within reach — this package provides none. This applies to
   `pure_pursuit_node` (Section 5) too, not just `lmpc_node` — both drive the car.
