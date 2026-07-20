# lmpc_ros2

ROS2 (rclcpp) wrapper around `LMPCCore` (`../src_gym/cpp/lmpc_core.cpp`, same controller as
`src/LMPC.cpp` + `online_training/`'s residual regression). No Python in the control loop.

Two executables, run as separate steps:
- **`pure_pursuit_node`** — drives a centerline via pure pursuit and produces the initial safe
  set `lmpc_node` needs (Section 3). Run this first for any track that doesn't have one yet.
- **`lmpc_node`** — the LMPC controller. Requires that pre-recorded initial safe set to run at all.

Same nodes for `f1tenth_gym_ros` and the real car — only `pose_topic`/`drive_topic`/`map_topic`
differ. **Docker required except Section 6 (real car).**

## Not included

- Particle filter — `pose_topic` must already publish `nav_msgs/msg/Odometry`.
- VESC/ackermann hardware bridge.
- Safety/e-stop layer beyond the controller's own solve-failure fallback.
- Live occupancy updates — map is read once at startup.

## 1. Prerequisites

- Docker + Compose v2.
- Optional: NVIDIA Container Toolkit (GPU rviz2) — drop the `deploy.resources...` block in
  `docker/docker-compose.yml` to skip it.
- Optional: X11 display for rviz2 (works out of the box on WSLg/native Linux).

Everything else (ROS2 Humble, `f1tenth_gym_ros` built from source against Humble, Eigen/OSQP/
osqp-eigen) is built into the image.

All config for the whole LMPC process — controller tuning *and* `pure_pursuit_node`'s
`max_speed` — lives in one file, self-contained under this package: `config/lmpc_params.yaml`
(Section 5). No separate env-file layer. It's a hand-kept transcription of `../Lmpc_params.yaml`
(ROS1/gym's own file, outside this package) — see its header comment. `max_speed` is currently
one shared value, not per-track — override it for a one-off run with `max_speed:=<value>`
(Section 3) if a different track needs a different cap.

## 2. Build

From the **repo root**:
```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml build
```
First build takes several minutes. Any repo file change invalidates the cache from `COPY .`
onward, since the build context is the whole repo.

## 3. Generating a fresh initial safe set for a new track

**Skip this section if you're just running the bundled `barc_oval` demo** — its safe set is
already included; go straight to Section 4. Needed for any other track.

`lmpc_node` needs an `_initial_safe_set.csv` for its track before it can run at all.
`pure_pursuit_node` produces one: drives a `_centerline.csv` via pure pursuit + a capped-speed
P-controller, records the CSV `LMPCCore` reads, exits after `laps` laps (default 2). Its speed
cap comes from `config/lmpc_params.yaml`'s `max_speed` — **required at the node level** (throws
if `<= 0`).

Reseed the bundled `barc_oval` track — a `seed` compose service does this directly:
```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml --profile seed run --rm seed
```
Not started by plain `up` (Section 4) — it drives the car itself and would fight `lmpc` for
`/drive` if it launched automatically every time, hence the `--profile seed` opt-in.

**New track:** needs its own `_centerline.csv` (Section 3 above) and, if it needs a different
speed cap than `config/lmpc_params.yaml`'s shared `max_speed`, pass `max_speed:=<value>` explicitly.
Point `track_name:=...` (and `track_dir:=...` if it's not bundled in this package's own `data/`)
at it — e.g. `docker compose ... run --rm seed ros2 launch lmpc_ros2 pure_pursuit.launch.py
track_name:=my_track max_speed:=1.5` (bind-mount the directory too if external, same pattern as
Section 4).

The node writes `<track_dir>/<track_name>_initial_safe_set.csv`, the same path `lmpc.launch.py`
reads — so: run this, wait for `"wrote ... -- shutting down"` in the log, then run Section 4.

Real car: same commands with `pose_topic:=/pf/pose/odom` (or your localization topic) — read
Section 6 first, the bench-test-before-track-test rule applies here too.

## 4. Run

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml up
```
Two containers: **`sim`** (`f1tenth_gym_ros` — sim, rviz2, map_server; publishes
`/ego_racecar/odom`, `/map`) and **`lmpc`** (`lmpc_node`, track `barc_oval`; publishes `/drive`).
Background: `up -d` then `logs -f`. Stop: `down`.

Different track (bind-mount its host directory in — not otherwise visible inside the container):
```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml \
  run --rm -v /host/path/to/dir:/host/path/to/dir lmpc \
  ros2 launch lmpc_ros2 lmpc.launch.py \
  track_dir:=/host/path/to/dir track_name:=my_track
```
Needs `<track_name>_waypoints.csv`/`<track_name>_initial_safe_set.csv` in that directory (no safe
set yet? see Section 3, same bind-mount pattern).

**Verify:** `ros2 topic hz /drive` → ~20Hz with changing values. In the `lmpc` logs:
- `"LMPCCore initialized ..."` — controller is up. Missing → check `/map` durability.
- `"control step took ... over Ts budget"` — occasional is fine, frequent → re-tune (Section 5).
- `"QP solve failed ..."` — fallback firing; frequent → track/map mismatch.

**Default mismatch:** `f1tenth_gym_ros` defaults to its own Levine map; `lmpc_ros2` defaults to
`barc_oval` — expect QP warnings out of the box. Fix: point `f1tenth_gym_ros/config/sim.yaml`'s
`map_path` at `data/barc_oval/barc_oval_map` (no `.png`), `sx/sy/stheta: 0.0`, rebuild (or
bind-mount the edited file for faster iteration).

## 5. Tuning

Params live in `config/lmpc_params.yaml`, self-contained under this package — the **only** file
to edit for the whole LMPC process, both nodes (`lmpc_node`'s section and `pure_pursuit_node`'s
section). It's a hand-kept transcription of `../Lmpc_params.yaml` (ROS1/gym's own file, outside
this package, in a different format `rosparam load`/the gym harness need) — retune both if you
change one and want them to match; they're intentionally two separate files, not shared across
that boundary.
- `r_accel`/`r_steer`/`r_d_accel`/`r_d_steer` — cost weights.
- `osqp_max_iter`/`osqp_time_limit` — don't shrink `osqp_max_iter` below 20000 (large tracks need
  it). Bound worst-case latency via `osqp_time_limit` instead.
- `dynamics_model: 1` — kinematic + online residual regression instead of known-dynamics.
- `max_speed` — `pure_pursuit_node`'s speed cap while seeding (Section 3). Shared across tracks
  unless overridden per-run with `max_speed:=<value>`.

Needs an image rebuild (Section 2) to take effect; re-check timing after any change.

Tuned around `Ts: 0.05`. Dev-machine `barc_oval`: mean ≈4.5ms, max ≈21-31ms (bounded by
`osqp_time_limit`). Real hardware is likely slower and other tracks solve differently — always
re-check the overrun log on your actual target before trusting these numbers.

## 6. Real car deployment

Docker not required — build natively on the car's compute.

**Prerequisites:** ROS2 + `colcon`, `rclcpp nav_msgs ackermann_msgs ament_index_cpp launch
launch_ros`, Eigen 3.4/OSQP v0.6.3/osqp-eigen v0.8.1 in `../src_gym/deps/`
(`../src_gym/build.sh` — only the C++ deps are needed, not its pybind/venv step).

**Build:**
```bash
colcon build --packages-select lmpc_ros2
source install/setup.bash
```

**Before deploying**, after a clean sim pass (Section 4):
1. Point `pose_topic` at your real localization topic — confirm the name (`ros2 topic list`).
2. Node reads `twist.linear.{x,y}` as body velocity, `twist.angular.z` as yaw rate — if your
   source only gives `vx`, `vy` reads near-zero (an approximation, not a guarantee).
3. Point `map_topic`/`track_dir` at your venue, not `barc_oval`.
4. **No `_initial_safe_set.csv` for this venue?** Stop and do Section 3 first — not optional.
5. Bench test first (wheels off the ground) for both nodes — confirm `/drive` looks sane.
6. `drive.speed` is an open-loop accel integration (`speed_cmd += accel * Ts`) — confirm your
   VESC/ackermann bridge expects that, not `drive.acceleration` (else: small change in
   `control_tick()`, `src/lmpc_node.cpp`).
7. Have a physical e-stop within reach — this package provides none.
