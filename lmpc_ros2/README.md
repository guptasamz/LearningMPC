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

**Required before Section 4** — the bundled `gold_conference_room` track ships a centerline but
no safe set yet; `lmpc_node` throws on startup without one. Seed it once (below), then move to
Section 4. Skip only if you've already seeded a `<track_dir>/<track_name>_initial_safe_set.csv`
for the track you're about to run.

`lmpc_node` needs an `_initial_safe_set.csv` for its track before it can run at all.
`pure_pursuit_node` produces one: drives a `_centerline.csv` via pure pursuit + a capped-speed
P-controller, records the CSV `LMPCCore` reads, exits after `laps` laps (default 2). Its speed
cap comes from `config/lmpc_params.yaml`'s `max_speed` — **required at the node level** (throws
if `<= 0`).

Seed the bundled `gold_conference_room` track — a `seed` compose service does this directly:
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
`/ego_racecar/odom`, `/map`) and **`lmpc`** (`lmpc_node`, track `gold_conference_room`; publishes
`/drive`).
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

**Default mismatch, already handled:** upstream `f1tenth_gym_ros` defaults to its own Levine map;
`lmpc_ros2` defaults to `gold_conference_room` — left alone, that's a QP-infeasible mismatch out of
the box. The Dockerfile (Section 2's build) already patches `f1tenth_gym_ros/config/sim.yaml`'s
`map_path` and `sx`/`sy`/`stheta` to `gold_conference_room`'s own map and start pose at image-build
time, so this isn't something you do by hand for the default track. Only relevant if you point
`track_name`/`track_dir` at a *different* track (the bind-mount example above): make the same edit
yourself — `map_path`/`sx`/`sy`/`stheta` to that track's own map/start pose, then rebuild (or
bind-mount an edited `sim.yaml` for faster iteration).

## 5. Tuning

Params live in `config/lmpc_params.yaml`, self-contained under this package — the **only** file
to edit for the whole LMPC process, both nodes (`lmpc_node`'s section and `pure_pursuit_node`'s
section). It started as a transcription of `../Lmpc_params.yaml` (ROS1/gym's own file, outside
this package), but `N`, `Ts`, `STEER_MAX`, `r_accel`/`r_steer`, and the vehicle physical params
now **intentionally diverge** from it — see the next paragraph and the file's own inline
comments for why. Retune both files by hand if you want a given change to apply to both; they're
intentionally separate, not shared across that boundary.
- `r_accel`/`r_steer`/`r_d_accel`/`r_d_steer` — cost weights.
- `osqp_max_iter`/`osqp_time_limit` — don't shrink `osqp_max_iter` below 20000 (large tracks need
  it). Bound worst-case latency via `osqp_time_limit` instead.
- `dynamics_model: 1` — kinematic + online residual regression instead of known-dynamics.
- `max_speed` — `pure_pursuit_node`'s speed cap while seeding (Section 3). Shared across tracks
  unless overridden per-run with `max_speed:=<value>`.
- Vehicle physical params (`friction_coeff`, `C_S_front`/`rear`, `height_cg`, `mass`,
  `moment_inertia`) currently match `f1tenth_gym_jl`'s (and `DA_MCTS_new_implementation`'s)
  generic f110_gym simulator defaults, **not** a real-car measurement — the file this repo
  originally shipped with (`../Lmpc_params.yaml`) labels its own values as actual measurements of
  car "lidart". Revisit before trusting the dynamics model on real hardware.

Needs an image rebuild (Section 2) to take effect; re-check timing after any change.

Tuned around `Ts: 0.025`, `N: 50` (`N * Ts` = 1.25s horizon, matching `f1tenth_gym_jl`'s
`HORIZON_STEPS=50`/`dt=0.025` on the same `barc_oval` track — deliberately ported together as a
set, not mixed with the old `N=25`/`Ts=0.05`-era weights). This is a **larger, more expensive QP**
than what was last measured (`≈4.5ms mean / ≈21-31ms max`, itself from `Ts: 0.05` testing that
doesn't carry over either) — this exact configuration's timing is unverified. Re-check the
overrun log (Section 4) before trusting it on any real hardware or track.

## 6. Real car deployment

Docker not required — build natively on the car's compute.

**Prerequisites:** ROS2 + `colcon`, `rclcpp nav_msgs ackermann_msgs ament_index_cpp launch
launch_ros`.

**Build — two steps, in order. Skipping step 1 is the most common failure here**
(`CMake Error ... Could not find a package configuration file provided by "osqp"`):

1. Build Eigen 3.4/OSQP v0.6.3/osqp-eigen v0.8.1 into `../src_gym/deps/` — `lmpc_ros2/CMakeLists.txt`
   looks for them there via `CMAKE_PREFIX_PATH` and won't configure without them:
   ```bash
   cd ../src_gym && ./build.sh && cd -
   ```
   This also builds a Python module (`lmpc_core`) this package doesn't need — that's fine, ignore
   it; if `build.sh` errors out on that *last* step specifically (e.g. no `.venv`/pybind11
   available), the Eigen/OSQP/osqp-eigen deps it needed to build first are already done and
   `lmpc_ros2` will still find them.
2. Now build this package:
   ```bash
   colcon build --packages-select lmpc_ros2
   source install/setup.bash
   ```

**Before running anything below**, after a clean sim pass (Section 4):
1. Confirm your real localization topic's name (`ros2 topic list`) — used as `pose_topic` below.
2. Node reads `twist.linear.{x,y}` as body velocity, `twist.angular.z` as yaw rate — if your
   source only gives `vx`, `vy` reads near-zero (an approximation, not a guarantee).
3. Confirm what publishes `map_topic` (transient-local `OccupancyGrid`) on your car — a
   map_server/localization node, not anything this package provides. Must already be running
   before either command below.
4. Have a physical e-stop within reach — this package provides none.

**1. Seed a safe set for your venue**, if you don't already have one (same requirement as
Section 3 — needs `<track_dir>/<track_name>_centerline.csv` already present):
```bash
ros2 launch lmpc_ros2 pure_pursuit.launch.py \
  pose_topic:=/pf/pose/odom \
  track_dir:=/path/to/venue track_name:=venue \
  max_speed:=1.0
```
Bench test first (wheels off the ground) — this node drives the car. `max_speed` has no
node-level default; set it explicitly and keep it low. Wait for `"wrote ... -- shutting down"`
in the log before moving on.

**2. Run the controller:**
```bash
ros2 launch lmpc_ros2 lmpc.launch.py \
  pose_topic:=/pf/pose/odom map_topic:=/map \
  track_dir:=/path/to/venue track_name:=venue
```
Bench test this one too before an actual track run — confirm `/drive` looks physically sane
(`ros2 topic echo /drive`) with the wheels off the ground first.

`drive.speed` carries a raw commanded **acceleration**, published directly — matching this
project's vendored/patched `f1tenth_gym` (`base_classes.py`'s `RaceCar::update_pose()` treats the
field that way, not as a target velocity). Confirm your VESC/ackermann bridge expects an
acceleration in that field, not a target speed — if it wants the latter, that's a small change in
`control_tick()` (`src/lmpc_node.cpp`), not the reverse.
