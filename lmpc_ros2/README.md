# lmpc_ros2

ROS2 wrapper around `LMPCCore` (`../src_gym/cpp/lmpc_core.cpp`). The control loop is C++/`rclcpp`; no Python runs in the controller.

Two executables are provided:

- `pure_pursuit_node`: drives a centerline and records the initial safe set required by LMPC.
- `lmpc_node`: runs the LMPC controller. It will not start without a matching `<track_name>_initial_safe_set.csv`.

This package does not launch the real vehicle stack, localization, VESC bridge, joystick, map server, or e-stop layer. In Docker simulation those are provided by the compose setup. On the real car they must already be running before `lmpc_node` is launched.

## 0. Safety

- Joystick/e-stop must override autonomy before any live run.
- Keep `max_speed` low for first safe-set collection runs, typically `<= 1.0 m/s`.
- Run with wheels off the ground first and inspect `/drive` before putting the car down.
- Confirm the vehicle bridge expects the command semantics this package publishes. `lmpc_node` currently publishes raw commanded acceleration in `drive.speed`, matching this repo's patched gym backend. If your VESC bridge expects target speed, change `src/lmpc_node.cpp` before real driving.

## 1. Topics

`lmpc_node` expects these ROS interfaces:

- `pose_topic`: `nav_msgs/msg/Odometry`. Meaning depends on `pose_source` (default `odom`): pose AND twist directly (the sim case), or (`pose_source:=pf`) only its `|v|` — see below.
- `map_topic`: `nav_msgs/msg/OccupancyGrid`, transient-local map, read once at startup.
- `drive_topic`: `ackermann_msgs/msg/AckermannDriveStamped`, command output.

Default simulation topics are handled by the launch files. On the real car, pass the actual topic names explicitly.

**Real localization publishing `PoseStamped`, not `Odometry`** (e.g. `syn_pf_cpp`'s `/tracked_pose`): set `pose_source:=pf` and `pf_pose_topic:=<that topic>`. `lmpc_node` then takes `x, y, yaw` from `pf_pose_topic`, reconstructs `omega`/`beta` (slip angle) by finite-differencing consecutive PF samples, and takes only `|v|` from `pose_topic`'s `Odometry` twist (real wheel-odometry `vy`/yaw-rate are typically unmeasured — see `vesc_to_odom`'s hardcoded `twist.linear.y = 0` and its no-slip kinematic `angular.z` formula, neither a real measurement of lateral slip). This is a straight port of `f1tenth_ws`'s `DA_MCTS_sim/node.py::_odom_cb`, already validated on this car's real hardware. See `lmpc_node.cpp`'s `reconstruct_from_pf()`.

The `beta` half of that estimate (finite-diff, then projected one step forward through the controller's own dynamics model — see `predict_beta()`) is noisier than `omega`'s, since it differentiates a discretely-sampled pose signal rather than reading a rate directly, so it's opt-in: `beta` is pinned to exactly `0` unconditionally by default. Pass `slip_angle_estimation:=true` to enable the estimate instead; `omega` is unaffected either way.

Typical real-car mapping from the F1TENTH stack:

```text
localization/PF (syn_pf_cpp) -> /tracked_pose -> pf_pose_topic (pose_source:=pf)
wheel odometry (vesc_to_odom) -> /odom        -> pose_topic (|v| only, in pf mode)
map_server                    -> /map         -> map_topic
lmpc_node                     -> /drive       -> ackermann_mux / VESC bridge
```

## 2. Docker Simulation

Docker is the supported path for simulation. It builds ROS2 Humble, `f1tenth_gym_ros`, Eigen, OSQP, and osqp-eigen into the image.

Prerequisites:

- Docker + Compose v2.
- Optional NVIDIA Container Toolkit for GPU RViz. Remove the `deploy.resources...` block in `docker/docker-compose.yml` if unavailable.
- Optional X11 display for RViz.

Build from the repository root:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml build
```

### 2.1 Seed The Initial Safe Set

LMPC needs `<track_dir>/<track_name>_initial_safe_set.csv`. Generate it once per venue/track with `pure_pursuit_node`. Both bundled tracks (`barc_oval` and `gold_conference_room`) already ship a recorded safe set — skip this section for those; go straight to 2.2.

To (re)seed the bundled `gold_conference_room` track anyway, or after editing its centerline:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml --profile seed run --rm seed
```

This drives the simulated car using pure pursuit, writes the safe-set CSV, then exits. It is intentionally not part of normal `docker compose up` because it publishes `/drive` and would fight `lmpc_node`.

For a custom track, bind-mount its directory and pass `track_dir`, `track_name`, and optionally `max_speed`:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml \
  run --rm -v /host/path/to/track:/host/path/to/track seed \
  ros2 launch lmpc_ros2 pure_pursuit.launch.py \
  track_dir:=/host/path/to/track track_name:=my_track max_speed:=1.5
```

The track directory must contain `<track_name>_centerline.csv`. The node writes `<track_name>_initial_safe_set.csv` in the same directory.

### 2.2 Run Closed Loop In Simulation

Start the simulator and LMPC:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml up
```

This starts three services:

- `sim`: `f1tenth_gym_ros`, RViz, map server, `/ego_racecar/odom`, `/map`.
- `pf`: `syn_pf_cpp` (vendored at `lmpc_ros2/third_party/syn_pf_cpp` -- the same particle filter that runs on the real car, see `UPSTREAMS.md`'s `ForzaETH/particle_filter` entry there) running against `sim`'s simulated lidar/map/odom, publishing a genuinely PF-estimated `/tracked_pose`.
- `lmpc`: `lmpc_node`, default `gold_conference_room` track, publishes `/drive`.

`lmpc` runs with `pose_source:=pf` by default -- it takes `x/y/yaw` from `pf`'s `/tracked_pose` and reconstructs `omega`/`beta` by finite-differencing consecutive PF samples, exactly the state-reconstruction path the real car uses (see Section 1), instead of trusting `sim`'s ground-truth twist directly. This means a clean sim run actually exercises the same code path -- and the same PF-driven noise/lag characteristics -- you'll get on the real car, not an artificially perfect one. Same pattern as `f1tenth_ws`'s own `run_verify_real_in_sim.sh`.

Run in the background with:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml up -d
docker compose -f lmpc_ros2/docker/docker-compose.yml logs -f
```

Stop with:

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml down
```

Custom track example (note: overriding the whole command like this replaces `lmpc`'s default `pose_source:=pf`, so it's repeated explicitly here -- drop it to fall back to sim ground truth instead):

```bash
docker compose -f lmpc_ros2/docker/docker-compose.yml \
  run --rm -v /host/path/to/track:/host/path/to/track lmpc \
  ros2 launch lmpc_ros2 lmpc.launch.py \
  track_dir:=/host/path/to/track track_name:=my_track \
  pose_source:=pf pf_pose_topic:=/tracked_pose
```

The directory must contain both `<track_name>_waypoints.csv` and `<track_name>_initial_safe_set.csv`.

Note: `pf`'s config (`lmpc_ros2/third_party/syn_pf_cpp/config/synpf_cpp_params.yaml`) has `gold_conference_room`'s start pose hardcoded (`initial_pose_x/y/theta`) -- for a different track, edit that file (or rebuild with an overriding config) to match, or the PF may fail to converge or lock onto the wrong location.

### 2.3 Simulation Checks

Inside the running ROS environment, verify:

```bash
ros2 topic hz /drive
ros2 topic hz /ego_racecar/odom
ros2 topic echo /drive
```

Expected logs:

- `LMPCCore initialized ...`: controller is up.
- `control step took ... over Ts budget`: occasional is acceptable, frequent means retune before real-car use.
- `QP solve failed ...`: fallback is firing; frequent failures usually mean track/map/safe-set mismatch.

Default map note: the Dockerfile patches upstream `f1tenth_gym_ros/config/sim.yaml` to use `gold_conference_room`. If you run a different track, also point the simulator `map_path`, `sx`, `sy`, and `stheta` at that track or bind-mount an edited `sim.yaml`.

## 3. Real Car With ROS2

Docker is not used on the car. Build natively in the ROS2 workspace.

Prerequisites:

- ROS2 Humble and `colcon`.
- ROS dependencies for this package: `rclcpp`, `nav_msgs`, `ackermann_msgs`, `ament_index_cpp`, `launch`, `launch_ros`.
- A real vehicle stack already running: joystick/e-stop, VESC bridge, lidar, map server, and localization.

### 3.1 Build Native Dependencies

From this repository, build Eigen/OSQP/osqp-eigen into `../src_gym/deps/` first:

```bash
cd ../src_gym
./build.sh
cd -
```

Then build the ROS2 package:

```bash
colcon build --packages-select lmpc_ros2
source install/setup.bash
```

If `build.sh` only fails at the final Python module step, that is usually fine for `lmpc_ros2`; the required C++ dependencies were already built.

### 3.2 Start The Car Stack

Launch the real F1TENTH stack outside this package. In the style of `f1tenth_ws`, the expected sequence is:

```bash
source /opt/ros/humble/setup.bash
source /path/to/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py map:=/path/to/venue.yaml
```

That stack should provide:

- joystick + `joy_teleop` for manual override.
- VESC driver and `ackermann_to_vesc_node` for actuation.
- `vesc_to_odom_node` publishing `/odom`.
- Hokuyo/URG lidar publishing `/scan`.
- `ackermann_mux` for command arbitration.
- static `base_link -> laser` TF.
- `nav2_map_server` publishing `/map`.

Launch localization separately, for example SynPF:

```bash
ros2 launch syn_pf_cpp synpf_cpp_real_launch.py
```

Before LMPC, verify the required streams:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo --once /map
ros2 topic list | grep -E 'tracked_pose|pf|odom'
```

Choose the localization topic(s) you will pass: `pose_topic` (wheel odometry, e.g. `/odom`) and, if your PF publishes `geometry_msgs/msg/PoseStamped` (e.g. `syn_pf_cpp`'s `/tracked_pose`) rather than `nav_msgs/msg/Odometry`, also `pose_source:=pf` and `pf_pose_topic` (see Section 1). Both `lmpc_node` and `pure_pursuit_node` (Section 3.3, below) support this split.

### 3.3 Seed A Real-Venue Safe Set

Skip this only if `<track_dir>/<track_name>_initial_safe_set.csv` already exists and matches the venue map/centerline.

Bench test first with wheels off the ground. Then run slowly:

```bash
ros2 launch lmpc_ros2 pure_pursuit.launch.py \
  pose_topic:=/odom \
  drive_topic:=/drive \
  track_dir:=/path/to/venue \
  track_name:=venue \
  max_speed:=1.0
```
`pose_topic` here must be a real `nav_msgs/msg/Odometry` topic (`pure_pursuit_node` has no `pose_source:=pf` option) -- typically the vehicle's own wheel odometry (e.g. `/odom` from `vesc_to_odom_node`), not the PF.

Wait for the log message that it wrote the safe set and is shutting down. The venue directory must contain `<track_name>_centerline.csv`.

### 3.4 Run LMPC On The Real Car

With vehicle stack, map server, localization, and safe set ready:

```bash
ros2 launch lmpc_ros2 lmpc.launch.py \
  pose_source:=pf \
  pose_topic:=/odom \
  pf_pose_topic:=/tracked_pose \
  map_topic:=/map \
  drive_topic:=/drive \
  track_dir:=/path/to/venue \
  track_name:=venue
```

If your stack publishes localization or wheel odometry somewhere else, replace `/tracked_pose`/`/odom` with those topics. If your PF instead publishes `nav_msgs/msg/Odometry` directly (no separate wheel-odometry topic needed), use `pose_source:=odom` (the default) with `pose_topic` pointed straight at it -- same as the sim case.

Real-car checks:

```bash
ros2 topic hz /drive
ros2 topic echo /drive
ros2 topic hz /tracked_pose
ros2 topic hz /odom
```

Keep the joystick override active. Stop immediately if `/drive` commands are stale, sign-flipped, saturated, or semantically wrong for the VESC bridge.

## 4. Tuning

All LMPC and pure-pursuit parameters live in:

```text
lmpc_ros2/config/lmpc_params.yaml
```

Important parameters:

- `max_speed`: pure-pursuit speed cap while seeding the safe set. Override per run with `max_speed:=<value>`.
- Track half-widths come from two layered sources, both automatic:
  - **Designed widths from the centerline csv** (primary/authoritative): if `<track_name>_centerline.csv` has 4 columns (TUM-style `x, y, w_tr_right, w_tr_left`, `#`-comments allowed), `lmpc_node` passes it to `LMPCCore` as `halfwidth_csv` (see `lmpc.launch.py`) and it caps the ray-marched half-width at `csv_width - MAP_MARGIN` wherever that's smaller. This is what fixes SLAM maps where the ray-march can escape through a doorway/wall gap and report a corridor of many meters (`gold_conference_room` measured up to 13.6m vs. a true ~0.84m before this cap). A 2-column centerline (no width data) degrades gracefully to pure ray-marching.
  - `track_half_width_max`: ceiling on the ray-marched half-width used both as `initialize_width()`'s pre-search default and its fallback when the wall search finds nothing within reach. Default `0.8` (matches the historical constant). Only worth lowering per-track if a centerline csv with real widths isn't available for that track — lowering it further when the csv cap is already active just needlessly under-cuts otherwise-legitimate width. Override per run with `track_half_width_max:=<value>` (`lmpc.launch.py` only; `pure_pursuit_node` doesn't use this).
- `Ts`, `N`: controller period and horizon.
- `r_accel`, `r_steer`, `r_d_accel`, `r_d_steer`: cost weights.
- `osqp_max_iter`, `osqp_time_limit`: solver latency controls. Do not shrink `osqp_max_iter` below `20000` unless you have verified large-track solves.
- `dynamics_model: 1`: kinematic model plus online residual regression.
- Vehicle physical params: currently match simulator defaults, not necessarily the measured real car. Revisit before trusting hardware dynamics.

Rebuild the Docker image after parameter edits for simulation. Rebuild the ROS2 package natively after source changes on the car.
