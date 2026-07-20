# lmpc_ros2

ROS2 (rclcpp) wrapper around `LMPCCore` (`../src_gym/cpp/lmpc_core.cpp`) — the same
byte-identical-to-`src/LMPC.cpp` controller, plus the `online_training/` residual-regression
extension, already validated via `../src_gym/lmpc_gym.py`. **No Python in the runtime control
loop** — `lmpc_node` is a plain C++ executable that includes `lmpc_core.cpp` directly and links
against Eigen/OSQP/osqp-eigen, same as the pybind11 module does, just without pybind11.

Targets `f1tenth_gym_ros` first (safer to validate against), then the real car — the node itself
has no sim-vs-real branches, only `pose_topic`/`drive_topic`/`map_topic` differ between them.

## What this does NOT provide

Read this before assuming anything is handled for you:

- **No particle filter.** `pose_topic` must already be publishing `nav_msgs/msg/Odometry`. In sim
  that's `f1tenth_gym_ros`'s ego-agent odometry; on the real car it's whatever your localization
  stack outputs (commonly a particle filter, e.g. published on `/pf/pose/odom`) — this package
  does not include one.
- **No VESC/ackermann hardware bridge.** This node publishes `ackermann_msgs/AckermannDriveStamped`
  on `drive_topic`. Getting that onto the actual motor/servo is a separate piece of your stack
  (e.g. `vesc_ackermann`).
- **No safety/e-stop layer.** No follow-the-gap, no CBF, no watchdog beyond the controller's own
  internal "keep previous solution" fallback on a failed QP solve. Add one before unsupervised
  real-track running.
- **No live occupancy updates.** The map is read once at startup from `map_topic` and never
  refreshed.

## 1. Prerequisites

- A ROS2 distro with `colcon` (Humble/Jazzy-era rclcpp APIs assumed throughout this package).
- ROS2 packages (via `rosdep` or your distro's package manager): `rclcpp`, `nav_msgs`,
  `ackermann_msgs`, `ament_index_cpp`, `launch`, `launch_ros`.
- Eigen 3.4 / OSQP v0.6.3 / osqp-eigen v0.8.1, built once into `../src_gym/deps/` by
  `../src_gym/build.sh` (this package's `CMakeLists.txt` reuses that same install prefix — see
  its `LMPC_DEPS_DIR` cache variable). If `../src_gym/deps/` doesn't exist yet:
  ```bash
  cd ../src_gym
  # build.sh expects VENV_PY at ../.venv/bin/python; point it elsewhere if you use conda etc.
  VENV_PY=$(which python3) bash -c '...'   # or just run build.sh after creating ../.venv
  ```
  You only need the C++ deps for this package — the Python venv itself is only required to also
  build the `lmpc_core` *pybind11* module for the (optional) gym-side comparison/tuning workflow
  below, not for `lmpc_ros2` itself.

## 2. Build

From a colcon workspace containing this package (e.g. `LearningMPC/lmpc_ros2` symlinked or
copied into `<ws>/src/`):

```bash
colcon build --packages-select lmpc_ros2
source install/setup.bash
```

If CMake can't find `Eigen3`/`osqp`/`OsqpEigen`, build `../src_gym/deps/` first (previous
section) — `lmpc_ros2/CMakeLists.txt` looks there via `CMAKE_PREFIX_PATH` automatically.

## 3. Run against f1tenth_gym_ros

1. Set up `f1tenth_gym_ros` (github.com/f1tenth/f1tenth_gym_ros) in the same workspace, per its
   own instructions, and launch it — confirm it's publishing `/ego_racecar/odom` and a `/map`
   (transient-local `nav_msgs/msg/OccupancyGrid`). It needs a map for its own laser-scan
   simulation, so this is normally already true once its sim is up.
2. In another terminal:
   ```bash
   ros2 launch lmpc_ros2 lmpc.launch.py
   ```
   This defaults to the bundled `barc_oval` track (`data/barc_oval/`) and
   `pose_topic:=/ego_racecar/odom`. If you're driving a different track in the sim, point
   `track_dir` at its `<track>_waypoints.csv`/`<track>_initial_safe_set.csv` pair instead:
   ```bash
   ros2 launch lmpc_ros2 lmpc.launch.py track_dir:=/path/to/your/track_dir
   ```
   (The launch file appends `<track>_waypoints.csv`/`<track>_initial_safe_set.csv` filenames from
   `track_dir`'s own basename — rename your files to match that convention, or edit the launch
   file's `parameters=[...]` block directly for a one-off.)
3. **Confirm it's working:**
   ```bash
   ros2 topic echo /drive
   ```
   should show `speed`/`steering_angle` values changing as the car drives. Watch the node's own
   log output for:
   - `"LMPCCore initialized ..."` — confirms the map arrived and the controller constructed. If
     you never see this, `map_topic` isn't being published (transient-local) yet — check
     `ros2 topic info /map` for a durability mismatch.
   - `"control step took ... over the ... Ts budget"` (throttled to 1/sec) — a real latency
     overrun. Occasional ones are expected (see Tuning below); frequent ones mean the timing
     margin found on the dev machine (Section 5) doesn't hold on yours — re-tune before trusting it.
   - `"QP solve failed -- reapplying previous control"` — harmless in isolation (this is
     `LMPCCore`'s own designed fallback, verified in testing to not cause crashes), but frequent
     occurrences mean something's off (track mismatch, bad initial pose, degenerate map margin).

## 4. Tuning

All controller numbers live in `config/lmpc_params.yaml` (transcribed from `../Lmpc_params.yaml`
— keep them in sync if you retune the Python/gym side too). Common knobs:

- `r_accel` / `r_steer` / `r_d_accel` / `r_d_steer` — cost weights, effort and rate-of-change.
- `osqp_max_iter` / `osqp_time_limit` — see `../Lmpc_params.yaml`'s inline comments. Don't shrink
  `osqp_max_iter` casually — it was raised from 4000 to 20000 specifically because large tracks
  (YasMarina-scale) need it (102 solver failures at 4000, 0 at 20000, replay-verified). Bound
  worst-case latency via `osqp_time_limit` instead — it races `osqp_max_iter` without touching
  that large-track headroom.
- `dynamics_model: 1` switches to the kinematic-nominal + online residual-regression model
  (`../online_training/`) instead of the original known-dynamics kinematic/single-track pair.

After any change, re-validate timing (next section) before trusting it on hardware.

## 5. Timing / real-time budget

This controller is tuned around `Ts: 0.05` (20Hz) — that's the actual per-step budget, not
whatever your lidar's publish rate happens to be. ROS2 already decouples the control timer from
sensor callbacks (this node's `control_tick()` runs on its own best-effort wall timer, always
consuming whichever odometry arrived most recently), so there's no requirement to match a faster
sensor rate.

Dev-machine measurement on `barc_oval` (8 laps, `dynamics_model: 0`, current
`config/lmpc_params.yaml` settings): mean ≈4.5ms, p95 ≈7.7ms, p99 ≈11-12ms, **max ≈21-31ms**
(bounded by `osqp_time_limit`; unbounded it was ≈200ms on a rare infeasible solve). This leaves
real margin under the 50ms `Ts` budget on that machine, but:

- **The real car's onboard compute is very likely slower than a dev laptop**, not faster.
- **This was only measured on `barc_oval`** (a small, ~25m track). A different track's geometry
  changes solve difficulty — re-run the measurement (or just watch for the overrun-warning log,
  Section 3) on whatever track you actually deploy to.
- ROS2's own message handling (deserialization, callback dispatch, publish) adds overhead this
  measurement doesn't capture, since it called `LMPCCore::step()` directly.

**Always re-check the overrun-warning log on your actual target hardware and track before trusting
timing.** Don't assume the numbers above transfer.

## 6. Real car deployment

Only after a clean sim pass (Section 3) with no crashes and an acceptable overrun rate:

1. **`pose_topic`**: point it at your real localization stack's odometry topic, e.g.:
   ```bash
   ros2 launch lmpc_ros2 lmpc.launch.py pose_topic:=/pf/pose/odom map_topic:=/map
   ```
   Confirm the topic name against your actual stack first (`ros2 topic list`) — `/pf/pose/odom`
   above is a common convention, not a guarantee.
2. **Odometry quality**: this node reads `twist.twist.linear.{x,y}` as body-frame velocity and
   `twist.twist.angular.z` as yaw rate directly from whatever `pose_topic` publishes. Many real
   odometry sources don't reliably report lateral velocity (`vy`) — if yours only gives `vx`, the
   controller still runs (`vy` just reads near-zero), but this is an approximation worth knowing
   about, not a guarantee of accuracy.
3. **Bench test before track test**: wheels off the ground, drive enabled, confirm `/drive` values
   look physically sane (no wild steering, accel-derived speed staying in a reasonable range)
   before ever putting the car on the ground.
4. **`drive.speed` semantics**: this node integrates commanded acceleration into an open-loop
   speed command (`speed_cmd += accel * Ts`, clamped to `v_min`/`v_max`), matching how
   `f110_gym`'s own `direct_accel_control` wrapper works in sim. **Verify this matches your actual
   VESC/ackermann bridge's expectation** — some accept `drive.acceleration` directly instead of a
   target `drive.speed`; if yours does, that's a small, localized change in `control_tick()`
   (`src/lmpc_node.cpp`).
5. Make sure whatever provides `map_topic` on the real car (map_server pointed at your venue's
   map, not `barc_oval`) is up and publishing transient-local before launching this node — swap
   `track_dir` to your venue's waypoint/initial-safe-set CSVs at the same time (Section 3).
6. Have a physical e-stop within reach. This package provides none.
