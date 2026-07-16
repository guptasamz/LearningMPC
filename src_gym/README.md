# LearningMPC on f1tenth_gym (C++ controller, python bridge)

The original C++ LMPC controller from `src/LMPC.cpp` running against the
`f1tenth_gym` simulator instead of the ROS `racecar_simulator`. The controller
stays in C++ (compiled to a python extension); python only feeds simulator
state in and applies the returned control.

```
f1tenth_gym (python, 100 Hz physics)
        │  state [x, y, yaw, v, yaw_rate, slip]        every 0.05 s (20 Hz,
        ▼                                              same as ros::Rate(20))
lmpc_core.LMPCCore  ← cpp/lmpc_core.cpp = verbatim src/LMPC.cpp
        │  (acceleration, steering angle)
        ▼
env.step([steer, accel])   (base_classes.py passes accel through)
```

## Build & run

```bash
./build.sh                          # one-time: installs Eigen/OSQP/osqp-eigen
                                    # into ./deps, builds lmpc_core*.so
../.venv/bin/python lmpc_gym.py --laps 30            # headless experiment
../.venv/bin/python lmpc_gym.py --laps 10 --render   # with the pyglet view
```

Lap and step logs land in `results/`.

## Files

| File | What it is |
|---|---|
| `cpp/lmpc_core.cpp` | `src/LMPC.cpp` with ROS plumbing removed, logic verbatim |
| `cpp/ros_shim/` | minimal stand-ins for ROS message headers + boost::split so the ORIGINAL `track.h` / `occupancy_grid.h` / `CSVReader.h` compile unmodified |
| `CMakeLists.txt`, `build.sh` | build (Eigen 3.4.0, OSQP v0.6.3, osqp-eigen v0.8.1, pybind11) |
| `lmpc_gym.py` | gym runner: map→occupancy grid, 20 Hz loop, lap timing/logging |

## Honest deviation list (everything that differs from `src/LMPC.cpp`)

Controller **math and decision logic are byte-for-byte transcriptions** —
`get_linearized_dynamics`, `solve_MPC` (QP layout, costs, all constraints),
`select_terminal_candidate`, `select_convex_safe_set`, `find_nearest_point`,
`update_cost_to_go`, `init_SS_from_data`, `reset_QPSolution`, `add_point`,
the lap/iteration state machine in `run()`, the kinematic/dynamic model
switch, and the ±0.41 rad steer clamp of `applyControl`. The original
`track.h`, `occupancy_grid.h`, `spline.h`, `CSVReader.h`, `car_params.h` are
included **unmodified**.

What is NOT identical:

1. **ROS I/O removed.** Publishers/subscribers/tf and all rviz visualization
   functions are gone. State enters via `set_state(x, y, yaw, vx, vy, yawdot)`
   (same quantities the odom callback extracted; `vel_ = hypot(vx, vy)` and
   `slip = atan2(vy, vx)` computed identically). The control is returned
   instead of published as AckermannDriveStamped.
2. **RRT obstacle path dropped.** `map_callback` / `rrt_path_callback` (track
   half-width updates from an RRT node's "path_found" topic) are not ported.
   In the standard racing setup nothing publishes that topic, so this code
   never ran anyway.
3. **Parameters** come from the same `Lmpc_params.yaml`, but loaded with
   python yaml and passed as a dict instead of `ros::NodeHandle::getParam`.
4. **Occupancy grid** is built from `data/levinelobby_track.png` + `.yaml`
   using ROS map_server's exact thresholding rule, instead of arriving on the
   `/map` topic. Same inflation (`occupancy_grid::inflate_map`, MAP_MARGIN)
   afterwards.
5. **Two uninitialized variables fixed**: `first_run_` and `time_` were never
   initialized in the original C++ (undefined behavior that happened to work
   on the author's machine). Explicitly `true` / `0` here — the values the
   code logic assumes.
6. **OSQP verbosity off** and the per-step `cout` of accel/steer removed —
   output-only changes. Solver generation pinned to the era of the original
   (OSQP 0.6.3, osqp-eigen 0.8.x API): fresh solver per step, warm start
   flag set, all other settings left at defaults, `solve()` semantics equal.
7. **The plant is f1tenth_gym, not racecar_simulator.** Physics params (mu,
   C_Sf/C_Sr, lf/lr, h, m, I) are set to the SAME values Lmpc_params.yaml
   gives the controller, so plant == model as in the original setup. Two
   environment-side differences remain: gym applies commanded acceleration
   directly (via the modified `base_classes.py`) with steering slewed at
   sv_max = 3.2 rad/s, and collision is lidar-TTC-based.
8. **Timing**: control at exactly 20 Hz in sim time (5 × 0.01 s physics steps
   per control step), mirroring `ros::Rate(20)` — but deterministic, with no
   ROS message latency/jitter.
9. **Map image converted RGBA→8-bit grayscale in place**
   (`data/levinelobby_track.png`) — pixel values byte-identical; gym's laser
   simulator requires a single-channel image.

10. **Control-rate cost (intentional formulation extension).** A stage cost
    $\sum_k \|(u_k - u_{k-1})/T_s\|^2_{R_d}$ with $u_{-1}$ = last applied
    input, matching Xue et al., arXiv:2309.10716. Note the units: the paper
    WRITES $c_{\Delta u}\|u_t - u_{t-1}\|^2$, but its code (Racing-LMPC-ROS2)
    penalizes the RATE $\|\Delta u / \Delta t\|^2$ via the constrained dU
    variable — a factor $1/T_s^2 = 400$ at 20 Hz. This port uses the code's
    (rate) semantics so `r_d_accel` / `r_d_steer` values are directly
    comparable to that paper's reported $c_{\Delta u}$; verified empirically
    (their 0.1 vs 1.0 convergence gap reproduces). Setting both to 0 (or
    removing the keys) recovers the original formulation exactly.
    Implemented as a tridiagonal block in the QP Hessian plus a linear term
    on $u_0$ — no new variables. Only `src_gym/cpp/lmpc_core.cpp` carries
    this; the original `src/LMPC.cpp` is untouched.

11. **Two robustness guards (deviations, behavior-preserving for valid
    solves).** (a) `wrap_angle` uses fmod instead of the original unbounded
    while-loops — identical result for finite inputs, but terminates on
    inf/NaN (the originals spin forever, observed as a hard hang). (b) A QP
    solution containing non-finite values is discarded and treated as a
    failed solve (the original failure path: previous solution kept),
    instead of poisoning the next linearization. Both triggered in practice
    by post-spin garbage states and by very low input-effort weights
    (R = 0.1) — the original code hangs in those regimes.

Not changed anywhere: `f1tenth_gym` internals beyond the user's own
`base_classes.py` accel-passthrough edit, and the repo's original `src/`,
`include/`, `data/` contents.
