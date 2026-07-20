# online_training — nominal + residual dynamics (Xue et al., arXiv:2309.10716)

## Code
- `nominal_dynamics/` — kinematic bicycle nominal (`kinematic_bicycle.py`) and
  the controller's single-track model as eval baseline (`known_single_track.py`)
- `residual_learnt_dynamics/` — numpy reference of the local error regression
  (`error_regression.py`, math from Racing-LMPC-ROS2 `safe_set.cpp`)
- `cpp/` — C++ port (`error_regression.hpp`, used by both the `residual_core`
  python module and `src_gym/cpp/lmpc_core.cpp`); build with `cpp/build.sh`

## Scripts
- `offline_eval.py` — one-step MSE/RMSE of nominal vs nominal+residual vs
  known-ST on a recorded run (rolling per-lap protocol)
- `validate_beta_course_angle.py` — can beta/v_x/v_y be recovered from PF pose
  alone (Savitzky-Golay course-angle method) under PF noise
- `verify_cpp_port.py` — asserts C++ regressor == numpy regressor (~1e-12)

## Data
- `data/<run>/` — recorded runs (steps CSVs with measured vx/vy/yawdot)
- `data/warmstart/` — 3-lap pure-pursuit dynamics pairs per (map, mu),
  recorded by `src_gym/record_initial_ss.py --out-dyn`, consumed by
  `lmpc_gym.py --reg-warmstart`

## Results (analysis outputs; campaign raw logs live in
## `src_gym/results_residual_dynmics/`)
- `results/beta_validation/` — slip-from-pose validation tables/plots
- `results/offline_eval/` — offline model-accuracy tables/plots
- `results/campaign/` — known-vs-residual campaign comparisons, lap-time
  curves, crash forensics
