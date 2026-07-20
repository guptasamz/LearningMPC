"""Offline evaluation of the nominal + residual dynamics model (part 1).

Protocol:
  * data = steps CSV from lmpc_gym.py with measured columns
    (meas_vx, meas_vy, meas_yawdot); one-step pairs at Ts = 0.05
  * split BY LAP, time-ordered: first laps -> regression buffer (train),
    last laps -> test queries. Mirrors the online use: current lap predicted
    from earlier laps' data.
  * three one-step predictors of (v_x, v_y, omega):
      nominal    kinematic bicycle only (nominal_dynamics/kinematic_bicycle)
      known ST   the controller's full single-track model (baseline;
                 near-exact at matched friction since the plant integrates
                 the same ODEs)
      nominal+residual   kinematic + local error regression
                 (residual_learnt_dynamics/error_regression, math from
                 Racing-LMPC-ROS2 safe_set.cpp)
  * metrics: per-state MSE and RMSE on the test laps.

Usage:
    python offline_eval.py data/sepang_beta_val/steps_*.csv [--test-laps 20]
                           [--dist-max 1.0] [--k-max 256]
"""

import argparse
import glob
import os
import sys
import time

import numpy as np
import pandas as pd
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nominal_dynamics.kinematic_bicycle import KinematicBicycleNominal
from nominal_dynamics.known_single_track import KnownSingleTrack
from residual_learnt_dynamics.error_regression import ErrorDynamicsRegressor

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_YAML = os.path.join(HERE, "..", "Lmpc_params.yaml")
TS = 0.05
V_GATE = 1.0
STATES = ["v_x", "v_y", "omega"]


def load_pairs(path):
    """Return X (n,3), U (n,2), X_next (n,3), lap (n,) one-step pairs."""
    df = pd.read_csv(path)
    need = {"meas_vx", "meas_vy", "meas_yawdot"}
    if not need.issubset(df.columns):
        sys.exit(f"{path}: no measured-state columns (rerun lmpc_gym.py)")
    X = df[["meas_vx", "meas_vy", "meas_yawdot"]].to_numpy(float)
    U = df[["accel_cmd", "steer_cmd"]].to_numpy(float)
    lap = df["iter"].to_numpy(int)
    v = np.hypot(X[:, 0], X[:, 1])
    # pair k -> k+1: valid where both ends are past launch and moving
    ok = (v[:-1] > V_GATE) & (v[1:] > V_GATE)
    ok[:int(2.0 / TS)] = False
    idx = np.where(ok)[0]
    return X[idx], U[idx], X[idx + 1], lap[idx]


def table(errors, mask=None):
    """errors: dict name -> (n,3) prediction errors."""
    rows = []
    for name, e in errors.items():
        if mask is not None:
            e = e[mask]
        for j, s in enumerate(STATES):
            rows.append([name, s, np.mean(e[:, j] ** 2),
                         np.sqrt(np.mean(e[:, j] ** 2))])
    return pd.DataFrame(rows, columns=["model", "state", "MSE", "RMSE"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("steps_csv", nargs="?", default=None)
    ap.add_argument("--test-laps", type=int, default=20,
                    help="number of final laps held out for testing")
    ap.add_argument("--dist-max", type=float, default=2.0,
                    help="neighbor radius / kernel bandwidth (scaled units); "
                         "with k-max active this mostly acts as k-NN + kernel")
    ap.add_argument("--k-max", type=int, default=256)
    ap.add_argument("--raw-distance", action="store_true",
                    help="no feature scaling (Racing-LMPC-ROS2 behavior)")
    args = ap.parse_args()

    path = args.steps_csv
    if path is None:
        cands = sorted(glob.glob(os.path.join(HERE, "data", "*", "steps_*.csv")))
        if not cands:
            sys.exit("no steps CSV under data/")
        path = cands[-1]
    print(f"data: {path}")

    with open(PARAMS_YAML) as f:
        params = yaml.safe_load(f)

    X, U, Xn, lap = load_pairs(path)
    laps = np.unique(lap)
    split = laps[-args.test_laps]
    tr, te = lap < split, lap >= split
    print(f"{len(X)} pairs | train: laps {laps[0]}..{split-1} ({tr.sum()}) | "
          f"test: laps {split}..{laps[-1]} ({te.sum()})")

    nominal = KinematicBicycleNominal(params["l_cg2front"],
                                      params["l_cg2rear"], TS)
    known = KnownSingleTrack(params, TS)

    # Rolling protocol (mirrors online use): predict each test lap L with a
    # regression buffer holding ALL laps < L — in particular the immediately
    # previous lap, whose speed regime is closest. A static past/future split
    # would force extrapolation, since LMPC makes every lap faster than all
    # before it.
    reg = ErrorDynamicsRegressor(nominal, dist_max=args.dist_max,
                                 k_max=args.k_max,
                                 scale=None if args.raw_distance else "std")
    reg.add_samples(X[tr], U[tr], Xn[tr])
    t0 = time.time()
    pred_res = np.empty((te.sum(), 3))
    used = np.empty(te.sum(), bool)
    pos = 0
    for L in laps[laps >= split]:
        m = lap == L
        n = m.sum()
        pred_res[pos:pos + n], used[pos:pos + n] = reg.predict(X[m], U[m])
        reg.add_samples(X[m], U[m], Xn[m])   # lap L joins the buffer for L+1
        pos += n
    dt_reg = time.time() - t0
    errors = {
        "nominal (kinematic)": nominal.predict_velocities_stacked(X[te], U[te]) - Xn[te],
        "nominal+residual": pred_res - Xn[te],
        "known ST model": known.predict_velocities_stacked(X[te], U[te]) - Xn[te],
    }
    print(f"regression (rolling by lap): {used.mean()*100:.1f}% of queries had "
          f">= min_pts neighbors (fallback = nominal elsewhere); {dt_reg:.1f}s "
          f"for {te.sum()} queries")

    out = table(errors)
    piv = out.pivot(index="model", columns="state", values="RMSE").reindex(
        ["nominal (kinematic)", "nominal+residual", "known ST model"])[STATES]
    pd.set_option("display.float_format", lambda v: f"{v:10.6f}")
    print("\nRMSE (one-step, test laps):")
    print(piv.to_string())
    print("\nMSE:")
    print(out.pivot(index="model", columns="state", values="MSE").reindex(
        piv.index).to_string())

    dataset = os.path.basename(os.path.dirname(path))
    res_dir = os.path.join(HERE, "results", "offline_eval")
    os.makedirs(res_dir, exist_ok=True)
    res_csv = os.path.join(res_dir, f"offline_eval_{dataset}.csv")
    out.to_csv(res_csv, index=False)

    # ---- plot: RMSE bars + error trace on one test lap ----
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    ax = axes[0]
    xs = np.arange(3)
    for j, name in enumerate(piv.index):
        ax.bar(xs + (j - 1) * 0.25, piv.loc[name].to_numpy(), 0.25, label=name)
    ax.set_xticks(xs, STATES)
    ax.set_yscale("log")
    ax.set_ylabel("RMSE (log)")
    ax.set_title(f"one-step prediction RMSE — {dataset}, "
                 f"h={args.dist_max}, k<={args.k_max}")
    ax.legend(fontsize=9)

    ax = axes[1]
    last = lap[te] == laps[-1]
    t = np.arange(last.sum()) * TS
    ax.plot(t, errors["nominal (kinematic)"][last][:, 1], color="#c44",
            lw=0.9, label="nominal v_y error")
    ax.plot(t, errors["nominal+residual"][last][:, 1], color="#2a78d6",
            lw=0.9, label="nominal+residual v_y error")
    ax.set_xlabel(f"time in final test lap [s]")
    ax.set_ylabel("v_y error [m/s]")
    ax.legend(fontsize=9)
    for a in axes:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    png = os.path.join(res_dir, f"offline_eval_{dataset}.png")
    fig.savefig(png, dpi=140)
    print(f"\nresults: {res_csv}\nplot: {png}")


if __name__ == "__main__":
    main()
