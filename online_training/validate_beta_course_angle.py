"""Validate PF-only slip-angle estimation against simulator ground truth.

Question: on the real car we only get (x, y, psi) from particle-filter
localization — no direct body velocities. Can we recover beta (and v_x, v_y)
from the pose track alone via the course-angle method:

    course = atan2(y_dot, x_dot)          (global velocity direction)
    beta   = course - psi
    v_x    =  x_dot*cos(psi) + y_dot*sin(psi)
    v_y    = -x_dot*sin(psi) + y_dot*cos(psi)

Setup: run lmpc_gym.py (which now logs measured meas_vx/meas_vy/meas_yawdot
from the gym plant), then feed ONLY the pose columns (x, y, yaw) through the
estimators below — optionally corrupted with PF-realistic noise — and score
against the logged ground truth.

Estimators:
  raw       central difference, no smoothing (worst case / sanity floor)
  sg        zero-phase Savitzky-Golay derivative (ACAUSAL — valid for offline
            training targets, the residual-learning use case)
  causal    trailing-window polynomial fit, derivative at window end (CAUSAL —
            what an online estimator without IMU/wheel-speed fusion could do;
            shows the lag penalty)

Usage:
    python validate_beta_course_angle.py data/barc_oval_beta_val/steps_*.csv
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DT = 0.05  # control/logging period (20 Hz), = LMPC Ts

# PF noise scenarios: (label, sigma_xy [m], sigma_psi [rad])
# 1 cm / 0.6 deg is a typical lidar-PF figure on f1tenth; 2 cm / 1.1 deg is
# pessimistic.
SCENARIOS = [
    ("clean",           0.000, 0.000),
    ("pf 5mm/0.3deg",   0.005, 0.005),
    ("pf 1cm/0.6deg",   0.010, 0.010),
    ("pf 2cm/1.1deg",   0.020, 0.020),
]

SG_WINDOW = 21       # samples (1.05 s at 20 Hz), zero-phase
SG_POLY = 3
CAUSAL_WINDOW = 13   # trailing samples (0.65 s)
CAUSAL_POLY = 2
V_GATE = 1.0         # m/s: exclude near-standstill from metrics


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def body_frame(xd, yd, psi):
    vx = xd * np.cos(psi) + yd * np.sin(psi)
    vy = -xd * np.sin(psi) + yd * np.cos(psi)
    return vx, vy


def est_raw(x, y, psi_u):
    """Central difference, raw noisy heading."""
    xd = np.gradient(x, DT)
    yd = np.gradient(y, DT)
    vx, vy = body_frame(xd, yd, psi_u)
    omega = np.gradient(psi_u, DT)
    return vx, vy, omega


def est_sg(x, y, psi_u):
    """Zero-phase Savitzky-Golay: derivative of local poly fit (acausal)."""
    xd = savgol_filter(x, SG_WINDOW, SG_POLY, deriv=1, delta=DT)
    yd = savgol_filter(y, SG_WINDOW, SG_POLY, deriv=1, delta=DT)
    psi_s = savgol_filter(psi_u, SG_WINDOW, SG_POLY)
    omega = savgol_filter(psi_u, SG_WINDOW, SG_POLY, deriv=1, delta=DT)
    vx, vy = body_frame(xd, yd, psi_s)
    return vx, vy, omega


def est_causal(x, y, psi_u):
    """Trailing-window polyfit, derivative evaluated at the newest sample.
    Causal: uses only past data. Vectorized via per-window least squares."""
    n = len(x)
    w = CAUSAL_WINDOW
    t = (np.arange(w) - (w - 1)) * DT          # window times, 0 at newest
    V = np.vander(t, CAUSAL_POLY + 1)          # [t^2, t, 1]
    P = np.linalg.pinv(V)                      # (poly+1, w)
    d_row = P[-2]                              # coefficient of t -> deriv at t=0
    s_row = P[-1]                              # constant term  -> value at t=0

    def roll(sig, row):
        out = np.full(n, np.nan)
        sw = np.lib.stride_tricks.sliding_window_view(sig, w)
        out[w - 1:] = sw @ row
        return out

    xd, yd = roll(x, d_row), roll(y, d_row)
    psi_s = roll(psi_u, s_row)
    omega = roll(psi_u, d_row)
    # fill the startup edge with raw values so shapes match
    bad = np.isnan(xd)
    xd[bad] = np.gradient(x, DT)[bad]
    yd[bad] = np.gradient(y, DT)[bad]
    psi_s[bad] = psi_u[bad]
    omega[bad] = np.gradient(psi_u, DT)[bad]
    vx, vy = body_frame(xd, yd, psi_s)
    return vx, vy, omega


ESTIMATORS = [("raw", est_raw), ("sg (acausal)", est_sg),
              ("causal", est_causal)]


def metrics(est, gt, mask):
    e = (est - gt)[mask]
    return np.sqrt(np.mean(e ** 2)), np.mean(np.abs(e)), np.max(np.abs(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("steps_csv", nargs="?", default=None,
                    help="steps CSV from lmpc_gym.py (default: newest in data/)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    path = args.steps_csv
    if path is None:
        cands = sorted(glob.glob(os.path.join(here, "data", "*", "steps_*.csv")))
        if not cands:
            sys.exit("no steps CSV found under data/")
        path = cands[-1]
    print(f"data: {path}")

    df = pd.read_csv(path)
    need = {"meas_vx", "meas_vy", "meas_yawdot"}
    if not need.issubset(df.columns):
        sys.exit("CSV lacks measured-state columns — rerun lmpc_gym.py "
                 "(logger was extended to record meas_vx/meas_vy/meas_yawdot)")

    x0, y0 = df["x"].to_numpy(), df["y"].to_numpy()
    psi0 = df["yaw"].to_numpy()
    vx_gt = df["meas_vx"].to_numpy()
    vy_gt = df["meas_vy"].to_numpy()
    om_gt = df["meas_yawdot"].to_numpy()
    beta_gt = np.arctan2(vy_gt, vx_gt)
    v_gt = np.hypot(vx_gt, vy_gt)

    mask = v_gt > V_GATE
    mask[:int(2.0 / DT)] = False  # drop standstill/launch
    print(f"{len(df)} steps, {mask.sum()} scored (v > {V_GATE} m/s)")
    print(f"ground-truth beta: |median| "
          f"{np.rad2deg(np.median(np.abs(beta_gt[mask]))):.2f} deg, "
          f"p95 {np.rad2deg(np.percentile(np.abs(beta_gt[mask]), 95)):.2f} deg, "
          f"max {np.rad2deg(np.max(np.abs(beta_gt[mask]))):.2f} deg")

    rng = np.random.default_rng(args.seed)
    rows = []
    curves = {}  # (scenario, est) -> beta estimate, for plotting
    for sc_name, s_xy, s_psi in SCENARIOS:
        x = x0 + rng.normal(0, s_xy, len(x0)) if s_xy else x0.copy()
        y = y0 + rng.normal(0, s_xy, len(y0)) if s_xy else y0.copy()
        psi = psi0 + rng.normal(0, s_psi, len(psi0)) if s_psi else psi0.copy()
        psi_u = np.unwrap(psi)

        for est_name, fn in ESTIMATORS:
            vx_e, vy_e, om_e = fn(x, y, psi_u)
            beta_e = np.arctan2(vy_e, np.maximum(vx_e, 1e-6))
            b_rmse, b_mae, b_max = metrics(wrap(beta_e - beta_gt),
                                           np.zeros_like(beta_gt), mask)
            vx_rmse, _, _ = metrics(vx_e, vx_gt, mask)
            vy_rmse, _, _ = metrics(vy_e, vy_gt, mask)
            om_rmse, _, _ = metrics(om_e, om_gt, mask)
            rows.append([sc_name, est_name,
                         np.rad2deg(b_rmse), np.rad2deg(b_mae),
                         np.rad2deg(b_max), vx_rmse, vy_rmse, om_rmse])
            curves[(sc_name, est_name)] = beta_e

    dataset = os.path.basename(os.path.dirname(path))
    out = pd.DataFrame(rows, columns=[
        "scenario", "estimator", "beta_RMSE_deg", "beta_MAE_deg",
        "beta_maxerr_deg", "vx_RMSE_mps", "vy_RMSE_mps", "omega_RMSE_radps"])
    pd.set_option("display.float_format", lambda v: f"{v:8.4f}")
    print("\n" + out.to_string(index=False))
    res_dir = os.path.join(here, "results", "beta_validation")
    os.makedirs(res_dir, exist_ok=True)
    res_csv = os.path.join(res_dir, f"beta_validation_{dataset}.csv")
    out.to_csv(res_csv, index=False)
    print(f"\nresults: {res_csv}")

    # ---- speed-binned breakdown: does accuracy hold at high speed? ----
    bins = [(V_GATE, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, np.inf)]
    print("\nbeta RMSE [deg] by speed bin (signal: median |beta_gt| in bin):")
    hdr = "  ".join(f"{f'v {lo:g}-{hi:g}':>14s}" for lo, hi in bins)
    print(f"{'scenario':>16s} {'estimator':>13s}  {hdr}")
    for sc_name, _, _ in SCENARIOS:
        for est_name, _ in ESTIMATORS:
            beta_e = curves[(sc_name, est_name)]
            cells = []
            for lo, hi in bins:
                m = mask & (v_gt >= lo) & (v_gt < hi)
                if m.sum() < 50:
                    cells.append(f"{'—':>14s}")
                    continue
                r = np.rad2deg(np.sqrt(np.mean(wrap(beta_e - beta_gt)[m] ** 2)))
                sig = np.rad2deg(np.median(np.abs(beta_gt[m])))
                cells.append(f"{r:6.2f} ({sig:4.2f})")
            print(f"{sc_name:>16s} {est_name:>13s}  {'  '.join(cells)}")

    # ---- plot: one representative lap window, mid-noise scenario ----
    sc_show = SCENARIOS[2][0]
    idx = np.where(mask)[0]
    seg = slice(idx[len(idx) // 2], min(idx[len(idx) // 2] + 400, idx[-1]))
    t = df["sim_time"].to_numpy()

    fig, axes = plt.subplots(3, 1, figsize=(11, 10))
    axes[0].sharex(axes[1])
    ax = axes[0]
    ax.plot(t[seg], np.rad2deg(beta_gt[seg]), "k-", lw=1.8, label="ground truth")
    for est_name, style in [("raw", dict(color="#c44", alpha=0.45, lw=0.8)),
                            ("sg (acausal)", dict(color="#2a78d6", lw=1.6)),
                            ("causal", dict(color="#2a9d5c", lw=1.2))]:
        ax.plot(t[seg], np.rad2deg(curves[(sc_show, est_name)][seg]),
                label=est_name, **style)
    ax.set_ylabel("beta [deg]")
    ax.set_title(f"slip angle from pose track only — scenario '{sc_show}'")
    ax.legend(ncol=4, fontsize=9)
    ax.set_ylim(np.rad2deg(beta_gt[seg]).min() - 3,
                np.rad2deg(beta_gt[seg]).max() + 3)

    ax = axes[1]
    ax.plot(t[seg], np.rad2deg(beta_gt[seg]), "k-", lw=1.8, label="ground truth")
    ax.plot(t[seg], np.rad2deg(curves[("clean", "sg (acausal)")][seg]),
            color="#2a78d6", lw=1.4, label="sg, clean pose")
    ax.plot(t[seg], np.rad2deg(curves[(SCENARIOS[3][0], "sg (acausal)")][seg]),
            color="#d68a2a", lw=1.2, label=f"sg, {SCENARIOS[3][0]}")
    ax.set_ylabel("beta [deg]")
    ax.set_title("noise sensitivity of the acausal (training-target) estimator")
    ax.legend(ncol=3, fontsize=9)

    ax = axes[2]
    width = 0.25
    xs = np.arange(len(SCENARIOS))
    for j, (est_name, _) in enumerate(ESTIMATORS):
        vals = [out[(out.scenario == s) & (out.estimator == est_name)]
                ["beta_RMSE_deg"].iloc[0] for s, _, _ in SCENARIOS]
        ax.bar(xs + (j - 1) * width, vals, width, label=est_name)
    med = np.rad2deg(np.median(np.abs(beta_gt[mask])))
    ax.axhline(med, color="k", ls="--", lw=1,
               label=f"|beta| median = {med:.2f} deg")
    ax.set_xticks(xs, [s for s, _, _ in SCENARIOS])
    ax.set_ylabel("beta RMSE [deg]")
    ax.set_title("estimation error vs PF noise level")
    ax.legend(fontsize=9)

    for a in axes:
        a.grid(True, alpha=0.3)
    axes[1].set_xlabel("sim time [s]")
    axes[2].set_xlabel("noise scenario")
    fig.tight_layout()
    png = os.path.join(res_dir, f"beta_validation_{dataset}.png")
    fig.savefig(png, dpi=140)
    print(f"plot: {png}")


if __name__ == "__main__":
    main()
