"""Crash investigation: how wrong was the LMPC's model, when, and where?

Compares the QP's one-control-step-ahead state prediction (logged as pred_*)
against what the simulator actually did 0.05 s later. Produces:

  results/pred_error_timeseries.png  -- per-step prediction error vs sim time
  results/pred_error_per_lap.png     -- per-lap RMSE (position / velocity)
  results/crash_trajectory.png       -- laps 27-29 paths on the map, crash marked

Usage:  ../.venv/bin/python analyze_crash.py [steps_csv]
"""

import glob
import math
import os
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BLUE, ORANGE, INK, MUTED, SURFACE = "#2a78d6", "#eb6834", "#1a1a19", "#6b6a63", "#fcfcfb"
GRID = "#e4e3dc"


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for s in ax.spines.values():
        s.set_color("#d8d7d0")


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main():
    steps_csv = sys.argv[1] if len(sys.argv) > 1 else sorted(
        glob.glob(os.path.join(HERE, "results", "steps_*.csv")))[-1]
    df = pd.read_csv(steps_csv)
    print(f"analyzing {steps_csv}: {len(df)} control steps, "
          f"laps {df['iter'].min()}..{df['iter'].max()}, "
          f"solver failures: {(df['solved'] == 0).sum()}")

    # align: prediction in row i is for the state measured in row i+1
    cur = df.iloc[:-1].reset_index(drop=True)
    nxt = df.iloc[1:].reset_index(drop=True)
    err_pos = np.hypot(nxt["x"] - cur["pred_x"], nxt["y"] - cur["pred_y"])
    err_v = (nxt["v"] - cur["pred_v"]).abs()
    err_yaw = np.abs(wrap(nxt["yaw"].values - cur["pred_yaw"].values))
    t = nxt["sim_time"].values
    lap = cur["iter"].values          # lap the prediction was made in
    crash_t = df["sim_time"].iloc[-1]
    crash_lap = int(df["iter"].iloc[-1])

    # ---------- per-step timeseries ----------
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, facecolor=SURFACE)
    series = [(err_pos, "position prediction error [m]"),
              (err_v, "speed prediction error [m/s]"),
              (err_yaw, "yaw prediction error [rad]")]
    lap_starts = [(int(l), t[np.argmax(lap == l)]) for l in np.unique(lap)]
    for ax, (e, label) in zip(axes, series):
        style(ax)
        in29 = lap == crash_lap
        ax.plot(t[~in29], e[~in29], ".", color=BLUE, ms=2.2, alpha=0.55)
        ax.plot(t[in29], e[in29], ".", color=ORANGE, ms=3.2)
        ax.axvline(crash_t, color=INK, lw=1, ls="--")
        ax.set_ylabel(label, color=MUTED, fontsize=9)
        for l, ts in lap_starts:
            ax.axvline(ts, color=GRID, lw=0.6, zorder=0)
    axes[0].set_title(
        f"LMPC one-step (0.05 s) model prediction error — blue: laps 2–{crash_lap-1}, "
        f"orange: lap {crash_lap} (crash), dashed: impact", color=INK, fontsize=11)
    axes[0].annotate("crash", xy=(crash_t, axes[0].get_ylim()[1] * 0.9),
                     xytext=(-40, 0), textcoords="offset points", color=INK, fontsize=9)
    axes[2].set_xlabel("sim time [s]", color=MUTED)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "results", "pred_error_timeseries.png"),
                dpi=140, facecolor=SURFACE)

    # ---------- per-lap RMSE ----------
    laps_u = np.unique(lap)
    rmse_pos = [float(np.sqrt(np.mean(err_pos[lap == l] ** 2))) for l in laps_u]
    rmse_v = [float(np.sqrt(np.mean(err_v[lap == l] ** 2))) for l in laps_u]
    tbl = pd.DataFrame({"lap": laps_u, "rmse_pos_m": rmse_pos, "rmse_v_mps": rmse_v})
    print(tbl.to_string(index=False, float_format=lambda x: f"{x:.4f}"))
    print(f"highest position-RMSE lap: {laps_u[int(np.argmax(rmse_pos))]}, "
          f"highest speed-RMSE lap: {laps_u[int(np.argmax(rmse_v))]}")

    fig2, ax2 = plt.subplots(1, 2, figsize=(11, 4.2), facecolor=SURFACE)
    for ax, vals, label in [(ax2[0], rmse_pos, "position RMSE per lap [m]"),
                            (ax2[1], rmse_v, "speed RMSE per lap [m/s]")]:
        style(ax)
        ax.plot(laps_u, vals, color=BLUE, lw=2, marker="o", ms=5, mec="white")
        k = np.where(laps_u == crash_lap)[0]
        if len(k):
            ax.plot(crash_lap, vals[int(k)], marker="o", ms=9, color=ORANGE,
                    mec="white", mew=1)
            ax.annotate("crash lap", (crash_lap, vals[int(k)]),
                        xytext=(-10, 12), textcoords="offset points",
                        color=ORANGE, fontsize=9, ha="right")
        ax.set_xlabel("lap (LMPC iteration)", color=MUTED)
        ax.set_title(label, color=INK, fontsize=10)
    fig2.tight_layout()
    fig2.savefig(os.path.join(HERE, "results", "pred_error_per_lap.png"),
                 dpi=140, facecolor=SURFACE)

    # ---------- crash-zone trajectory ----------
    stem = os.path.join(HERE, "..", "data", "levinelobby_track")
    with open(stem + ".yaml") as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(stem + ".png"))
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    h, w = img.shape
    ext = [ox, ox + w * res, oy, oy + h * res]

    fig3, ax3 = plt.subplots(figsize=(9, 9), facecolor=SURFACE)
    style(ax3)
    disp = np.full(img.shape, 253.0)
    disp[img < 100] = 178
    ax3.imshow(disp, cmap="gray", vmin=0, vmax=255, extent=ext, origin="upper")
    shades = ["#9dc3ec", BLUE]
    for i, l in enumerate([crash_lap - 2, crash_lap - 1]):
        d = df[df["iter"] == l]
        ax3.plot(d["x"], d["y"], color=shades[i], lw=1.6, label=f"lap {l}")
    d29 = df[df["iter"] == crash_lap]
    ax3.plot(d29["x"], d29["y"], color=ORANGE, lw=2.2, label=f"lap {crash_lap} (crash)")
    ax3.plot(df["x"].iloc[-1], df["y"].iloc[-1], marker="*", ms=18, color=INK,
             mec="white", mew=1, ls="none", label="impact")
    ax3.legend(loc="lower left", frameon=True, facecolor=SURFACE,
               edgecolor="#d8d7d0", fontsize=9, labelcolor=INK)
    pad = 2.2
    ax3.set_xlim(df["x"].iloc[-1] - pad, df["x"].iloc[-1] + pad)
    ax3.set_ylim(df["y"].iloc[-1] - pad, df["y"].iloc[-1] + pad)
    ax3.set_title("crash zone — final laps' paths", color=INK, fontsize=11)
    ax3.set_xlabel("x [m]", color=MUTED); ax3.set_ylabel("y [m]", color=MUTED)
    fig3.tight_layout()
    fig3.savefig(os.path.join(HERE, "results", "crash_trajectory.png"),
                 dpi=140, facecolor=SURFACE)
    print("saved 3 figures to results/")


if __name__ == "__main__":
    main()
