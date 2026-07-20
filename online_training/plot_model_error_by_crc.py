"""One-step model-error comparison across CRC values (mu fixed).

For each CRC cell of one (map, mu) folder: per-lap mean of the LMPC model's
one-step prediction error, from the steps CSV audit columns
(pred_* = QP prediction of the next state, meas_*/v = what the plant did).

    e_v     = |pred_v      - v_{k+1}|          [m/s]
    e_omega = |pred_yawdot - meas_yawdot_{k+1}| [rad/s]

Claim being tested: the residual model is accurate inside the visited data
envelope at every CRC; low CRC crashes because the speed ratchet queries the
model OUTSIDE that envelope — visible as an error blow-up in the final laps
of low-CRC runs, absent in high-CRC runs.

Usage:
    python plot_model_error_by_crc.py [--cell .../CRC_experiments/levinelobby_track/levinelobby_track_1.0]
"""

import argparse
import glob
import os
import re

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib import cm

HERE = os.path.dirname(os.path.abspath(__file__))
CRCS = ["2.0", "1.5", "1.0", "0.5", "0.1", "0.05", "0.01"]
CRASH_RED = "#c0392b"
V_GATE = 1.0


def per_lap_error(steps_csv):
    df = pd.read_csv(steps_csv)
    v_next = df["v"].values[1:]
    om_next = df["meas_yawdot"].values[1:]
    lap = df["iter"].values[:-1]
    e_v = np.abs(df["pred_v"].values[:-1] - v_next)
    e_om = np.abs(df["pred_yawdot"].values[:-1] - om_next)
    ok = (df["v"].values[:-1] > V_GATE) & (v_next > V_GATE)
    g = pd.DataFrame({"lap": lap[ok] - 2, "e_v": e_v[ok],
                      "e_om": e_om[ok]}).groupby("lap").mean()
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", default=os.path.join(
        HERE, "..", "src_gym", "results_residual_dynmics", "CRC_experiments",
        "levinelobby_track", "levinelobby_track_1.0"))
    args = ap.parse_args()
    name = os.path.basename(args.cell)

    cmap = cm.get_cmap("Oranges")
    crc_color = {c: cmap(0.90 - 0.62 * i / (len(CRCS) - 1))
                 for i, c in enumerate(CRCS)}

    fig, axes = plt.subplots(2, 1, figsize=(11, 8.5), sharex=True)
    drawn = []
    for crc in CRCS:
        d = os.path.join(args.cell, f"{name}_{crc}")
        csvs = sorted(glob.glob(os.path.join(d, "steps_*.csv")))
        log = os.path.join(d, "run.log")
        if not csvs or not os.path.exists(log):
            continue
        crashed = bool(re.search(r"crashed: True", open(log).read()))
        g = per_lap_error(csvs[-1])
        drawn.append(crc)
        for ax, col in zip(axes, ["e_v", "e_om"]):
            ax.plot(g.index, g[col], color=crc_color[crc], lw=1.8)
            if crashed:
                ax.plot(g.index[-1], g[col].iloc[-1], "x", color=CRASH_RED,
                        ms=9, mew=2.2, zorder=5)

    axes[0].set_ylabel("per-lap mean |pred v − actual v| [m/s]")
    axes[1].set_ylabel("per-lap mean |pred ω − actual ω| [rad/s]")
    axes[1].set_xlabel("lap")
    for ax in axes:
        ax.set_yscale("log")
        ax.grid(True, alpha=0.25, which="both")
    handles = [Line2D([], [], color=crc_color[c], lw=2.2, label=f"CRC = {c}")
               for c in drawn]
    handles.append(Line2D([], [], marker="x", ls="none", color=CRASH_RED,
                          ms=9, mew=2.2, label="crash"))
    axes[0].legend(handles=handles, fontsize=9, ncol=2, framealpha=0.9)
    axes[0].set_title(f"{name} — one-step model error per lap, by CRC "
                      "(log scale)")
    fig.tight_layout()
    out = os.path.join(args.cell, "model_error_by_crc.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"saved {out}")

    # numeric summary: mid-run plateau vs final-3-laps error
    print(f"\n{'CRC':>6s} {'e_v mid':>10s} {'e_v last3':>10s} "
          f"{'e_om mid':>10s} {'e_om last3':>11s}")
    for crc in drawn:
        d = os.path.join(args.cell, f"{name}_{crc}")
        g = per_lap_error(sorted(glob.glob(os.path.join(d, "steps_*.csv")))[-1])
        n = len(g)
        mid = g.iloc[n // 3: 2 * n // 3]
        last = g.iloc[-3:]
        print(f"{crc:>6s} {mid.e_v.mean():10.4f} {last.e_v.mean():10.4f} "
              f"{mid.e_om.mean():10.4f} {last.e_om.mean():11.4f}")


if __name__ == "__main__":
    main()
