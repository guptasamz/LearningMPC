"""Lap-time curves of the CRC (control-rate cost) experiment.

One figure per (map, mu) folder under
results_residual_dynmics/CRC_experiments/<map>/<map>_<mu>/ — one line per CRC
value (sequential ramp: dark = strong damping), red x on the last completed
lap of crashed runs, no point markers. Saved as laptime_by_crc.png inside the
same folder.

Usage:
    python plot_crc_curves.py [--base ../src_gym/results_residual_dynmics/CRC_experiments]
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
CRCS = ["2.0", "1.5", "1.0", "0.5", "0.1", "0.05", "0.01"]  # dark -> light
CRASH_RED = "#c0392b"


def load_cell(d):
    laps = sorted(glob.glob(os.path.join(d, "laps_*.csv")))
    log = os.path.join(d, "run.log")
    if not laps or not os.path.exists(log):
        return None, None
    df = pd.read_csv(laps[-1])
    crashed = bool(re.search(r"crashed: True", open(log).read()))
    return df, crashed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.path.join(
        HERE, "..", "src_gym", "results_residual_dynmics", "CRC_experiments"))
    args = ap.parse_args()

    cmap = cm.get_cmap("Oranges")
    # rank-spaced steps, darkest for the largest CRC
    crc_color = {c: cmap(0.90 - 0.62 * i / (len(CRCS) - 1))
                 for i, c in enumerate(CRCS)}

    made = 0
    for mu_dir in sorted(glob.glob(os.path.join(args.base, "*", "*_0.*"))) + \
                  sorted(glob.glob(os.path.join(args.base, "*", "*_1.0"))):
        cell_name = os.path.basename(mu_dir)          # <map>_<mu>
        cells = {}
        for crc in CRCS:
            df, crashed = load_cell(os.path.join(mu_dir, f"{cell_name}_{crc}"))
            if df is not None and len(df):
                cells[crc] = (df, crashed)
        if not cells:
            continue

        fig, ax = plt.subplots(figsize=(10, 6))
        ends = []
        for crc, (df, crashed) in cells.items():
            x = (df["lap_index(iter)"] - 2).values
            y = df["lap_time_s"].values
            ax.plot(x, y, color=crc_color[crc], lw=1.8)
            if crashed:
                ax.plot(x[-1], y[-1], "x", color=CRASH_RED, ms=9, mew=2.2,
                        zorder=5)
            ends.append((x[-1], y[-1], crc))

        # staggered end-of-line labels with leaders (same recipe as the
        # per-mu campaign plots)
        y0, y1 = ax.get_ylim()
        gap = (y1 - y0) * 0.035
        x_range = max(e[0] for e in ends) or 1.0
        ax.set_xlim(right=x_range * 1.12)
        dx = 0.014 * x_range
        placed = []
        for xe, ye, crc in sorted(ends, key=lambda e: (round(e[0], -1), e[1])):
            ly = ye
            while any(abs(xe - px) < 0.08 * x_range and abs(ly - py) < gap
                      for px, py in placed):
                ly += gap
            placed.append((xe, ly))
            if abs(ly - ye) > 1e-9:
                ax.plot([xe + 0.2 * dx, xe + 0.9 * dx], [ye, ly],
                        color="#bbbbbb", lw=0.6, zorder=4)
            ax.text(xe + dx, ly, crc, fontsize=8.5, color="#333333",
                    ha="left", va="center", zorder=6)

        handles = [Line2D([], [], color=crc_color[c], lw=2.2,
                          label=f"CRC = {c}") for c in CRCS if c in cells]
        handles.append(Line2D([], [], marker="x", ls="none", color=CRASH_RED,
                              ms=9, mew=2.2, label="crashed after this lap"))
        ax.legend(handles=handles, fontsize=9, ncol=2, framealpha=0.9)
        ax.set_xlabel("lap")
        ax.set_ylabel("lap time [s]")
        ax.grid(True, alpha=0.25)
        ax.set_title(f"{cell_name} — lap time by control-rate cost (CRC)")
        fig.tight_layout()
        out = os.path.join(mu_dir, "laptime_by_crc.png")
        fig.savefig(out, dpi=140, bbox_inches="tight")
        plt.close(fig)
        made += 1
        print(f"saved {out}")
    print(f"{made} figures")


if __name__ == "__main__":
    main()
