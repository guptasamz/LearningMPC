"""Within-lap prediction error on the crash lap, by CRC.

For each crashed CRC cell (mu fixed): instantaneous one-step model error
|pred - actual| at every control step of its CRASH lap, plotted against
track position s (so corners align across runs), overlaid with:
  * the surviving CRC runs on the SAME lap number (solid, orange ramp)
  * the crashed run's PREVIOUS lap (dashed) - separates "error was already
    growing lap-over-lap" from "instantaneous blow-up within the lap"
  * vertical line = crash position.

Usage:
    python plot_crashlap_error.py [--cell .../levinelobby_track_1.0]
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


def load(cell, name, crc):
    d = os.path.join(cell, f"{name}_{crc}")
    csvs = sorted(glob.glob(os.path.join(d, "steps_*.csv")))
    log = os.path.join(d, "run.log")
    if not csvs or not os.path.exists(log):
        return None, None
    df = pd.read_csv(csvs[-1])
    crashed = bool(re.search(r"crashed: True", open(log).read()))
    # instantaneous one-step errors, attached to row k
    df = df.iloc[:-1].assign(
        e_v=np.abs(df["pred_v"].values[:-1] - df["v"].values[1:]),
        e_om=np.abs(df["pred_yawdot"].values[:-1]
                    - df["meas_yawdot"].values[1:]),
        lap=df["iter"].values[:-1] - 2)
    return df, crashed


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

    runs = {c: load(args.cell, name, c) for c in CRCS}
    runs = {c: v for c, v in runs.items() if v[0] is not None}
    crashed_crcs = [c for c, (_, cr) in runs.items() if cr]
    survivors = [c for c, (_, cr) in runs.items() if not cr]

    nrows = len(crashed_crcs)
    fig, axes = plt.subplots(nrows, 2, figsize=(13, 3.4 * nrows),
                             squeeze=False)
    for r, crc in enumerate(crashed_crcs):
        df, _ = runs[crc]
        L = int(df["lap"].max())            # crash lap (in progress at crash)
        s_crash = df["s"].values[-1]
        for col, (ch, ylab) in enumerate([
                ("e_v", "|pred v − v| [m/s]"),
                ("e_om", "|pred ω − ω| [rad/s]")]):
            ax = axes[r][col]
            for sc in survivors:            # same lap number, survivors
                sdf, _ = runs[sc][0], None
                sl = runs[sc][0][runs[sc][0]["lap"] == L]
                if len(sl):
                    ax.plot(sl["s"], sl[ch], color=crc_color[sc], lw=1.2)
            prev = df[df["lap"] == L - 1]   # crashed run, previous lap
            ax.plot(prev["s"], prev[ch], color=CRASH_RED, lw=1.0, ls="--",
                    alpha=0.55)
            cur = df[df["lap"] == L]        # crashed run, crash lap
            ax.plot(cur["s"], cur[ch], color=CRASH_RED, lw=1.8)
            ax.axvline(s_crash, color=CRASH_RED, lw=1.0, ls=":", alpha=0.8)
            ax.set_yscale("log")
            ax.grid(True, alpha=0.25, which="both")
            ax.set_ylabel(ylab, fontsize=9)
            if col == 0:
                ax.set_title(f"CRC = {crc} — crash lap {L} "
                             f"(crash at s = {s_crash:.1f} m)",
                             fontsize=10, loc="left")
        axes[r][1].set_title(f"survivors on lap {L}: "
                             + ", ".join(survivors), fontsize=9, loc="right")
    for ax in axes[-1]:
        ax.set_xlabel("track position s [m]")
    handles = ([Line2D([], [], color=CRASH_RED, lw=1.8, label="crashed CRC, crash lap"),
                Line2D([], [], color=CRASH_RED, lw=1.0, ls="--", alpha=0.55,
                       label="crashed CRC, previous lap"),
                Line2D([], [], color=CRASH_RED, lw=1.0, ls=":",
                       label="crash position")]
               + [Line2D([], [], color=crc_color[c], lw=1.6,
                         label=f"CRC {c} (survived)") for c in survivors])
    fig.legend(handles=handles, loc="upper center", ncol=4, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle(f"{name} — instantaneous model error along the crash lap",
                 y=1.05, fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = os.path.join(args.cell, "model_error_crashlap_by_crc.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
