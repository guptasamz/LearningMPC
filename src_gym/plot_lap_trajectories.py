"""Plot track boundaries + every lap's trajectory, color-coded by lap time.

Boundaries come from the f1tenth_racetracks centerline csv (x, y, w_right,
w_left): edge = centerline +- width along the local normal. Trajectories come
from the run's steps_*.csv grouped by lap; color follows that lap's time on a
sequential single-hue ramp (light = slow early laps, dark = fast late laps),
with a colorbar as the lap-time legend.

Usage:
    ../.venv/bin/python plot_lap_trajectories.py --map barc_oval_orginal \
        --run results/barc_orig_v3
"""

import argparse
import csv
import glob
import os

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
INK, MUTED, SURFACE, GRID = "#1a1a19", "#6b6a63", "#fcfcfb", "#e4e3dc"


def load_centerline_with_widths(map_name):
    path = os.path.join(REPO, "data", "maps", map_name, f"{map_name}_centerline.csv")
    rows = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            rows.append([float(v) for v in row[:4]])
    return np.array(rows)  # x, y, w_right, w_left


def track_boundaries(cl):
    """Left/right edges: centerline offset along the local normal."""
    pts = cl[:, :2]
    closed = np.vstack([pts, pts[:1]])
    tang = np.diff(closed, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normal = np.column_stack([-tang[:, 1], tang[:, 0]])  # left of travel
    left = pts + normal * cl[:, 3:4]
    right = pts - normal * cl[:, 2:3]
    return np.vstack([left, left[:1]]), np.vstack([right, right[:1]])


def make_plot(map_name, run_dir, out=None):
    """Render boundaries + lap trajectories for a finished run. Returns the
    output path, or None if the map has no width-annotated centerline
    (e.g. levinelobby) or the run produced no laps."""
    step_files = sorted(glob.glob(os.path.join(run_dir, "steps_*.csv")))
    lap_files = sorted(glob.glob(os.path.join(run_dir, "laps_*.csv")))
    if not step_files or not lap_files:
        return None
    steps = pd.read_csv(step_files[-1])
    laps = pd.read_csv(lap_files[-1])
    if laps.empty:
        return None
    lap_time = dict(zip(laps["lap_index(iter)"], laps["lap_time_s"]))

    cl = load_centerline_with_widths(map_name)
    left, right = track_boundaries(cl)

    # single-hue sequential ramp; dark = fast (low lap time)
    ramp = LinearSegmentedColormap.from_list(
        "blues_trunc", plt.get_cmap("Blues")(np.linspace(0.25, 1.0, 256)))
    tmin, tmax = laps["lap_time_s"].min(), laps["lap_time_s"].max()
    norm = Normalize(tmin, tmax)
    color_of = lambda t: ramp(1.0 - norm(t))  # fast -> dark end

    fig, ax = plt.subplots(figsize=(10, 9), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.plot(left[:, 0], left[:, 1], color=INK, lw=1.6, zorder=5, label="track boundary")
    ax.plot(right[:, 0], right[:, 1], color=INK, lw=1.6, zorder=5)
    center_closed = np.vstack([cl[:, :2], cl[:1, :2]])
    ax.plot(center_closed[:, 0], center_closed[:, 1], color=MUTED, lw=1.2,
            ls=(0, (5, 4)), zorder=2, label="centerline")

    for it, t in sorted(lap_time.items()):          # slow (light) under fast (dark)
        d = steps[steps["iter"] == it]
        ax.plot(d["x"], d["y"], color=color_of(t), lw=1.1, alpha=0.85, zorder=3)

    sm = ScalarMappable(cmap=ramp.reversed(), norm=norm)  # colorbar: dark at low times
    cbar = fig.colorbar(sm, ax=ax, fraction=0.045, pad=0.02)
    cbar.set_label("lap time [s]  (dark = fast)", color=MUTED, fontsize=10)
    cbar.ax.tick_params(colors=MUTED, labelsize=9)
    cbar.outline.set_edgecolor("#d8d7d0")

    ax.set_aspect("equal")
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for s in ax.spines.values():
        s.set_color("#d8d7d0")
    ax.set_xlabel("x [m]", color=MUTED)
    ax.set_ylabel("y [m]", color=MUTED)
    ax.legend(loc="upper right", frameon=True, facecolor=SURFACE,
              edgecolor="#d8d7d0", fontsize=9, labelcolor=INK)
    n = len(lap_time)
    ax.set_title(f"{map_name} — {n} laps, {tmax:.2f} s → {tmin:.2f} s",
                 color=INK, fontsize=12, pad=10)
    fig.tight_layout()

    out = out or os.path.join(run_dir, "lap_trajectories.png")
    fig.savefig(out, dpi=140, facecolor=SURFACE)
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--run", required=True, help="results dir of the run")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = make_plot(args.map, args.run, args.out)
    print(f"saved {out}" if out else "no plot (missing data or widths)")


if __name__ == "__main__":
    main()
