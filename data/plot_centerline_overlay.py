"""Overlay the centerline and initial safe set on the Levine lobby track map.

Reads (from this script's directory):
    centerline_waypoints.csv   -- x,y waypoints of the track centerline
    initial_safe_set.csv       -- recorded path-follower laps
                                  (cols: time,x,y,yaw,vel,accel_cmd,steer_cmd,s)
    levinelobby_track.png      -- occupancy grid image (map_server format)
    levinelobby_track.yaml     -- map metadata (resolution, origin)

Writes:
    ../media/centerline_on_levinelobby_track.png

Usage:
    python3 plot_centerline_overlay.py
"""

import csv
import os
import re

import numpy as np
from PIL import Image
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_PNG = os.path.join(HERE, "levinelobby_track.png")
MAP_YAML = os.path.join(HERE, "levinelobby_track.yaml")
WAYPOINTS_CSV = os.path.join(HERE, "centerline_waypoints.csv")
SAFE_SET_CSV = os.path.join(HERE, "initial_safe_set.csv")
OUT_PNG = os.path.join(HERE, "..", "media", "centerline_on_levinelobby_track.png")

BLUE = "#2a78d6"   # centerline
INK = "#1a1a19"    # text / markers
MUTED = "#6b6a63"  # axes
SURFACE = "#fcfcfb"


def load_map_metadata(yaml_path):
    """Parse resolution and origin from a map_server yaml (no yaml dep needed)."""
    text = open(yaml_path).read()
    res = float(re.search(r"resolution:\s*([0-9.]+)", text).group(1))
    org = re.search(r"origin:\s*\[([^\]]+)\]", text).group(1)
    ox, oy = [float(v) for v in org.split(",")[:2]]
    return res, ox, oy


def main():
    res, ox, oy = load_map_metadata(MAP_YAML)
    img = np.array(Image.open(MAP_PNG).convert("L"))
    h, w = img.shape

    wp = np.array(
        [
            [float(r[0]), float(r[1])]
            for r in csv.reader(open(WAYPOINTS_CSV))
            if len(r) >= 2
        ]
    )
    wp_closed = np.vstack([wp, wp[:1]])  # close the loop

    # initial safe set: time,x,y,yaw,vel,accel_cmd,steer_cmd,s
    ss = np.loadtxt(SAFE_SET_CSV, delimiter=",")
    ss_xy, ss_vel = ss[:, 1:3], ss[:, 4]

    # crop the map to the track bounding box plus padding
    pad = 2.0
    x0, x1 = wp[:, 0].min() - pad, wp[:, 0].max() + pad
    y0, y1 = wp[:, 1].min() - pad, wp[:, 1].max() + pad
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
    r1, r0 = int(h - (y0 - oy) / res), int(h - (y1 - oy) / res)
    c0, c1 = max(c0, 0), min(c1, w)
    r0, r1 = max(r0, 0), min(r1, h)
    crop = img[r0:r1, c0:c1]
    extent = [ox + c0 * res, ox + c1 * res, oy + (h - r1) * res, oy + (h - r0) * res]

    # recolor for display: free -> near-white, walls -> light gray
    disp = np.full(crop.shape, 253, dtype=float)
    disp[crop < 100] = 178
    disp[(crop >= 100) & (crop < 240)] = 235

    halo = [pe.withStroke(linewidth=3, foreground="white")]
    fig, ax = plt.subplots(figsize=(10.5, 10.5), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.imshow(disp, cmap="gray", vmin=0, vmax=255, extent=extent, origin="upper",
              interpolation="nearest")

    # centerline, faded to background context
    ax.plot(wp_closed[:, 0], wp_closed[:, 1], color=BLUE, lw=2, alpha=0.1, zorder=4)
    ax.plot(wp[:, 0], wp[:, 1], "o", color=BLUE, ms=4, mec="white", mew=0.7,
            alpha=0.1, zorder=5)

    # initial safe set colored by longitudinal speed (sequential: one hue,
    # light -> dark; truncate the light end so slow points stay visible on white)
    speed_cmap = LinearSegmentedColormap.from_list(
        "greens_trunc", plt.get_cmap("Greens")(np.linspace(0.35, 1.0, 256))
    )
    segments = np.stack([ss_xy[:-1], ss_xy[1:]], axis=1)
    seg_vel = 0.5 * (ss_vel[:-1] + ss_vel[1:])
    # drop the lap-wrap jump segment (end of lap back to start line)
    seg_len = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
    keep = seg_len < 0.5
    lc = LineCollection(segments[keep], cmap=speed_cmap, capstyle="round",
                        linewidth=2.5, zorder=3)
    lc.set_array(seg_vel[keep])
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02)
    cbar.set_label("initial safe set — longitudinal speed [m/s]", color=MUTED,
                   fontsize=10)
    cbar.ax.tick_params(colors=MUTED, labelsize=9)
    cbar.outline.set_edgecolor("#d8d7d0")

    ax.plot(0, 0, marker="*", ms=17, color=INK, mec="white", mew=1, ls="none",
            zorder=6)
    ax.annotate("start (0, 0)", xy=(-0.25, 0.15), xytext=(-5.2, 1.6), color=INK,
                fontsize=10, path_effects=halo,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=1))
    ax.text(4.6, -1.0, "centerline\n(52 waypoints)", color=BLUE, fontsize=10,
            ha="left", va="center", path_effects=halo, zorder=7, alpha=0.55)

    handles = [
        Line2D([], [], color=BLUE, lw=2, marker="o", ms=5, mec="white", alpha=0.35),
        Line2D([], [], color=speed_cmap(0.6), lw=2.5),
        Line2D([], [], color=INK, marker="*", ms=12, ls="none", mec="white"),
    ]
    ax.legend(handles,
              ["centerline_waypoints.csv", "initial_safe_set.csv (speed-coded)",
               "sim origin"],
              loc="upper left", fontsize=10, labelcolor=INK, frameon=True,
              facecolor=SURFACE, edgecolor="#d8d7d0", framealpha=0.95, fancybox=True)

    ax.set_xlabel("x [m]", color=MUTED)
    ax.set_ylabel("y [m]", color=MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    for spine in ax.spines.values():
        spine.set_color("#d8d7d0")
    ax.set_title("Levine lobby track — initial safe set speed profile",
                 color=INK, fontsize=13, pad=12)
    ax.set_aspect("equal")
    plt.tight_layout()

    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    plt.savefig(OUT_PNG, dpi=150, facecolor=SURFACE)
    print(f"saved {os.path.normpath(OUT_PNG)}")
    print(f"speed range: {ss_vel.min():.2f} .. {ss_vel.max():.2f} m/s")


if __name__ == "__main__":
    main()
