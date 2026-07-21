"""Validate Track::initialize_width against the TUM centerline widths.

Replicates the C++ half-width computation EXACTLY (track.h + occupancy_grid.h):
  * occupancy grid from <map>_map.png/.yaml via the map_server rule
    (same as lmpc_gym.load_occupancy_grid)
  * occupied  <=> cell value > 50 (unknown -1 is NOT occupied)
  * inflate_map: every occupied cell paints the square window
    [i-mc, i+mc) x [j-mc, j+mc), mc = ceil(margin/resolution)
  * grid lookup with the ceil()-1 index convention of xy2ind
  * ray-march from each centerline point along +/- the perpendicular of the
    local tangent in steps of one grid resolution until occupied

and compares against the w_tr_right_m / w_tr_left_m columns of
<map>_centerline.csv, at margin 0 (raw walls - the direct comparison) and at
the controller margin (what the QP actually uses).

Usage:
    python validate_halfwidths.py --map gold_conference_room [--margin 0.3]
"""

import argparse
import os

import numpy as np
import pandas as pd
import yaml
from PIL import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, "..", "data", "maps")
THRESHOLD = 50


def load_grid(stem):
    with open(stem + ".yaml") as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(stem + ".png").convert("L"), dtype=np.float64)
    p = (255.0 - img) / 255.0
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[p > meta["occupied_thresh"]] = 100
    grid[p < meta["free_thresh"]] = 0
    grid = np.flipud(grid)                      # row 0 = origin (bottom)
    return grid, float(meta["resolution"]), \
        float(meta["origin"][0]), float(meta["origin"][1])


def inflate(grid, margin, res):
    """occupancy_grid::inflate_map: square window [i-mc, i+mc) per occupied
    cell (upper bound EXCLUSIVE, exactly as the C++ loops)."""
    if margin <= 0:
        return grid.copy()
    mc = int(np.ceil(margin / res))
    occ = grid > THRESHOLD
    out = grid.copy()
    inflated = np.zeros_like(occ)
    for dy in range(-mc, mc):
        for dx in range(-mc, mc):
            shifted = np.zeros_like(occ)
            ys = slice(max(0, dy), occ.shape[0] + min(0, dy))
            yd = slice(max(0, -dy), occ.shape[0] + min(0, -dy))
            xs = slice(max(0, dx), occ.shape[1] + min(0, dx))
            xd = slice(max(0, -dx), occ.shape[1] + min(0, -dx))
            shifted[ys, xs] = occ[yd, xd]
            inflated |= shifted
    out[inflated] = 100
    return out


def is_occupied(grid, res, ox, oy, x, y):
    """occupancy_grid::is_xy_occupied with the exact ceil()-1 indexing."""
    xi = int(np.ceil((x - ox) / res)) - 1
    yi = int(np.ceil((y - oy) / res)) - 1
    yi = max(0, min(grid.shape[0] - 1, yi))
    xi = max(0, min(grid.shape[1] - 1, xi))
    return grid[yi, xi] > THRESHOLD


def ray_width(grid, res, ox, oy, p, n, max_steps=400):
    """march from p along unit vector n in steps of res until occupied."""
    for t in range(max_steps):
        q = p + t * res * n
        if is_occupied(grid, res, ox, oy, q[0], q[1]):
            return np.linalg.norm(q - p), q
    return np.nan, p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="gold_conference_room")
    ap.add_argument("--margin", type=float, default=0.3,
                    help="controller MAP_MARGIN used in the run")
    args = ap.parse_args()
    d = os.path.join(MAPS, args.map)

    cl = pd.read_csv(os.path.join(d, f"{args.map}_centerline.csv"),
                     comment="#", header=None,
                     names=["x", "y", "w_right", "w_left"])
    P = cl[["x", "y"]].to_numpy()
    seg = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)][:-1]

    grid0, res, ox, oy = load_grid(os.path.join(d, f"{args.map}_map"))
    grids = {"raw walls (margin 0)": inflate(grid0, 0.0, res),
             f"inflated (margin {args.margin})": inflate(grid0, args.margin,
                                                         res)}

    # tangents by central difference on the closed centerline
    T = (np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0))
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    NL = np.stack([-T[:, 1], T[:, 0]], axis=1)   # left normal
    NR = -NL

    results, hits = {}, {}
    for gname, g in grids.items():
        wl = np.empty(len(P)); wr = np.empty(len(P))
        hl = np.empty_like(P); hr = np.empty_like(P)
        for i in range(len(P)):
            wr[i], hr[i] = ray_width(g, res, ox, oy, P[i], NR[i])
            wl[i], hl[i] = ray_width(g, res, ox, oy, P[i], NL[i])
        results[gname] = (wl, wr)
        hits[gname] = (hl, hr)

    wl0, wr0 = results["raw walls (margin 0)"]
    print(f"{args.map}: {len(P)} centerline points, track {s[-1]+seg[-1]:.1f} m")
    print(f"CSV widths:      left {cl.w_left.mean():.3f} m mean, "
          f"right {cl.w_right.mean():.3f} m mean")
    print(f"ray-march (m=0): left {np.nanmean(wl0):.3f} m mean, "
          f"right {np.nanmean(wr0):.3f} m mean")
    print(f"|raymarch - csv|: left mean {np.nanmean(np.abs(wl0-cl.w_left)):.3f} "
          f"max {np.nanmax(np.abs(wl0-cl.w_left)):.3f} | right mean "
          f"{np.nanmean(np.abs(wr0-cl.w_right)):.3f} "
          f"max {np.nanmax(np.abs(wr0-cl.w_right)):.3f} m")

    fig = plt.figure(figsize=(13, 10))
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(s, cl.w_left, color="#1a7a3c", lw=1.8, label="CSV w_tr_left")
    ax.plot(s, cl.w_right, color="#1a7a3c", lw=1.8, ls="--",
            label="CSV w_tr_right")
    ax.plot(s, wl0, color="#2a78d6", lw=1.4, label="ray-march left (margin 0)")
    ax.plot(s, wr0, color="#2a78d6", lw=1.4, ls="--",
            label="ray-march right (margin 0)")
    wlm, wrm = results[f"inflated (margin {args.margin})"]
    ax.plot(s, wlm, color="#d6702a", lw=1.2,
            label=f"ray-march left (margin {args.margin}) = controller")
    ax.plot(s, wrm, color="#d6702a", lw=1.2, ls="--",
            label=f"ray-march right (margin {args.margin})")
    ax.set_xlabel("track position s [m]")
    ax.set_ylabel("half width [m]")
    ax.set_title(f"{args.map} — half-widths: track.h ray-march vs centerline CSV")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 1, 2)
    g = grids["raw walls (margin 0)"]
    ax.imshow(np.flipud(g > THRESHOLD), cmap="gray_r", alpha=0.35,
              extent=[ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res],
              origin="upper")
    ax.plot(P[:, 0], P[:, 1], color="#555555", lw=0.8, label="centerline")
    hl, hr = hits["raw walls (margin 0)"]
    ax.plot(hl[:, 0], hl[:, 1], ".", color="#2a78d6", ms=1.5,
            label="ray-march wall hits (margin 0)")
    ax.plot(hr[:, 0], hr[:, 1], ".", color="#2a78d6", ms=1.5)
    bl = P + NL * cl.w_left.to_numpy()[:, None]
    br = P + NR * cl.w_right.to_numpy()[:, None]
    ax.plot(bl[:, 0], bl[:, 1], ".", color="#1a7a3c", ms=1.5,
            label="CSV-width boundary")
    ax.plot(br[:, 0], br[:, 1], ".", color="#1a7a3c", ms=1.5)
    ax.set_aspect("equal")
    ax.legend(fontsize=9)
    ax.set_title("wall hits: ray-march (blue) vs CSV widths (green)")

    fig.tight_layout()
    out = os.path.join(d, "halfwidth_validation.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"plot: {out}")


if __name__ == "__main__":
    main()
