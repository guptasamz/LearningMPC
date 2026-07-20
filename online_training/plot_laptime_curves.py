"""Lap-time curves of the final residual campaign, color-coded by friction.

One panel per map; one line per mu (sequential ramp: light = low grip,
dark = high grip); red x where the run crashed (placed on the last completed
lap). No point markers. y-scales are per-map (lap times differ by 30x
between barc_oval and Sepang).

Usage:
    python plot_laptime_curves.py [--campaign ../src_gym/results_residual_dynmics/final]
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
from matplotlib import cm, colors

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = ["barc_oval", "barc_oval_orginal", "levinelobby_track",
        "Sepang", "YasMarina"]
MUS = [f"{x/10:.1f}" for x in range(1, 11)]
CRASH_RED = "#c0392b"


def load_cell(campaign, m, mu):
    d = os.path.join(campaign, m, f"{m}_{mu}")
    laps = sorted(glob.glob(os.path.join(d, "laps_*.csv")))
    log = os.path.join(d, "run.log")
    if not laps or not os.path.exists(log):
        return None, None
    df = pd.read_csv(laps[-1])
    crashed = bool(re.search(r"crashed: True", open(log).read()))
    return df, crashed


def draw_map_panel(ax, campaign, m, mu_color, end_labels=False):
    """Plot all mu lines of one map onto ax; returns list of mus drawn."""
    drew, ends = [], []
    for mu in MUS:
        df, crashed = load_cell(campaign, m, mu)
        if df is None or not len(df):
            continue
        drew.append(mu)
        x = (df["lap_index(iter)"] - 2).values     # lap 0 = first LMPC lap
        y = df["lap_time_s"].values
        ax.plot(x, y, color=mu_color[mu], lw=1.8)
        if crashed:
            ax.plot(x[-1], y[-1], "x", color=CRASH_RED, ms=9, mew=2.2,
                    zorder=5)
        ends.append((x[-1], y[-1], mu))
    if end_labels and ends:
        # stagger labels that would collide (lines ending near the same lap
        # with close lap times); a thin leader connects a moved label to its
        # line end
        # gap sized from the AXIS scale (label height), not the endpoint
        # spread; widen x so edge labels don't clip
        y0, y1 = ax.get_ylim()
        gap = (y1 - y0) * 0.035
        x_range = max(x2 for x2, _, _ in ends) or 1.0
        ax.set_xlim(right=x_range * 1.10)
        dx = 0.014 * x_range
        placed = []
        for xe, ye, mu in sorted(ends, key=lambda e: (round(e[0], -1), e[1])):
            ly = ye
            while any(abs(xe - px) < 0.08 * x_range and abs(ly - py) < gap
                      for px, py in placed):
                ly += gap
            placed.append((xe, ly))
            if abs(ly - ye) > 1e-9:
                ax.plot([xe + 0.2 * dx, xe + 0.9 * dx], [ye, ly],
                        color="#bbbbbb", lw=0.6, zorder=4)
            ax.text(xe + dx, ly, mu, fontsize=8.5, color="#333333",
                    ha="left", va="center", zorder=6)
    ax.set_xlabel("lap")
    ax.set_ylabel("lap time [s]")
    ax.grid(True, alpha=0.25)
    return drew


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--campaign",
                    default=os.path.join(HERE, "..", "src_gym",
                                         "results_residual_dynmics", "final"))
    ap.add_argument("--out", default=os.path.join(
        HERE, "results", "campaign", "laptime_curves_final_campaign.png"))
    args = ap.parse_args()

    # sequential single-hue ramp over mu (magnitude), truncated so the
    # lightest step stays visible on white
    cmap = cm.get_cmap("Blues")
    mu_color = {mu: cmap(0.30 + 0.68 * (float(mu) - 0.1) / 0.9) for mu in MUS}

    # ---- per-map figures, saved into each map's campaign folder ----
    for m in MAPS:
        fig1, ax1 = plt.subplots(figsize=(10, 6))
        drawn = draw_map_panel(ax1, args.campaign, m, mu_color,
                               end_labels=True)
        if not drawn:
            plt.close(fig1)
            continue
        handles = [Line2D([], [], color=mu_color[mu], lw=2.2,
                          label=f"mu = {mu}") for mu in drawn]
        handles.append(Line2D([], [], marker="x", ls="none", color=CRASH_RED,
                              ms=9, mew=2.2, label="crashed after this lap"))
        ax1.legend(handles=handles, fontsize=9, ncol=2, framealpha=0.9)
        ax1.set_title(f"{m} — LMPC learnt dynamics (warm-start + r_d 0.5): "
                      "lap time by friction")
        fig1.tight_layout()
        out1 = os.path.join(args.campaign, m, "laptime_curves.png")
        fig1.savefig(out1, dpi=140, bbox_inches="tight")
        plt.close(fig1)
        print(f"saved {out1}")

    # ---- per-map small multiples: one panel per mu, rest as gray context
    #      (the readable answer when many lines finish within a tight band) ----
    for m in MAPS:
        cells = {mu: load_cell(args.campaign, m, mu) for mu in MUS}
        cells = {mu: v for mu, v in cells.items()
                 if v[0] is not None and len(v[0])}
        if not cells:
            continue
        n = len(cells)
        ncols = 4
        nrows = int(np.ceil(n / ncols))
        fig2, axes2 = plt.subplots(nrows, ncols,
                                   figsize=(3.6 * ncols, 2.9 * nrows),
                                   sharex=True, sharey=True)
        axes2 = np.atleast_1d(axes2).flat
        for ax2, (mu, (df, crashed)) in zip(axes2, cells.items()):
            for mu_b, (df_b, _) in cells.items():   # context
                ax2.plot(df_b["lap_index(iter)"] - 2, df_b["lap_time_s"],
                         color="#d9d9d9", lw=0.9)
            x = (df["lap_index(iter)"] - 2).values
            y = df["lap_time_s"].values
            ax2.plot(x, y, color=mu_color[mu], lw=2.0)
            if crashed:
                ax2.plot(x[-1], y[-1], "x", color=CRASH_RED, ms=8, mew=2.0)
                ax2.set_title(f"mu = {mu}  (crash lap {int(x[-1])})",
                              fontsize=10)
            else:
                ax2.set_title(f"mu = {mu}  (100 laps clean)", fontsize=10)
            ax2.grid(True, alpha=0.25)
        for ax2 in list(axes2)[n:]:
            ax2.axis("off")
        fig2.suptitle(f"{m} — one panel per friction (gray = other runs)",
                      fontsize=12)
        fig2.supxlabel("lap")
        fig2.supylabel("lap time [s]")
        fig2.tight_layout()
        out2 = os.path.join(args.campaign, m, "laptime_curves_per_mu.png")
        fig2.savefig(out2, dpi=140, bbox_inches="tight")
        plt.close(fig2)
        print(f"saved {out2}")

    # ---- combined overview figure ----
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    axes = axes.flat
    for ax, m in zip(axes, MAPS):
        draw_map_panel(ax, args.campaign, m, mu_color)
        ax.set_title(m)

    # legend panel: mu ramp as a colorbar + crash marker
    ax = axes[len(MAPS)]
    ax.axis("off")
    sm = cm.ScalarMappable(norm=colors.Normalize(0.1, 1.0),
                           cmap=colors.LinearSegmentedColormap.from_list(
                               "mus", [mu_color[mu] for mu in MUS]))
    cb = fig.colorbar(sm, ax=ax, orientation="horizontal", fraction=0.4,
                      aspect=25, pad=0.1)
    cb.set_label("plant friction mu")
    cb.set_ticks([0.1, 0.4, 0.7, 1.0])
    ax.legend(handles=[Line2D([], [], marker="x", ls="none", color=CRASH_RED,
                              ms=9, mew=2.2, label="crashed after this lap")],
              loc="upper center", frameon=False, fontsize=11)

    fig.suptitle("LMPC with learnt dynamics (warm-start + r_d 0.5) — "
                 "lap-time convergence by friction", y=0.995, fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, dpi=140, bbox_inches="tight")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
