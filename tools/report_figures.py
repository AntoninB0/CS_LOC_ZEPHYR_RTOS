#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
report_figures.py — synthesis figures for the campaign report (2026-09-01).

Reuses the iq_consistency metric (per-channel coherence R) and produces, in
doc/figures/ :
  fig1_strong_vs_faded.png   strong link vs faded link (per-channel fading dips)
  fig2_env_H2.png            same profile (boat, 72 ch), corridor vs outdoor (H2)
  fig3_densite_pirecas.png   drone (24 ch) vs boat (36 ch) at N=3 (density <-> worst case)
  fig4_R_vs_amplitude_H1.png R-vs-amplitude scatter, all conditions (H1)

No data is recomputed other than through iq_consistency.consistency().
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from iq_consistency import consistency, reliability_curve, load

TESTS = "../tests/20260901"
OUT = "../doc/figures"

ENVLABEL = {"ext": "outdoor", "couloir": "corridor"}

# (folder, profile, env, N, {beacon: distance_m}) ; per-beacon distance = RSSI order
DATA = {
    "drone_out_N1_5m_ext":   ("test_20260901_092918", "DRONE_FAST", "ext",     1, {0: 5}),
    "drone_out_N1_15m_ext":  ("test_20260901_093659", "DRONE_FAST", "ext",     1, {0: 15}),
    "drone_out_N3_ext":      ("test_20260901_094650", "DRONE_FAST", "ext",     3, {0: 5, 1: 10, 2: 15}),
    "drone_in_N1_5m_int":    ("test_20260901_100812", "DRONE_INDOOR", "couloir", 1, {0: 5}),
    "drone_in_N1_15m_int":   ("test_20260901_101133", "DRONE_INDOOR", "couloir", 1, {0: 15}),
    "drone_in_N3_int":       ("test_20260901_101759", "DRONE_INDOOR", "couloir", 3, {2: 5, 0: 10, 1: 15}),
    "boat_N1_5m_int":        ("test_20260901_103101", "BOAT", "couloir",       1, {0: 5}),
    "boat_N1_15m_int":       ("test_20260901_102731", "BOAT", "couloir",       1, {0: 15}),
    "boat_N3_int":           ("test_20260901_102401", "BOAT", "couloir",       3, {0: 5, 1: 10, 2: 15}),
    "boat_N1_5m_ext":        ("test_20260901_105150", "BOAT", "ext",           1, {0: 5}),
    "boat_N1_15m_ext":       ("test_20260901_104749", "BOAT", "ext",           1, {0: 15}),
    "boat_N3_ext":           ("test_20260901_105658", "BOAT", "ext",           3, {1: 5, 0: 10, 2: 15}),
}


def perch_of(key, beacon):
    """Return (per_channel, global) for one (test, beacon)."""
    folder = DATA[key][0]
    path = os.path.join(TESTS, folder, folder + ".json")
    data, _ = load(path, beacon)
    recs = data.get(beacon, [])
    return consistency(recs)


def series(key, beacon):
    perch, g = perch_of(key, beacon)
    chs = sorted(perch)
    R = np.array([perch[c]["R"] for c in chs])
    amp = np.array([perch[c]["amp"] for c in chs])
    return np.array(chs), R, amp, g


def fig1_strong_vs_faded():
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    for a, (key, b, title) in zip(ax, [
        ("boat_N1_5m_ext", 0, "Strong link: BOAT outdoor 5 m (RSSI -52.5 dBm)"),
        ("drone_out_N3_ext", 2, "Faded link: DRONE outdoor N=3 15 m (RSSI -84.1 dBm)"),
    ]):
        chs, R, amp, g = series(key, b)
        a.plot(chs, R, ".-", ms=4, lw=0.8, color="C0", label=f"coherence R (median {g['R_median']:.2f})")
        a.axhline(g["R_median"], color="C0", ls=":", lw=0.8, alpha=0.6)
        ar = a.twinx()
        ar.plot(chs, amp, color="gray", alpha=0.35, lw=1.0)
        ar.set_ylabel("amplitude |Y_I·Y_R|", color="gray", fontsize=8)
        ar.tick_params(axis="y", labelcolor="gray", labelsize=7)
        a.set_ylim(0, 1.02); a.set_xlabel("CS channel"); a.set_title(title, fontsize=9)
        a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
    ax[0].set_ylabel("coherence R")
    fig.suptitle("Per-channel coherence: strong link (flat, near 1) vs faded link "
                 "(R dips at the multipath nulls)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, "fig1_strong_vs_faded.png")


def fig2_env_H2():
    """Boat, 72 channels, same profile, corridor vs outdoor, at 15 m (N=1)."""
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    entries = [
        ("boat_N1_15m_int", 0, "BOAT corridor 15 m (RSSI -71.7)", "C1"),
        ("boat_N1_15m_ext", 0, "BOAT outdoor 15 m (RSSI -79.4)", "C0"),
    ]
    for key, b, lab, col in entries:
        chs, R, amp, g = series(key, b)
        ax[0].plot(chs, R, ".-", ms=3, lw=0.7, color=col,
                   label=f"{lab}, median {g['R_median']:.2f}")
        frac, Rs = reliability_curve(R)
        ax[1].plot(frac, Rs, "-", lw=1.8, color=col, label=lab)
        for p in (10, 50):
            idx = max(1, int(round(p / 100 * Rs.size))) - 1
            ax[1].plot(p, Rs[idx], "o", ms=5, color=col)
    ax[0].set_ylim(0, 1.02); ax[0].set_xlabel("CS channel"); ax[0].set_ylabel("coherence R")
    ax[0].set_title("Per-channel coherence (72 channels)", fontsize=9)
    ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8, loc="lower left")
    for p in (10, 20, 50, 80):
        ax[1].axvline(p, color="k", ls=":", lw=0.5, alpha=0.3)
    ax[1].set_xlim(0, 100); ax[1].set_ylim(0, 1.02)
    ax[1].set_xlabel("% of channels kept (best first)")
    ax[1].set_ylabel("worst R in the kept top-%")
    ax[1].set_title("Reliability vs fraction (top-X%)", fontsize=9)
    ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8, loc="lower left")
    fig.suptitle("H2: multipath at comparable SNR: the corridor has a better RSSI "
                 "but a lower worst case (faded channels)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, "fig2_env_H2.png")


def fig3_densite_pirecas():
    """Drone (24 ch) vs boat (36 ch) at N=3 outdoor, at 10 m and 15 m."""
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    panels = [
        (10, [("drone_out_N3_ext", 1, "DRONE 24 ch", "C0"),
              ("boat_N3_ext", 0, "BOAT 36 ch", "C3")]),
        (15, [("drone_out_N3_ext", 2, "DRONE 24 ch", "C0"),
              ("boat_N3_ext", 2, "BOAT 36 ch", "C3")]),
    ]
    for a, (dist, entries) in zip(ax, panels):
        for key, b, lab, col in entries:
            chs, R, amp, g = series(key, b)
            frac, Rs = reliability_curve(R)
            a.plot(frac, Rs, "-", lw=1.8, color=col,
                   label=f"{lab}, median {g['R_median']:.2f} / 10th {g['R_p10']:.2f}")
            idx = max(1, int(round(0.10 * Rs.size))) - 1
            a.plot(10, Rs[idx], "o", ms=6, color=col)
        for p in (10, 20, 50, 80):
            a.axvline(p, color="k", ls=":", lw=0.5, alpha=0.3)
        a.set_xlim(0, 100); a.set_ylim(0, 1.02)
        a.set_xlabel("% of channels kept (best first)")
        a.set_title(f"N=3 outdoor, link at {dist} m", fontsize=9)
        a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
    ax[0].set_ylabel("worst R in the kept top-% (10th pct marked)")
    fig.suptitle("Density vs worst-case trade-off: the drone sparse map (24 ch) "
                 "gives a better 10th percentile at mid range", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, "fig3_densite_pirecas.png")


def fig4_R_vs_amplitude_H1():
    """R vs amplitude scatter, all (test, beacon), colour = environment."""
    fig, a = plt.subplots(figsize=(9, 6))
    cols = {"ext": "C0", "couloir": "C3"}
    seen = set()
    for key, (_, prof, env, N, dmap) in DATA.items():
        for b in dmap:
            perch, _ = perch_of(key, b)
            amp = np.array([d["amp"] for d in perch.values()])
            R = np.array([d["R"] for d in perch.values()])
            lab = ENVLABEL[env] if env not in seen else None
            seen.add(env)
            a.scatter(amp, R, s=10, c=cols[env], alpha=0.35, label=lab, edgecolors="none")
    a.set_xscale("log")
    a.set_ylim(0, 1.02)
    a.set_xlabel("per-channel amplitude  |Y_I·Y_R|  (SNR proxy, log scale)")
    a.set_ylabel("coherence R")
    a.set_title("H1: across all conditions, R collapses as the amplitude (SNR) "
                "drops: a single R(amplitude) trend", fontsize=10)
    a.grid(alpha=0.3, which="both"); a.legend(fontsize=9, title="environment")
    fig.tight_layout()
    save(fig, "fig4_R_vs_amplitude_H1.png")


def fig5_single_vs_multi():
    """Single-beacon (N=1) vs multi-beacon (N=3) coherence at a matched 5 m
    geometry, for the two profiles whose channel map does NOT change with N
    (drone 24 ch, indoor 72 ch) — so the comparison is clean. Shows that the
    per-link coherence at N=3 matches N=1."""
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    panels = [
        ("DRONE_FAST, outdoor, 5 m (24 ch)",
         [("drone_out_N1_5m_ext", 0, "N=1 (single)", "C0"),
          ("drone_out_N3_ext", 0, "N=3 (multi)", "C3")]),
        ("DRONE_INDOOR, corridor, 5 m (72 ch)",
         [("drone_in_N1_5m_int", 0, "N=1 (single)", "C0"),
          ("drone_in_N3_int", 2, "N=3 (multi)", "C3")]),
    ]
    for a, (title, entries) in zip(ax, panels):
        for key, b, lab, col in entries:
            chs, R, amp, g = series(key, b)
            frac, Rs = reliability_curve(R)
            a.plot(frac, Rs, "-", lw=1.8, color=col,
                   label=f"{lab}, median {g['R_median']:.3f} / 10th {g['R_p10']:.3f}")
        for p in (10, 20, 50, 80):
            a.axvline(p, color="k", ls=":", lw=0.5, alpha=0.3)
        a.set_xlim(0, 100); a.set_ylim(0, 1.02)
        a.set_xlabel("% of channels kept (best first)")
        a.set_title(title, fontsize=9)
        a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
    ax[0].set_ylabel("worst R in the kept top-%")
    fig.suptitle("Single vs multi-beacon: the per-link coherence at N=3 matches "
                 "N=1 (same channel map)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, "fig5_single_vs_multi.png")


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print(f"[-> {p}]")


if __name__ == "__main__":
    fig1_strong_vs_faded()
    fig2_env_H2()
    fig3_densite_pirecas()
    fig4_R_vs_amplitude_H1()
    fig5_single_vs_multi()
