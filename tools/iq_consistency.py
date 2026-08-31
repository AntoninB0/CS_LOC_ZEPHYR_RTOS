#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
iq_consistency.py — IQ consistency score for a STATIC channel-sounding setup.

Evaluates the ACQUISITION quality directly from the raw IQ, independently of any
distance estimation. On a static setup the channel is constant, so the phase of
each tone must be stable over time. Per channel k:

    R_k = | mean_t( exp(j * phi_k(t)) ) |   in [0, 1]
    phi_k(t) = arg( Y_I(t) * Y_R(t) )       (two-way product cancels the common LO)

R_k is the mean resultant length (circular statistics): 1 = perfectly stable
phase, 0 = random. Global score = median of R_k over channels; the 10th
percentile flags the worst (faded) channels.

Outputs the GLOBAL score (per beacon) and the PER-CHANNEL score (plot), and can
COMPARE several tests side by side (for a config-comparison table).

Usage:
  python iq_consistency.py --json ../test_A/test_A.json
  python iq_consistency.py --json ../test_A/test_A.json ../test_B/test_B.json    # compare
  python iq_consistency.py --json <f> --beacon 0 --min-samples 30 --out score.png

Reuses only the `iq` field of the test JSON. Dependencies: numpy, matplotlib.
"""

import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np


def consistency(recs, min_samples=20):
    """recs (list of measurement dicts with 'iq') -> (per_channel, global).
    per_channel[ch] = {R, n, amp} ; global = {R_median, R_p10, R_mean,
    R_amp_weighted, n_channels, n_proc}."""
    phasors = defaultdict(list)   # channel -> unit phasors over time
    amps = defaultdict(list)      # channel -> |Y_I*Y_R| over time
    for r in recs:
        for ch, il, ql, ir, qr, tql, tqr in r.get("iq", []):
            if tql == 0 and tqr == 0:
                H = complex(il, ql) * complex(ir, qr)
                a = abs(H)
                if a > 0:
                    phasors[ch].append(H / a)
                    amps[ch].append(a)

    perch = {}
    for ch, v in phasors.items():
        if len(v) >= min_samples:
            perch[ch] = {"R": float(abs(np.mean(v))), "n": len(v),
                         "amp": float(np.mean(amps[ch]))}
    if not perch:
        return {}, {"R_median": float("nan"), "R_p10": float("nan"),
                    "R_mean": float("nan"), "R_amp_weighted": float("nan"),
                    "n_channels": 0, "n_proc": len(recs)}
    R = np.array([d["R"] for d in perch.values()])
    w = np.array([d["amp"] for d in perch.values()])
    glob = {"R_median": float(np.median(R)),
            "R_p10": float(np.percentile(R, 10)),
            "R_mean": float(np.mean(R)),
            "R_amp_weighted": float(np.sum(R * w) / np.sum(w)),
            "n_channels": int(R.size), "n_proc": len(recs)}
    return perch, glob


def load(path, beacon=None):
    doc = json.load(open(path, encoding="utf-8"))
    out = {}
    for bkey, recs in doc.get("mesures", {}).items():
        b = int(bkey[1:]) if str(bkey).startswith("b") else int(bkey)
        if beacon is not None and b != beacon:
            continue
        recs = [r for r in recs if r.get("iq")]
        if recs:
            out[b] = recs
    return out, doc.get("note", "")   # note = label de config (--note)


def reliability_curve(Rvals):
    """Sorted-descending coherence and the fraction of channels kept.
    Point (x %, y) reads: 'the best x % of channels all have R >= y' (y = the
    worst R among the kept top-x %). This is the coherence quantile curve — the
    'top 10/20/50 %' view. Monotonically decreasing."""
    Rs = np.sort(np.asarray(Rvals, float))[::-1]
    n = Rs.size
    frac = np.arange(1, n + 1) / n * 100.0
    return frac, Rs


def report_figure(results, labels, beacon, out):
    """Rich 4-panel report for one beacon (overlays all tests present):
    A per-channel coherence, B reliability-vs-fraction (top-X%), C distribution,
    D coherence vs amplitude."""
    import matplotlib.pyplot as plt

    keys = [lb for lb in labels if (lb, beacon) in results and results[(lb, beacon)][0]]
    if not keys:
        print(f"[report: no channels for beacon b{beacon}]", file=sys.stderr)
        return
    single = len(keys) == 1
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    axA, axB, axC, axD = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]

    for k, lb in enumerate(keys):
        perch, g = results[(lb, beacon)]
        chs = sorted(perch)
        R = np.array([perch[c]["R"] for c in chs])
        amp = np.array([perch[c]["amp"] for c in chs])
        c = colors[k % len(colors)]
        lab = f"{lb} (med {g['R_median']:.2f})"

        axA.plot(chs, R, ".-", ms=4, lw=0.7, color=c, label=lab)
        axA.axhline(g["R_median"], color=c, ls=":", lw=0.7, alpha=0.5)

        frac, Rs = reliability_curve(R)
        axB.plot(frac, Rs, "-", lw=1.5, color=c, label=lab)
        for p in (10, 20, 50, 80):
            idx = max(1, int(round(p / 100 * Rs.size))) - 1
            axB.plot(p, Rs[idx], "o", ms=5, color=c)
            if single:
                axB.annotate(f"{Rs[idx]:.2f}", (p, Rs[idx]), fontsize=8,
                             textcoords="offset points", xytext=(4, 4))

        axC.hist(R, bins=20, range=(0, 1), alpha=0.5, color=c, label=lab)
        axD.plot(amp, R, ".", ms=4, color=c, alpha=0.55, label=lab)

    # A — per-channel coherence (+ amplitude on a twin axis when single test)
    if single:
        perch = results[(keys[0], beacon)][0]
        chs = sorted(perch)
        axAr = axA.twinx()
        axAr.plot(chs, [perch[c]["amp"] for c in chs], color="gray", alpha=0.3, lw=1.0)
        axAr.set_ylabel("amplitude", color="gray", fontsize=8)
        axAr.tick_params(axis="y", labelcolor="gray", labelsize=7)
    axA.set_ylim(0, 1.02); axA.set_xlabel("CS channel"); axA.set_ylabel("coherence R")
    axA.set_title("A · per-channel coherence" + ("  (gray = amplitude)" if single else ""),
                  fontsize=9)
    axA.grid(alpha=0.3); axA.legend(fontsize=7, loc="lower right")

    # B — reliability vs fraction kept (the 'top X%' curve)
    for p in (10, 20, 50, 80):
        axB.axvline(p, color="k", ls=":", lw=0.5, alpha=0.3)
    axB.set_xlim(0, 100); axB.set_ylim(0, 1.02)
    axB.set_xlabel("% of channels kept (best first)")
    axB.set_ylabel("min R in the kept top-%")
    axB.set_title("B · reliability vs fraction  (coherence quantile / top-X%)", fontsize=9)
    axB.grid(alpha=0.3); axB.legend(fontsize=7, loc="lower left")

    # C — coherence distribution
    axC.set_xlim(0, 1); axC.set_xlabel("coherence R"); axC.set_ylabel("channels")
    axC.set_title("C · coherence distribution", fontsize=9); axC.grid(alpha=0.3)
    if not single:
        axC.legend(fontsize=7)

    # D — coherence vs amplitude (fading -> low R)
    axD.set_ylim(0, 1.02); axD.set_xlabel("mean |Y_I·Y_R|  (amplitude / SNR proxy)")
    axD.set_ylabel("coherence R")
    axD.set_title("D · coherence vs amplitude  (fading -> low R)", fontsize=9)
    axD.grid(alpha=0.3)

    fig.suptitle(f"IQ consistency report — beacon b{beacon}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"[report -> {out}]")


def main():
    ap = argparse.ArgumentParser(description="IQ consistency score (static setup)")
    ap.add_argument("--json", nargs="+", required=True, help="one or more test JSONs")
    ap.add_argument("--beacon", type=int, help="only this beacon")
    ap.add_argument("--min-samples", type=int, default=20,
                    help="min procedures per channel to score it (default 20)")
    ap.add_argument("--out", help="output PNG (default: consistency.png)")
    ap.add_argument("--report", action="store_true",
                    help="rich analysis report: per-channel coherence, "
                         "reliability-vs-fraction (top-X%% curve), distribution, "
                         "coherence-vs-amplitude")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # compute everything: results[(label, beacon)] = (perch, glob)
    results = {}
    labels = []
    for path in args.json:
        data, note = load(path, args.beacon)
        label = note or os.path.basename(os.path.dirname(path)) or os.path.basename(path)
        labels.append(label)
        for b, recs in sorted(data.items()):
            results[(label, b)] = consistency(recs, args.min_samples)

    if not results:
        sys.exit("No measurement with IQ ('iq' field) found. Use a test recorded "
                 "with the current uart_console.")

    # ── console : global table ──────────────────────────────────────────────
    print(f"\n{'test':<28} {'b':>2} | {'R med':>7} {'R 10th':>7} {'amp-w':>7} "
          f"{'chan':>5} {'proc':>5}")
    print("-" * 72)
    for (label, b), (perch, g) in results.items():
        print(f"{label[:28]:<28} b{b:<1} | {g['R_median']:>7.3f} {g['R_p10']:>7.3f} "
              f"{g['R_amp_weighted']:>7.3f} {g['n_channels']:>5d} {g['n_proc']:>5d}")
    print()

    if args.report:
        rb = args.beacon if args.beacon is not None \
            else sorted({b for (_, b) in results})[0]
        report_figure(results, labels, rb, args.out or "consistency_report.png")
        return

    # ── plot : per-channel R (one row per beacon, one series per test) ────────
    beacons = sorted({b for (_, b) in results})
    single = len(labels) == 1
    fig, axes = plt.subplots(len(beacons), 1, squeeze=False, sharex=True,
                             figsize=(11, 3.0 * len(beacons)))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for ax, b in zip(axes[:, 0], beacons):
        for k, label in enumerate(labels):
            key = (label, b)
            if key not in results:
                continue
            perch, g = results[key]
            if not perch:
                continue
            chs = sorted(perch)
            R = [perch[c]["R"] for c in chs]
            c = colors[k % len(colors)]
            ax.plot(chs, R, ".-", ms=5, lw=0.8, color=c,
                    label=f"{label}  (median {g['R_median']:.3f})")
            ax.axhline(g["R_median"], color=c, ls=":", lw=0.8, alpha=0.6)
            if single:   # overlay mean amplitude (fading) on a twin axis
                axr = ax.twinx()
                amp = np.array([perch[cc]["amp"] for cc in chs])
                axr.plot(chs, amp, color="gray", alpha=0.35, lw=1.0)
                axr.set_ylabel("mean |Y_I·Y_R|  (amplitude)", color="gray", fontsize=8)
                axr.tick_params(axis="y", labelcolor="gray", labelsize=7)
        ax.set_ylim(0, 1.02); ax.set_ylabel(f"b{b}  consistency R")
        ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="lower right")
    axes[-1, 0].set_xlabel("CS channel (0–78)")
    ttl = "IQ phase consistency per channel  (1 = stable, 0 = random)"
    if single:
        ttl += "  —  gray = mean amplitude (fading)"
    fig.suptitle(ttl, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = args.out or "consistency.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"[per-channel plot -> {out}]")


if __name__ == "__main__":
    main()
