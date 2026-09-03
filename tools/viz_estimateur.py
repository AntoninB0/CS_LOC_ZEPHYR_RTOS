#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_estimateur.py — estimator diagnostic figures to ILLUSTRATE uncertainty and
multipath, from the same data as the live chain.

For one (or several) CS measurement, it plots three panels:
  1. IFFT delay profile |IFFT(H)| vs distance — each PEAK = one path (direct +
     reflections). Multipath is directly visible: several peaks, and sometimes a
     reflection higher than the direct one (source of the bias).
  2. Bayesian likelihood per alias lobe — find_amp() score at each candidate lobe
     over [0, R_MAX]. Shows the AMBIGUITY: the winning lobe and its runners-up;
     at low SNR the wrong lobe can come out ahead.
  3. Zoom on the selected lobe — peak width = phase uncertainty (~cm).

This script DOES NOT MODIFY estimation.py: it reuses its public functions
(find_amp, calcIFFTDist, gradient_refine, naboer_global3, lobe_period) and the
cs_decoder decoder, exactly like uart_console. The marked distances are
therefore those of YOUR estimator (before RTT/calibration/median, not plotted).

Usage:
  python viz_estimateur.py --file capture_ref_921600.txt         # 1 fig/beacon (median measurement)
  python viz_estimateur.py --json test_XXXX/test_XXXX.json       # from an enriched test (iq field)
  python viz_estimateur.py --file capture.txt --beacon 0 --counter 42
  python viz_estimateur.py --file capture.txt --worst            # the most outlier measurement/beacon
  python viz_estimateur.py --file capture.txt --index 10         # the 10th valid measurement
  python viz_estimateur.py --port /dev/ttyACM0 --collect 40      # live capture then figures

⚠ --json only works on tests RECORDED WITH IQ (the "iq" field, added recently to
uart_console). Older tests only have the distances.

Outputs: timestamped folder viz_YYYYMMDD_HHMMSS/ with one PNG per plotted measurement.
Dependencies: numpy, scipy, matplotlib (tools venv: see install.txt).
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from cs_decoder import CS_FREQ_BASE_HZ, DropStats, Pairer
import estimation as est

try:
    import matplotlib
    matplotlib.use("Agg")                 # file rendering, headless
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib missing. Activate the venv: cd tools && source .venv/bin/activate "
             "&& pip install -r install.txt")

try:
    from scipy.signal import find_peaks
except ImportError:
    find_peaks = None


# ── Faithful reuse of the input preparation of estimation.estimate() ──────────
# (same lines as estimate(): if you change the tone filter over there, mirror it
#  here so the figures match the live chain.)
def prepare(m):
    """Measurement -> valid tones (ch, f, transfer2, ang1, ang2) or None."""
    ch  = np.array([t.channel for t in m.tones])
    Y_I = np.array([complex(t.i_loc, t.q_loc) for t in m.tones])
    Y_R = np.array([complex(t.i_ref, t.q_ref) for t in m.tones])
    f   = np.array([t.freq_hz for t in m.tones])
    tq  = np.array([t.tq_loc for t in m.tones])
    tqr = np.array([t.tq_ref for t in m.tones])

    valid = (tq == 0) & (tqr == 0) & (np.abs(Y_I) > 0) & (np.abs(Y_R) > 0)
    if np.count_nonzero(valid) < est.MIN_TONES:
        return None
    return {
        "ch": ch[valid],
        "f": f[valid],
        "t2": (Y_I * Y_R)[valid],
        "ang1": (-np.angle(Y_I))[valid],
        "ang2": (-np.angle(Y_R))[valid],
        "n_valid": int(np.count_nonzero(valid)),
        "n_tones": len(m.tones),
    }


def ifft_profile(ch, t2, n=2048):
    """Delay profile: mirror of estimation.calcIFFTDist, but returns the full
    CURVE (x in meters, |IFFT|) and the argmax (= the estimator's d_ifft)."""
    H = np.zeros(79, dtype=complex)
    H[ch] = t2
    yf = np.abs(np.fft.ifft(H, n=n)[:n // 2])
    x = np.arange(n // 2) * est.C / (2 * n * 1e6)     # 1e6 = grid step (1 MHz)
    return x, yf, x[int(np.argmax(yf))]


def bayes_candidates(first_est, f, ang1, ang2):
    """naboer_global3's comb of candidate lobes over [0, R_MAX] and their scores."""
    period = est.lobe_period(f)
    kmin = math.ceil((0.0 - first_est) / period)
    kmax = math.floor((est.R_MAX - first_est) / period)
    if kmin > kmax:
        return np.array([first_est]), est.find_amp(np.array([first_est]), f, ang1, ang2), period
    cand = first_est + np.arange(kmin, kmax + 1) * period
    scores = est.find_amp(cand, f, ang1, ang2)
    return cand, np.atleast_1d(scores), period


def plot_measurement(m, outpath, rmax_view=40.0):
    """Plot the 3 panels for measurement m. Returns (d_ifft, d_bay) or None."""
    p = prepare(m)
    if p is None:
        return None
    ch, f, t2, ang1, ang2 = p["ch"], p["f"], p["t2"], p["ang1"], p["ang2"]

    # YOUR estimator's pipeline (steps 1-3), to mark the same distances.
    x, yf, d_ifft = ifft_profile(ch, t2)
    d_first = est.gradient_refine(ang1, ang2, f, d_ifft)      # naboer's first_est
    d_bay = est.naboer_global3(ang1, ang2, f, d_first)
    cand, scores, period = bayes_candidates(d_first, f, ang1, ang2)

    fig, ax = plt.subplots(3, 1, figsize=(10, 9))

    # 1) IFFT delay profile — multipath = multiple peaks
    ax[0].plot(x, yf, lw=1.1)
    ax[0].axvline(d_ifft, color="tab:red", ls="--", lw=1.2,
                  label=f"argmax IFFT = {d_ifft:.2f} m")
    ax[0].axvline(d_bay, color="tab:green", ls="-", lw=1.2,
                  label=f"Bayesian = {d_bay:.2f} m")
    if find_peaks is not None and yf.max() > 0:
        pk, _ = find_peaks(yf, height=0.25 * yf.max(), distance=8)
        for i in pk[:6]:
            ax[0].annotate(f"{x[i]:.1f} m", (x[i], yf[i]), fontsize=7,
                           textcoords="offset points", xytext=(0, 3),
                           ha="center", color="dimgray")
    # Widen the view if needed so the IFFT argmax (often a distant reflection
    # that the Bayesian corrects) stays visible in the frame.
    view = min(max(rmax_view, 1.15 * max(d_ifft, d_bay)), float(x.max()))
    ax[0].set_xlim(0, view)
    ax[0].set_xlabel("distance (m)"); ax[0].set_ylabel("|IFFT|")
    ax[0].set_title("Delay profile (IFFT) — each peak = one path "
                    "(direct + multipath)")
    ax[0].legend(fontsize=8); ax[0].grid(True, alpha=0.3)

    # 2) Likelihood per alias lobe — ambiguity
    ax[1].vlines(cand, scores.min(), scores, color="tab:blue", alpha=0.35, lw=0.8)
    ax[1].plot(cand, scores, ".", ms=4, color="tab:blue")
    i_best = int(np.argmax(scores))
    ax[1].plot(cand[i_best], scores[i_best], "o", ms=9, mfc="none",
               mec="tab:red", mew=1.6, label=f"winning lobe ≈ {cand[i_best]:.2f} m")
    ax[1].axvline(d_bay, color="tab:green", ls="-", lw=1.0)
    ax[1].set_xlim(0, est.R_MAX)
    ax[1].set_xlabel("candidate distance (m)"); ax[1].set_ylabel("log-likelihood")
    ax[1].set_title(f"Likelihood per alias lobe (spacing {period*100:.1f} cm) — "
                    "height = plausibility; winner/2nd gap = ambiguity margin")
    ax[1].legend(fontsize=8); ax[1].grid(True, alpha=0.3)

    # 3) Zoom on the selected lobe — phase uncertainty
    rr = np.linspace(d_bay - 5 * period, d_bay + 5 * period, 800)
    sc = est.find_amp(rr, f, ang1, ang2)
    ax[2].plot(rr, sc, lw=1.1)
    ax[2].axvline(d_bay, color="tab:green", ls="-", lw=1.2,
                  label=f"Bayesian = {d_bay:.3f} m")
    ax[2].set_xlabel("distance (m)"); ax[2].set_ylabel("log-likelihood")
    ax[2].set_title(f"Zoom on the selected lobe — width ≈ phase uncertainty "
                    f"(lobe spacing {period*100:.1f} cm)")
    ax[2].legend(fontsize=8); ax[2].grid(True, alpha=0.3)

    fig.suptitle(f"b{m.beacon}  rc={m.counter}  |  {p['n_valid']}/{p['n_tones']} valid tones  "
                 f"|  RSSI {m.rssi_loc}/{m.rssi_ref} dBm  |  "
                 f"IFFT={d_ifft:.2f} m  Bayesian={d_bay:.2f} m", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(outpath, dpi=120)
    plt.close(fig)
    return d_ifft, d_bay


# ── Measurement collection (file or live) ─────────────────────────────────────

def measurements_from_file(path, beacon=None):
    stats = DropStats()
    pairer = Pairer(stats)
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = pairer.feed(line)
            if m is not None and (beacon is None or m.beacon == beacon):
                out.append(m)
    print(stats.summary(), file=sys.stderr)
    return out


# ── Reconstruction from an enriched test JSON ("iq" field per measurement) ────
# The --test JSON stores, per measurement, iq = [[channel, i_loc, q_loc, i_ref,
# q_ref, tq_loc, tq_ref], ...]. We rebuild minimal objects compatible with
# prepare()/plot_measurement() (same attributes as cs_decoder.ToneIQ /
# Measurement, minus the RTT not needed for the figures).
class _Tone:
    __slots__ = ("channel", "freq_hz", "i_loc", "q_loc", "tq_loc",
                 "i_ref", "q_ref", "tq_ref")

    def __init__(self, ch, il, ql, ir, qr, tql, tqr):
        self.channel = ch
        self.freq_hz = CS_FREQ_BASE_HZ + ch * 1_000_000
        self.i_loc, self.q_loc, self.tq_loc = il, ql, tql
        self.i_ref, self.q_ref, self.tq_ref = ir, qr, tqr


class _Meas:
    __slots__ = ("beacon", "counter", "t_ms", "rssi_loc", "rssi_ref",
                 "tones", "rtt_pairs")

    def __init__(self, beacon, counter, t_ms, rssi_loc, rssi_ref, tones):
        self.beacon, self.counter, self.t_ms = beacon, counter, t_ms
        self.rssi_loc, self.rssi_ref, self.tones = rssi_loc, rssi_ref, tones
        self.rtt_pairs = []


def meas_from_record(beacon, r):
    """Build a _Meas from a JSON record (must contain 'iq')."""
    return _Meas(beacon, r.get("counter"), r.get("t_ms"),
                 r.get("rssi_loc"), r.get("rssi_ref"),
                 [_Tone(*t) for t in r["iq"]])


def measurements_from_json(path, beacon=None):
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    out, n_no_iq = [], 0
    for bkey, recs in doc.get("mesures", {}).items():
        b = int(bkey[1:]) if str(bkey).startswith("b") else int(bkey)
        if beacon is not None and b != beacon:
            continue
        for r in recs:
            if r.get("iq"):
                out.append(meas_from_record(b, r))
            else:
                n_no_iq += 1
    if not out:
        sys.exit(f"{path}: no measurement with IQ (the 'iq' field). "
                 f"{'This test predates IQ storage.' if n_no_iq else ''} "
                 "Regenerate from a recent --test or a raw capture (--file).")
    return out


def measurements_from_port(port, baud, secs, beacon=None):
    import serial
    stats = DropStats()
    pairer = Pairer(stats)
    out = []
    deadline = time.time() + secs
    buf = b""
    with serial.Serial(port, baud, timeout=0.2) as ser:
        print(f"[capture {secs} s on {port} @ {baud}]", file=sys.stderr)
        while time.time() < deadline:
            buf += ser.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                m = pairer.feed(line)
                if m is not None and (beacon is None or m.beacon == beacon):
                    out.append(m)
    print(stats.summary(), file=sys.stderr)
    return out


# ── Selection of the measurements to plot ─────────────────────────────────────

def select(meas, args):
    """Return the list of measurements to plot according to the options."""
    if args.counter is not None:
        sel = [m for m in meas if m.counter == args.counter
               and (args.beacon is None or m.beacon == args.beacon)]
        return sel or []
    if args.index is not None:
        return [meas[args.index]] if 0 <= args.index < len(meas) else []

    # Default / --worst: one REPRESENTATIVE measurement per beacon. We estimate
    # d_bay of each measurement (via the estimator) then take the median
    # (typical case) or the largest deviation from that median (--worst,
    # multipath case).
    by_beacon = {}
    for m in meas:
        p = prepare(m)
        if p is None:
            continue
        d = est.naboer_global3(p["ang1"], p["ang2"], p["f"],
                               est.gradient_refine(p["ang1"], p["ang2"], p["f"],
                                                   ifft_profile(p["ch"], p["t2"])[2]))
        by_beacon.setdefault(m.beacon, []).append((d, m))
    chosen = []
    for b, lst in sorted(by_beacon.items()):
        med = float(np.median([d for d, _ in lst]))
        # typical = measurement CLOSEST to the median; --worst = the farthest
        dist_to_med = lambda dm: abs(dm[0] - med)      # noqa: E731
        pick = max(lst, key=dist_to_med) if args.worst else min(lst, key=dist_to_med)
        chosen.append(pick[1])
    return chosen


def main():
    ap = argparse.ArgumentParser(
        description="IFFT + Bayesian figures (uncertainty / multipath)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="raw capture (IQL/IQP lines)")
    src.add_argument("--json", help="enriched test JSON ('iq' field per measurement)")
    src.add_argument("--port", help="live capture then figures")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--collect", type=int, default=30, help="live capture duration (s)")
    ap.add_argument("--beacon", type=int, help="keep only one beacon")
    ap.add_argument("--counter", type=int, help="plot a specific procedure")
    ap.add_argument("--index", type=int, help="plot the k-th valid measurement")
    ap.add_argument("--worst", action="store_true",
                    help="plot the most outlier measurement per beacon (multipath case)")
    ap.add_argument("--rmax-view", type=float, default=40.0,
                    help="max distance shown in the IFFT profile (m)")
    ap.add_argument("--out", help="output folder (default viz_timestamped)")
    args = ap.parse_args()

    if args.file:
        meas = measurements_from_file(args.file, args.beacon)
    elif args.json:
        meas = measurements_from_json(args.json, args.beacon)
    else:
        meas = measurements_from_port(args.port, args.baud, args.collect, args.beacon)
    if not meas:
        sys.exit("No complete measurement in the input.")

    sel = select(meas, args)
    if not sel:
        sys.exit("No measurement matches the selection.")

    outdir = args.out or time.strftime("viz_%Y%m%d_%H%M%S")
    os.makedirs(outdir, exist_ok=True)
    for m in sel:
        png = os.path.join(outdir, f"b{m.beacon}_rc{m.counter}.png")
        res = plot_measurement(m, png, args.rmax_view)
        if res is None:
            print(f"[b{m.beacon} rc{m.counter}: too few valid tones, skipped]",
                  file=sys.stderr)
            continue
        d_ifft, d_bay = res
        print(f"b{m.beacon} rc{m.counter}: IFFT={d_ifft:.2f} m  Bayesian={d_bay:.2f} m  -> {png}")
    print(f"[figures -> {outdir}/]")


if __name__ == "__main__":
    main()
