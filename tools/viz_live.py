#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_live.py — INTERACTIVE viewer of the distance-estimation pipeline.

From an enriched test JSON (per-measurement `iq` field), it replays, for EACH
measurement, the steps of estimation.py and displays them:
  - top    : distance vs time (smoothed d_m + raw d_raw), with a time CURSOR
  - middle : IFFT delay profile (peaks = paths), argmax vs Bayesian
  - bottom-left  : Bayesian likelihood per alias lobe (candidates + winner)
  - bottom-right : zoom on the selected lobe (width = phase uncertainty)
The title shows the DECISION step by step: IFFT -> gradient -> naboer -> final.

Controls:
  - Bottom slider = time cursor (measurement index). Arrow keys <-/-> also work.
  - Radio buttons (if several beacons) = beacon selection.

Reuses estimation.py (the exact estimator functions), without modifying it.

Usage:
  python viz_live.py --json ../test_YYYYMMDD_HHMMSS/test_YYYYMMDD_HHMMSS.json
  python viz_live.py --json <f> --beacon 0
  python viz_live.py --json <f> --snapshot frame.png --index 42   # export one image (headless)

Requires a display (interactive window). Dependencies: numpy, scipy, matplotlib.
"""

import argparse
import json
import math
import sys

import numpy as np

from cs_decoder import CS_FREQ_BASE_HZ
import estimation as est


# ── Rebuild a measurement from a JSON record (iq field) ───────────────────────
# iq = [[channel, i_loc, q_loc, i_ref, q_ref, tq_loc, tq_ref], ...]
class _Tone:
    __slots__ = ("channel", "freq_hz", "i_loc", "q_loc", "tq_loc",
                 "i_ref", "q_ref", "tq_ref")

    def __init__(self, ch, il, ql, ir, qr, tql, tqr):
        self.channel = ch
        self.freq_hz = CS_FREQ_BASE_HZ + ch * 1_000_000
        self.i_loc, self.q_loc, self.tq_loc = il, ql, tql
        self.i_ref, self.q_ref, self.tq_ref = ir, qr, tqr


def _prepare(rec):
    """Record -> valid tones (same rules as estimation.estimate)."""
    tones = [_Tone(*t) for t in rec["iq"]]
    ch  = np.array([t.channel for t in tones])
    Y_I = np.array([complex(t.i_loc, t.q_loc) for t in tones])
    Y_R = np.array([complex(t.i_ref, t.q_ref) for t in tones])
    f   = np.array([t.freq_hz for t in tones])
    tq  = np.array([t.tq_loc for t in tones])
    tqr = np.array([t.tq_ref for t in tones])
    valid = (tq == 0) & (tqr == 0) & (np.abs(Y_I) > 0) & (np.abs(Y_R) > 0)
    if np.count_nonzero(valid) < est.MIN_TONES:
        return None
    return {"ch": ch[valid], "f": f[valid], "t2": (Y_I * Y_R)[valid],
            "ang1": (-np.angle(Y_I))[valid], "ang2": (-np.angle(Y_R))[valid],
            "n": int(np.count_nonzero(valid)), "ntot": len(tones)}


def _ifft_profile(ch, t2, n=2048):
    H = np.zeros(79, dtype=complex)
    H[ch] = t2
    yf = np.abs(np.fft.ifft(H, n=n)[:n // 2])
    x = np.arange(n // 2) * est.C / (2 * n * 1e6)
    return x, yf, x[int(np.argmax(yf))]


def compute(rec):
    """Replay the pipeline for one measurement. Returns everything the plots
    need, or None if the measurement is rejected (too few valid tones)."""
    p = _prepare(rec)
    if p is None:
        return None
    x, yf, d_ifft = _ifft_profile(p["ch"], p["t2"])
    d_grad = est.gradient_refine(p["ang1"], p["ang2"], p["f"], d_ifft)   # local step
    d_bay = est.naboer_global3(p["ang1"], p["ang2"], p["f"], d_grad)      # anti-alias
    period = est.lobe_period(p["f"])
    kmin = math.ceil((0.0 - d_grad) / period)
    kmax = math.floor((est.R_MAX - d_grad) / period)
    if kmin <= kmax:
        cand = d_grad + np.arange(kmin, kmax + 1) * period
        scores = np.atleast_1d(est.find_amp(cand, p["f"], p["ang1"], p["ang2"]))
    else:
        cand = np.array([d_grad]); scores = np.atleast_1d(est.find_amp(cand, p["f"], p["ang1"], p["ang2"]))
    return {"x": x, "yf": yf, "d_ifft": d_ifft, "d_grad": d_grad, "d_bay": d_bay,
            "cand": cand, "scores": scores, "period": period,
            "f": p["f"], "ang1": p["ang1"], "ang2": p["ang2"],
            "n": p["n"], "ntot": p["ntot"]}


def load_json(path, beacon=None):
    doc = json.load(open(path, encoding="utf-8"))
    data = {}
    for bkey, recs in doc.get("mesures", {}).items():
        b = int(bkey[1:]) if str(bkey).startswith("b") else int(bkey)
        if beacon is not None and b != beacon:
            continue
        recs = [r for r in recs if r.get("iq")]
        if recs:
            recs.sort(key=lambda r: r["t_ms"])
            data[b] = recs
    if not data:
        sys.exit(f"{path}: no measurement with IQ ('iq' field). "
                 "Use a test recorded with the current uart_console.")
    return data


def main():
    ap = argparse.ArgumentParser(description="Interactive viewer of the estimation pipeline")
    ap.add_argument("--json", required=True, help="enriched test JSON (iq field)")
    ap.add_argument("--beacon", type=int, help="show only one beacon")
    ap.add_argument("--index", type=int, default=0, help="starting / snapshot measurement")
    ap.add_argument("--snapshot", help="export one image and exit (headless)")
    args = ap.parse_args()

    import matplotlib
    if args.snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, RadioButtons

    data = load_json(args.json, args.beacon)
    beacons = sorted(data)

    fig = plt.figure(figsize=(13, 9))
    left = 0.20 if len(beacons) > 1 else 0.08
    gs = fig.add_gridspec(3, 2, left=left, right=0.97, top=0.90, bottom=0.15,
                          hspace=0.55, wspace=0.22)
    ax_time = fig.add_subplot(gs[0, :])
    ax_ifft = fig.add_subplot(gs[1, :])
    ax_bay = fig.add_subplot(gs[2, 0])
    ax_zoom = fig.add_subplot(gs[2, 1])
    ax_slider = fig.add_axes([left, 0.05, 0.97 - left, 0.03])

    state = {"b": beacons[0], "i": args.index, "cursor": None, "t": []}

    def draw_time():
        """Distance/time series of the current beacon + cursor (redrawn on switch)."""
        recs = data[state["b"]]
        t0 = recs[0]["t_ms"]
        t = [(r["t_ms"] - t0) / 1000.0 for r in recs]
        state["t"] = t
        ax_time.clear()
        ax_time.plot(t, [r["d_m"] for r in recs], "-", lw=1.0,
                     color="tab:blue", label="d_m (smoothed)")
        db = [(ti, r.get("d_brut_m")) for ti, r in zip(t, recs)
              if r.get("d_brut_m") is not None]
        if db:
            ax_time.plot(*zip(*db), ".", ms=3, color="tab:orange",
                         alpha=0.45, label="d_raw")
        ax_time.set_xlabel("time (s)"); ax_time.set_ylabel("distance (m)")
        ax_time.grid(alpha=0.3); ax_time.legend(fontsize=8, loc="upper right")
        state["cursor"] = ax_time.axvline(t[0], color="red", lw=1.4)

    def draw(i):
        recs = data[state["b"]]
        i = max(0, min(i, len(recs) - 1))
        state["i"] = i
        rec = recs[i]
        # time cursor
        if state["t"]:
            state["cursor"].set_xdata([state["t"][i], state["t"][i]])
        for ax in (ax_ifft, ax_bay, ax_zoom):
            ax.clear()

        res = compute(rec)
        if res is None:
            ax_ifft.text(0.5, 0.5, "measurement rejected (too few valid tones)",
                         ha="center", va="center", transform=ax_ifft.transAxes)
            fig.suptitle(f"b{state['b']}  measurement {i+1}/{len(recs)}  rc={rec.get('counter')}  "
                         f"— REJECTED", fontsize=11)
            fig.canvas.draw_idle()
            return

        x, yf = res["x"], res["yf"]
        d_ifft, d_grad, d_bay = res["d_ifft"], res["d_grad"], res["d_bay"]
        period = res["period"]

        # 1) IFFT delay profile
        ax_ifft.plot(x, yf, lw=1.1)
        ax_ifft.axvline(d_ifft, color="tab:red", ls="--", lw=1.2,
                        label=f"IFFT argmax = {d_ifft:.2f} m")
        ax_ifft.axvline(d_bay, color="tab:green", lw=1.2,
                        label=f"Bayesian = {d_bay:.2f} m")
        view = min(max(40.0, 1.15 * max(d_ifft, d_bay)), float(x.max()))
        ax_ifft.set_xlim(0, view)
        ax_ifft.set_xlabel("distance (m)"); ax_ifft.set_ylabel("|IFFT|")
        ax_ifft.set_title("Delay profile (IFFT) — peaks = paths (direct + multipath)",
                          fontsize=9)
        ax_ifft.legend(fontsize=8); ax_ifft.grid(alpha=0.3)

        # 2) likelihood per alias lobe
        cand, scores = res["cand"], res["scores"]
        ax_bay.vlines(cand, scores.min(), scores, color="tab:blue", alpha=0.35, lw=0.7)
        ax_bay.plot(cand, scores, ".", ms=3, color="tab:blue")
        ib = int(np.argmax(scores))
        ax_bay.plot(cand[ib], scores[ib], "o", ms=9, mfc="none", mec="tab:red", mew=1.6)
        ax_bay.axvline(d_bay, color="tab:green", lw=1.0)
        ax_bay.set_xlim(0, est.R_MAX)
        ax_bay.set_xlabel("candidate distance (m)"); ax_bay.set_ylabel("log-likelihood")
        ax_bay.set_title(f"Likelihood per alias lobe (spacing {period*100:.1f} cm)", fontsize=9)
        ax_bay.grid(alpha=0.3)

        # 3) zoom on the selected lobe
        rr = np.linspace(d_bay - 5 * period, d_bay + 5 * period, 600)
        sc = est.find_amp(rr, res["f"], res["ang1"], res["ang2"])
        ax_zoom.plot(rr, sc, lw=1.1)
        ax_zoom.axvline(d_bay, color="tab:green", lw=1.2)
        ax_zoom.set_xlabel("distance (m)"); ax_zoom.set_ylabel("log-likelihood")
        ax_zoom.set_title("Zoom on selected lobe — width = uncertainty", fontsize=9)
        ax_zoom.grid(alpha=0.3)

        # decision, step by step
        dfin = rec.get("d_m")
        fig.suptitle(
            f"b{state['b']}  measurement {i+1}/{len(recs)}  rc={rec.get('counter')}  "
            f"RSSI {rec.get('rssi_loc')}/{rec.get('rssi_ref')} dBm  "
            f"({res['n']}/{res['ntot']} tones)\n"
            f"IFFT {d_ifft:.2f} m  ->  gradient {d_grad:.2f} m  ->  "
            f"naboer {d_bay:.2f} m  ->  final(JSON) {dfin:.2f} m",
            fontsize=11)
        fig.canvas.draw_idle()

    # time slider
    recs0 = data[state["b"]]
    slider = Slider(ax_slider, "measurement", 0, len(recs0) - 1,
                    valinit=min(args.index, len(recs0) - 1), valstep=1)
    slider.on_changed(lambda v: draw(int(v)))

    # beacon selection (if several)
    radio = None
    if len(beacons) > 1:
        ax_radio = fig.add_axes([0.02, 0.45, 0.13, 0.18])
        ax_radio.set_title("beacon", fontsize=9)
        radio = RadioButtons(ax_radio, [f"b{b}" for b in beacons])

        def on_beacon(label):
            state["b"] = int(label[1:])
            n = len(data[state["b"]])
            slider.valmax = n - 1
            slider.ax.set_xlim(slider.valmin, n - 1)
            draw_time()
            slider.set_val(min(int(slider.val), n - 1))   # triggers draw
            draw(int(slider.val))
        radio.on_clicked(on_beacon)

    # arrow keys to scrub
    def on_key(event):
        if event.key in ("right", "left"):
            slider.set_val(int(slider.val) + (1 if event.key == "right" else -1))
    fig.canvas.mpl_connect("key_press_event", on_key)

    draw_time()
    draw(state["i"])

    if args.snapshot:
        fig.savefig(args.snapshot, dpi=120)
        print(f"[snapshot -> {args.snapshot}]")
        return
    plt.show()


if __name__ == "__main__":
    main()
