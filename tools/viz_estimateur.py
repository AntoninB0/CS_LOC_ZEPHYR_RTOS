#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_estimateur.py — figures diagnostiques de l'estimateur pour ILLUSTRER
l'incertitude et le multipath, à partir des mêmes données que la chaîne live.

Pour une (ou plusieurs) mesure CS, trace trois panneaux :
  1. Profil de retard IFFT |IFFT(H)| vs distance — chaque PIC = un trajet
     (direct + réflexions). Le multipath se voit directement : plusieurs pics,
     et parfois une réflexion plus haute que le direct (source du biais).
  2. Vraisemblance bayésienne par lobe d'alias — score find_amp() à chaque lobe
     candidat sur [0, R_MAX]. Montre l'AMBIGUÏTÉ : le lobe gagnant et ses
     poursuivants ; à faible SNR le mauvais lobe peut passer devant.
  3. Zoom sur le lobe retenu — largeur du pic = incertitude de phase (~cm).

Ce script NE MODIFIE PAS estimation.py : il réutilise ses fonctions publiques
(find_amp, calcIFFTDist, gradient_refine, naboer_global3, lobe_period) et le
décodeur cs_decoder, exactement comme uart_console. Les distances marquées sont
donc celles de TON estimateur (avant RTT/calibration/médiane, non tracés).

Usage :
  python viz_estimateur.py --file capture_ref_921600.txt         # 1 fig/beacon (mesure médiane)
  python viz_estimateur.py --json test_XXXX/test_XXXX.json       # depuis un test enrichi (champ iq)
  python viz_estimateur.py --file capture.txt --beacon 0 --counter 42
  python viz_estimateur.py --file capture.txt --worst            # la mesure la plus aberrante/beacon
  python viz_estimateur.py --file capture.txt --index 10         # la 10e mesure valide
  python viz_estimateur.py --port /dev/ttyACM0 --collect 40      # capture live puis figures

⚠ --json ne marche que sur les tests ENREGISTRÉS AVEC IQ (champ "iq", ajouté
récemment à uart_console). Les tests plus anciens n'ont que les distances.

Sorties : dossier horodaté viz_AAAAMMJJ_HHMMSS/ avec un PNG par mesure tracée.
Dépendances : numpy, scipy, matplotlib (venv tools : voir install.txt).
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
    matplotlib.use("Agg")                 # rendu fichier, sans écran
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib absent. Active le venv : cd tools && source .venv/bin/activate "
             "&& pip install -r install.txt")

try:
    from scipy.signal import find_peaks
except ImportError:
    find_peaks = None


# ── Reprise fidèle de la préparation d'entrée de estimation.estimate() ────────
# (mêmes lignes que estimate() : si tu changes le filtre de tones là-bas,
#  reporte-le ici pour que les figures collent au live.)
def prepare(m):
    """Mesure -> tones valides (ch, f, transfer2, ang1, ang2) ou None."""
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
    """Profil de retard : mirror de estimation.calcIFFTDist, mais renvoie la
    COURBE complète (x en mètres, |IFFT|) et l'argmax (= d_ifft de l'estimateur)."""
    H = np.zeros(79, dtype=complex)
    H[ch] = t2
    yf = np.abs(np.fft.ifft(H, n=n)[:n // 2])
    x = np.arange(n // 2) * est.C / (2 * n * 1e6)     # 1e6 = pas de grille (1 MHz)
    return x, yf, x[int(np.argmax(yf))]


def bayes_candidates(first_est, f, ang1, ang2):
    """Comb de lobes candidats de naboer_global3 sur [0, R_MAX] et leur score."""
    period = est.lobe_period(f)
    kmin = math.ceil((0.0 - first_est) / period)
    kmax = math.floor((est.R_MAX - first_est) / period)
    if kmin > kmax:
        return np.array([first_est]), est.find_amp(np.array([first_est]), f, ang1, ang2), period
    cand = first_est + np.arange(kmin, kmax + 1) * period
    scores = est.find_amp(cand, f, ang1, ang2)
    return cand, np.atleast_1d(scores), period


def plot_measurement(m, outpath, rmax_view=40.0):
    """Trace les 3 panneaux pour la mesure m. Renvoie (d_ifft, d_bay) ou None."""
    p = prepare(m)
    if p is None:
        return None
    ch, f, t2, ang1, ang2 = p["ch"], p["f"], p["t2"], p["ang1"], p["ang2"]

    # Pipeline de TON estimateur (étapes 1-3), pour marquer les mêmes distances.
    x, yf, d_ifft = ifft_profile(ch, t2)
    d_first = est.gradient_refine(ang1, ang2, f, d_ifft)      # first_est de naboer
    d_bay = est.naboer_global3(ang1, ang2, f, d_first)
    cand, scores, period = bayes_candidates(d_first, f, ang1, ang2)

    fig, ax = plt.subplots(3, 1, figsize=(10, 9))

    # 1) Profil de retard IFFT — multipath = pics multiples
    ax[0].plot(x, yf, lw=1.1)
    ax[0].axvline(d_ifft, color="tab:red", ls="--", lw=1.2,
                  label=f"argmax IFFT = {d_ifft:.2f} m")
    ax[0].axvline(d_bay, color="tab:green", ls="-", lw=1.2,
                  label=f"bayésien = {d_bay:.2f} m")
    if find_peaks is not None and yf.max() > 0:
        pk, _ = find_peaks(yf, height=0.25 * yf.max(), distance=8)
        for i in pk[:6]:
            ax[0].annotate(f"{x[i]:.1f} m", (x[i], yf[i]), fontsize=7,
                           textcoords="offset points", xytext=(0, 3),
                           ha="center", color="dimgray")
    # Vue élargie si besoin pour que l'argmax IFFT (souvent une réflexion
    # lointaine que le bayésien corrige) reste visible dans le cadre.
    view = min(max(rmax_view, 1.15 * max(d_ifft, d_bay)), float(x.max()))
    ax[0].set_xlim(0, view)
    ax[0].set_xlabel("distance (m)"); ax[0].set_ylabel("|IFFT|")
    ax[0].set_title("Profil de retard (IFFT) — chaque pic = un trajet "
                    "(direct + multipath)")
    ax[0].legend(fontsize=8); ax[0].grid(True, alpha=0.3)

    # 2) Vraisemblance par lobe d'alias — ambiguïté
    ax[1].vlines(cand, scores.min(), scores, color="tab:blue", alpha=0.35, lw=0.8)
    ax[1].plot(cand, scores, ".", ms=4, color="tab:blue")
    i_best = int(np.argmax(scores))
    ax[1].plot(cand[i_best], scores[i_best], "o", ms=9, mfc="none",
               mec="tab:red", mew=1.6, label=f"lobe gagnant ≈ {cand[i_best]:.2f} m")
    ax[1].axvline(d_bay, color="tab:green", ls="-", lw=1.0)
    ax[1].set_xlim(0, est.R_MAX)
    ax[1].set_xlabel("distance candidate (m)"); ax[1].set_ylabel("log-vraisemblance")
    ax[1].set_title(f"Vraisemblance par lobe d'alias (pas {period*100:.1f} cm) — "
                    "hauteur = plausibilité ; écart gagnant/2e = marge d'ambiguïté")
    ax[1].legend(fontsize=8); ax[1].grid(True, alpha=0.3)

    # 3) Zoom lobe retenu — incertitude de phase
    rr = np.linspace(d_bay - 5 * period, d_bay + 5 * period, 800)
    sc = est.find_amp(rr, f, ang1, ang2)
    ax[2].plot(rr, sc, lw=1.1)
    ax[2].axvline(d_bay, color="tab:green", ls="-", lw=1.2,
                  label=f"bayésien = {d_bay:.3f} m")
    ax[2].set_xlabel("distance (m)"); ax[2].set_ylabel("log-vraisemblance")
    ax[2].set_title(f"Zoom sur le lobe retenu — largeur ≈ incertitude de phase "
                    f"(pas de lobe {period*100:.1f} cm)")
    ax[2].legend(fontsize=8); ax[2].grid(True, alpha=0.3)

    fig.suptitle(f"b{m.beacon}  rc={m.counter}  |  {p['n_valid']}/{p['n_tones']} tones valides  "
                 f"|  RSSI {m.rssi_loc}/{m.rssi_ref} dBm  |  "
                 f"IFFT={d_ifft:.2f} m  bayésien={d_bay:.2f} m", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(outpath, dpi=120)
    plt.close(fig)
    return d_ifft, d_bay


# ── Collecte des mesures (fichier ou live) ───────────────────────────────────

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


# ── Reconstruction depuis un JSON de test enrichi (champ "iq" par mesure) ────
# Le JSON de --test stocke, par mesure, iq = [[canal, i_loc, q_loc, i_ref,
# q_ref, tq_loc, tq_ref], ...]. On rebâtit des objets minimalistes compatibles
# avec prepare()/plot_measurement() (mêmes attributs que cs_decoder.ToneIQ /
# Measurement, hors RTT non nécessaire aux figures).
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
    """Construit un _Meas depuis un enregistrement JSON (doit contenir 'iq')."""
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
        sys.exit(f"{path} : aucune mesure avec IQ (champ 'iq'). "
                 f"{'Ce test est antérieur au stockage des IQ.' if n_no_iq else ''} "
                 "Régénère depuis un --test récent ou une capture brute (--file).")
    return out


def measurements_from_port(port, baud, secs, beacon=None):
    import serial
    stats = DropStats()
    pairer = Pairer(stats)
    out = []
    deadline = time.time() + secs
    buf = b""
    with serial.Serial(port, baud, timeout=0.2) as ser:
        print(f"[capture {secs} s sur {port} @ {baud}]", file=sys.stderr)
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


# ── Sélection des mesures à tracer ───────────────────────────────────────────

def select(meas, args):
    """Renvoie la liste des mesures à tracer selon les options."""
    if args.counter is not None:
        sel = [m for m in meas if m.counter == args.counter
               and (args.beacon is None or m.beacon == args.beacon)]
        return sel or []
    if args.index is not None:
        return [meas[args.index]] if 0 <= args.index < len(meas) else []

    # Par défaut / --worst : une mesure REPRÉSENTATIVE par beacon. On estime
    # d_bay de chaque mesure (via l'estimateur) puis on prend la médiane (cas
    # typique) ou le plus grand écart à cette médiane (--worst, cas multipath).
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
        # typique = mesure la plus PROCHE de la médiane ; --worst = la plus loin
        dist_to_med = lambda dm: abs(dm[0] - med)      # noqa: E731
        pick = max(lst, key=dist_to_med) if args.worst else min(lst, key=dist_to_med)
        chosen.append(pick[1])
    return chosen


def main():
    ap = argparse.ArgumentParser(
        description="Figures IFFT + bayésien (incertitude / multipath)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="capture brute (lignes IQL/IQP)")
    src.add_argument("--json", help="JSON de test enrichi (champ 'iq' par mesure)")
    src.add_argument("--port", help="capture live puis figures")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--collect", type=int, default=30, help="durée capture live (s)")
    ap.add_argument("--beacon", type=int, help="ne garder qu'un beacon")
    ap.add_argument("--counter", type=int, help="tracer une procédure précise")
    ap.add_argument("--index", type=int, help="tracer la k-ème mesure valide")
    ap.add_argument("--worst", action="store_true",
                    help="tracer la mesure la plus aberrante par beacon (cas multipath)")
    ap.add_argument("--rmax-view", type=float, default=40.0,
                    help="distance max affichée dans le profil IFFT (m)")
    ap.add_argument("--out", help="dossier de sortie (défaut viz_horodaté)")
    args = ap.parse_args()

    if args.file:
        meas = measurements_from_file(args.file, args.beacon)
    elif args.json:
        meas = measurements_from_json(args.json, args.beacon)
    else:
        meas = measurements_from_port(args.port, args.baud, args.collect, args.beacon)
    if not meas:
        sys.exit("Aucune mesure complète dans l'entrée.")

    sel = select(meas, args)
    if not sel:
        sys.exit("Aucune mesure ne correspond à la sélection.")

    outdir = args.out or time.strftime("viz_%Y%m%d_%H%M%S")
    os.makedirs(outdir, exist_ok=True)
    for m in sel:
        png = os.path.join(outdir, f"b{m.beacon}_rc{m.counter}.png")
        res = plot_measurement(m, png, args.rmax_view)
        if res is None:
            print(f"[b{m.beacon} rc{m.counter} : trop peu de tones valides, ignorée]",
                  file=sys.stderr)
            continue
        d_ifft, d_bay = res
        print(f"b{m.beacon} rc{m.counter}: IFFT={d_ifft:.2f} m  bayésien={d_bay:.2f} m  -> {png}")
    print(f"[figures -> {outdir}/]")


if __name__ == "__main__":
    main()
