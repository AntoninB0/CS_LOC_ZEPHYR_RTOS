#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
iq_estimation.py — estimation de distance à partir des captures IQ (PARNAV).

Portage Python du pipeline MATLAB (linreg / IFFT / bayésien gradient+alias),
alimenté par les lignes IQL/IQP du firmware (JSON de cs_console, ou live).

DONNÉES — deux moitiés à apparier par (beacon, ranging_counter) :
  - IQL (local_subevent) : steps HCI parsés par cs_console. Step mode-2 :
      paths[0] = l'antenna path RÉEL (I/Q local, Y_I)
      paths[1] = le tone extension slot (PAS le remote !)
  - IQP (peer_procedure) : ranging data RAS brut (raw_hex), décodé ICI.
      Format vérifié sur données réelles : ranging header 4 o
      (counter 12b + config 4b, tx_power, antenna_paths_mask) puis par
      subevent : header 8 o (start_acl_evt u16, freq_comp u16, statuts 2 o,
      ref_power i8, num_steps u8) puis num_steps steps {mode(1) + data} :
      mode0 réflecteur = 3 o (quality, rssi, antenna),
      mode1 réflecteur = MODE1_PEER_LEN o (RTT AA-only),
      mode2 = 1 + (n_paths+1)×4 o (perm + [PCT(3)+quality(1)]…).
      Les steps du pair n'ont PAS de canal : alignement avec les steps
      locaux mode-2 par ORDRE (même procédure → même séquence de canaux).

Usage batch :   python iq_estimation.py iq_capture.json [--plot] [--beacon N]
Usage live  :   est = IQEstimator(); res = est.feed(record)  # record = dict
                # res = None tant que la paire est incomplète, sinon
                # {"beacon","counter","t_board_ms","linreg","ifft","bayesian"}
Dépendances :   pip install numpy scipy   (matplotlib pour --plot)
"""

import json
import sys
import numpy as np
from scipy.special import i0 as bessel_i0, i1 as bessel_i1

# ── constantes (mêmes conventions que le MATLAB) ───────────────────────────
C        = 299792458.0
DELTA_F  = 1e6          # espacement canal CS (Hz) ; freq = canal × 1 MHz
PERIOD   = 2.3          # période d'alias (m) du score bayésien
DELTA_S  = 0.01         # pas du gradient
JUMP_THRESH = 18.0      # filtre de sauts (m), post-traitement batch
MED_WIN  = 7            # fenêtre de médiane glissante, batch

MODE1_PEER_LEN = 6      # octets de data d'un step mode-1 réflecteur (observé
                        # sur données réelles ; ajuster si le parse dérive)


# ════════════════════════════ parseur RAS pair ═════════════════════════════

def parse_peer_ras(hexstr):
    """Décode le raw_hex d'une ligne IQP → (header dict, liste de steps).
    Steps mode-2 : {"mode":2, "paths":[{"i","q","tone_quality"}...]}."""
    data = bytes.fromhex(hexstr)
    if len(data) < 12:
        raise ValueError("IQP trop court")

    counter_cfg = int.from_bytes(data[0:2], "little")
    header = {
        "ranging_counter": counter_cfg & 0x0FFF,
        "config_id": counter_cfg >> 12,
        "tx_power": int.from_bytes(data[2:3], "little", signed=True),
        "antenna_paths_mask": data[3],
    }
    n_paths = bin(header["antenna_paths_mask"]).count("1") or 1
    mode2_len = 1 + (n_paths + 1) * 4

    steps, pos = [], 4
    while pos + 8 <= len(data):
        num_steps = data[pos + 7]
        ref_power = int.from_bytes(data[pos + 6:pos + 7], "little", signed=True)
        pos += 8
        for _ in range(num_steps):
            if pos >= len(data):
                break
            mode = data[pos] & 0x7F
            aborted = bool(data[pos] & 0x80)
            pos += 1
            if mode == 0:
                dlen = 3
                step = {"mode": 0, "rssi":
                        int.from_bytes(data[pos + 1:pos + 2], "little",
                                       signed=True)}
            elif mode == 1:
                dlen = MODE1_PEER_LEN
                # Layout observé sur trames réelles : quality(1), NADM(1,
                # 0xFF = indispo en AA-only), RSSI(1, i8), ToA_ToD(2, i16 LE,
                # unités de 0,5 ns, 0x8000 = indispo), antenna(1).
                toa = int.from_bytes(data[pos + 3:pos + 5], "little",
                                     signed=True)
                nadm = data[pos + 1]
                step = {"mode": 1,
                        "quality": data[pos],
                        "nadm": None if nadm == 0xFF else nadm,
                        "rssi": int.from_bytes(data[pos + 2:pos + 3],
                                               "little", signed=True),
                        "toa_tod_half_ns": None if toa == -32768 else toa}
            elif mode == 2:
                dlen = mode2_len
                paths = []
                for p in range(n_paths + 1):
                    off = pos + 1 + 4 * p
                    pct = int.from_bytes(data[off:off + 3], "little")
                    i12 = pct & 0xFFF
                    q12 = (pct >> 12) & 0xFFF
                    paths.append({
                        "i": i12 - 4096 if i12 & 0x800 else i12,
                        "q": q12 - 4096 if q12 & 0x800 else q12,
                        "tone_quality": data[off + 3],
                    })
                step = {"mode": 2, "paths": paths,
                        "antenna_permutation": data[pos]}
            else:
                raise ValueError(f"mode step inconnu {mode} @ {pos}")
            if pos + dlen > len(data):
                break
            step["aborted"] = aborted
            step["ref_power"] = ref_power
            steps.append(step)
            pos += dlen
    return header, steps


# ══════════════════ estimateurs (portage fidèle du MATLAB) ═════════════════

def calc_reg(transfer, channels):
    """Régression linéaire de la phase déroulée vs index de canal.
    d = -(c·pente)/(4π·Δf).  == calcREG_sparse"""
    ang = np.angle(transfer)
    valid = (ang != 0) & np.isfinite(ang)
    if np.count_nonzero(valid) < 2:
        return float("nan")
    x = np.asarray(channels, float)[valid]
    a = ang[valid]
    order = np.argsort(x)
    x, a = x[order], np.unwrap(a[order])
    m, _ = np.polyfit(x, a, 1)
    return -(C * m) / (4 * np.pi * DELTA_F)


def calc_ifft_dist(transfer, channels, nfft=2048):
    """Pic de l'IFFT du transfert POSÉ SUR LA GRILLE de canaux (les canaux
    thinned laissent des zéros — équivalent du vecteur dense du .mat).
    Axe : x = idx/(2·N·Δf)·c (aller-retour).  == calcIFFTDist"""
    grid = np.zeros(int(max(channels)) + 1, dtype=complex)
    grid[np.asarray(channels, int)] = transfer
    y = np.abs(np.fft.ifft(grid, nfft)[:nfft // 2])
    x_m = np.arange(nfft // 2) / (2.0 * nfft * DELTA_F) * C
    return float(x_m[int(np.argmax(y))])


def _dmr_sum(r, freqs, half_sum, gamma):
    """gamma : scalaire OU vecteur par tone (SNR effectif — le MATLAB fixait
    γ=1 partout ; ici γ_k ∝ qualité×amplitude, les tones fadés pèsent moins)."""
    phase = (2 * np.pi * freqs / C) * r - half_sum
    arg = 2 * gamma * np.cos(phase)
    num = (2 * np.pi * freqs / C) * (-2 * gamma) * np.sin(phase) * bessel_i1(arg)
    with np.errstate(divide="ignore", invalid="ignore"):
        d = num / bessel_i0(arg)
    return float(np.nansum(d))


def gradient_refine(ang1, ang2, freqs, r_init, delta_s=DELTA_S, gamma=1.0):
    """Descente sur la vraisemblance Rice/von-Mises.  == gradient_refine,
    généralisé à un γ par tone (poids)."""
    half_sum = (np.asarray(ang1) + np.asarray(ang2)) / 2.0
    freqs = np.asarray(freqs, float)
    gamma = np.asarray(gamma, float)
    gnorm = float(np.mean(gamma))  # pas d'itération ~ celui du MATLAB
    r = float(r_init)
    dmr = _dmr_sum(r, freqs, half_sum, gamma)
    n = 0
    while abs(dmr) >= 1e-4 and n < 100:
        r += delta_s * dmr / gnorm
        dmr = _dmr_sum(r, freqs, half_sum, gamma)
        n += 1
    return r


def find_amp(r, freqs, ang1, ang2, gamma=1.0):
    """Score Σ log I₀(2γ·cos(·)) — γ par tone possible.  == find_amp"""
    arg = 2 * np.asarray(gamma, float) * np.cos(
        (2 * np.pi * np.asarray(freqs, float) * r / C)
        - (np.asarray(ang1) + np.asarray(ang2)) / 2.0)
    return float(np.nansum(np.log(bessel_i0(arg))))


def naboer_global3(ang1, ang2, freqs, first_est, gamma=1.0,
                   period=PERIOD, r_min=0.0, r_max=150.0, top=3):
    """Recherche globale d'alias [0,150] m (pas period/5), raffine le top-3,
    retourne le meilleur raffiné.  == naboer_global3"""
    step = period / 5.0
    kmin = int(np.ceil((r_min - first_est) / step))
    kmax = int(np.floor((r_max - first_est) / step))
    if kmin > kmax:
        return float(np.clip(first_est, r_min, r_max))
    cand = first_est + np.arange(kmin, kmax + 1) * step
    cand = np.unique(cand[(cand >= r_min) & (cand <= r_max)])
    if cand.size == 0:
        return float(np.clip(first_est, r_min, r_max))
    scores = np.array([find_amp(r, freqs, ang1, ang2, gamma) for r in cand])
    best_r, best_s = cand[int(np.argmax(scores))], -np.inf
    for r0 in cand[np.argsort(scores)[::-1][:top]]:
        r_ref = gradient_refine(ang1, ang2, freqs, r0, gamma=gamma)
        if not np.isfinite(r_ref):
            continue
        r_ref = float(np.clip(r_ref, r_min, r_max))
        s = find_amp(r_ref, freqs, ang1, ang2, gamma)
        if s > best_s:
            best_s, best_r = s, r_ref
    return float(best_r)


# ═══════════════════════ appariement local/pair + pipeline ═════════════════

def _local_mode1_tod_toa(step):
    """ToD_ToA d'un step mode-1 LOCAL (initiateur) depuis son champ raw.
    Layout HCI initiateur : quality(1), NADM(1), RSSI(1), ToD_ToA(2, i16 LE,
    0,5 ns, 0x8000 indispo), antenna(1)[, ...]. Retourne None si indispo."""
    try:
        d = bytes.fromhex(step.get("raw", ""))
        if len(d) < 5:
            return None
        v = int.from_bytes(d[3:5], "little", signed=True)
        return None if v == -32768 else v
    except ValueError:
        return None


def _local_mode0_freq_offset_ppm(step):
    """Measured_Freq_Offset d'un step mode-0 LOCAL : i16 LE en unités de
    0,01 ppm (0xC000 = indispo), octets 3-4 du data initiateur."""
    try:
        d = bytes.fromhex(step.get("raw", ""))
        if len(d) < 5:
            return None
        v = int.from_bytes(d[3:5], "little", signed=True)
        return None if v == -16384 else v * 0.01
    except ValueError:
        return None


def rtt_distance(local_steps, peer_steps):
    """Distance RTT (m) depuis les steps mode-1 des DEUX côtés :
    ToF = (ToD_ToA_initiateur + ToA_ToD_réflecteur)/2 × 0,5 ns ; d = ToF·c.
    Médiane sur les paires alignées par ordre. None si indisponible.
    Grossier (σ ~ mètres) mais NON-AMBIGU → sert d'arbitre d'alias."""
    tod = [_local_mode1_tod_toa(s) for s in local_steps
           if s.get("mode") == 1]
    toa = [s.get("toa_tod_half_ns") for s in peer_steps
           if s.get("mode") == 1]
    pairs = [(a, b) for a, b in zip(tod, toa)
             if a is not None and b is not None]
    if len(pairs) < 2:
        return None
    tof_s = np.median([(a + b) / 2.0 for a, b in pairs]) * 0.5e-9
    d = tof_s * C
    return float(d) if 0.0 <= d <= 500.0 else None


def _tones_from_pair(local_steps, peer_steps):
    """Aligne les mode-2 local/pair par ordre → (channels, Y_I, Y_R)."""
    loc2 = [s for s in local_steps if s.get("mode") == 2 and s.get("paths")]
    pee2 = [s for s in peer_steps if s.get("mode") == 2 and s.get("paths")]
    n = min(len(loc2), len(pee2))
    QUAL_W = {0: 1.0, 1: 0.7, 2: 0.4, 3: 0.0}  # nibble bas du tone_quality
    ch, yi, yr, w = [], [], [], []
    for k in range(n):
        pl, pp = loc2[k]["paths"][0], pee2[k]["paths"][0]
        y_i = complex(pl["i"], pl["q"])
        y_r = complex(pp["i"], pp["q"])
        wq = (QUAL_W.get(pl.get("tone_quality", 0) & 0x0F, 1.0) *
              QUAL_W.get(pp.get("tone_quality", 0) & 0x0F, 1.0))
        if abs(y_i) == 0 or abs(y_r) == 0 or wq == 0.0:
            continue  # tone invalide ou marqué inutilisable
        ch.append(loc2[k]["channel"])
        yi.append(y_i)
        yr.append(y_r)
        w.append(wq)
    ch, yi, yr, w = (np.array(ch, float), np.array(yi),
                     np.array(yr), np.array(w, float))
    if ch.size:
        # Pondération par l'AMPLITUDE du produit (∝ SNR du tone) : le fading
        # multipath creuse certains canaux, leur phase est bruitée. Normalisé
        # à la médiane, borné pour qu'aucun tone ne domine ni ne disparaisse.
        amp = np.abs(yi * yr)
        med = np.median(amp[amp > 0]) or 1.0
        w = np.clip(w * amp / med, 0.15, 3.0)
    return ch, yi, yr, w


def estimate_pair(local_steps, peer_steps):
    """→ dict {linreg, ifft, bayesian, alias_m, n_tones} ou None si < 2 tones."""
    ch, y_i, y_r, w = _tones_from_pair(local_steps, peer_steps)
    if ch.size < 2:
        return None
    transfer = y_i * y_r                   # produit → somme des phases (2 voies)
    freqs = ch * DELTA_F                   # convention MATLAB : canal × 1 MHz
    ang1, ang2 = -np.angle(y_i), -np.angle(y_r)

    # Portée NON-AMBIGUË réelle : c / (2·Δf_effectif), avec Δf_effectif =
    # pgcd des écarts de canaux × 1 MHz. Le MATLAB cherchait sur [0,150] m
    # car son dataset avait des tones espacés de 1 MHz ; avec le channel
    # thinning (ex. 3 → 50 m), la métrique bayésienne a des pics IDENTIQUES
    # à chaque période d'alias : chercher au-delà fait gagner un alias au
    # bruit (symptôme : d + 50 m). On borne donc la recherche à une période.
    spacing = int(np.gcd.reduce(np.diff(np.sort(ch)).astype(int))) or 1
    alias_m = C / (2.0 * spacing * DELTA_F)

    d_ifft = calc_ifft_dist(transfer, ch)
    d_seed = d_ifft % alias_m              # replier la graine dans la période
    d_bay = gradient_refine(ang1, ang2, freqs, d_seed, gamma=w)
    d_bay = naboer_global3(ang1, ang2, freqs, d_bay, gamma=w,
                           r_max=min(150.0, alias_m * 0.999))

    # RTT mode-1 : grossier (σ métrique) mais NON-AMBIGU → il désigne le bon
    # repli de la solution PBR fine quand la vraie distance dépasse alias_m.
    d_rtt = rtt_distance(local_steps, peer_steps)
    d_final = d_bay
    if d_rtt is not None:
        k = round((d_rtt - d_bay) / alias_m)
        cand = d_bay + k * alias_m
        if 0.0 <= cand <= 500.0:
            d_final = float(cand)

    # NADM (détection d'altération, dispo avec la sounding sequence) :
    # 0 = altération très improbable … 6 = très probable ; None en AA_ONLY.
    nadms = [st["nadm"] for st in peer_steps
             if st.get("mode") == 1 and st.get("nadm") is not None]
    nadm_med = float(np.median(nadms)) if nadms else None

    # freq_offset des steps mode-0 (0,01 ppm) : dérive d'horloge init↔refl,
    # exposé pour le modèle d'horloge PARNAV (pas appliqué aux phases ici).
    ppm = [v for v in (_local_mode0_freq_offset_ppm(st)
                       for st in local_steps if st.get("mode") == 0)
           if v is not None]

    return {"linreg": calc_reg(transfer, ch), "ifft": d_ifft,
            "bayesian": d_final, "bayesian_wrapped": d_bay,
            "rtt": d_rtt, "nadm": nadm_med, "freq_offset_ppm":
            (float(np.mean(ppm)) if ppm else None),
            "alias_m": alias_m, "n_tones": int(ch.size)}


class IQEstimator:
    """Mode LIVE : nourrir feed() avec chaque record (dict au format du JSON
    de cs_console). Retourne l'estimation dès que la paire IQL/IQP d'un
    (beacon, counter) est complète, sinon None."""

    def __init__(self, max_pending=32):
        self.pending = {}
        self.max_pending = max_pending

    def feed(self, rec):
        key = (rec.get("beacon"), rec.get("ranging_counter"))
        slot = self.pending.setdefault(key, {})
        if rec.get("type") == "local_subevent":
            slot.setdefault("local", []).extend(rec.get("steps") or [])
            slot["t"] = rec.get("t_board_ms", 0)
        elif rec.get("type") == "peer_procedure":
            try:
                _, slot["peer"] = parse_peer_ras(rec["raw_hex"])
            except (ValueError, KeyError):
                self.pending.pop(key, None)
                return None
        if "local" in slot and "peer" in slot:
            self.pending.pop(key, None)
            est = estimate_pair(slot["local"], slot["peer"])
            if est:
                est.update(beacon=key[0], counter=key[1],
                           t_board_ms=slot.get("t", 0))
            return est
        if len(self.pending) > self.max_pending:  # purge des moitiés orphelines
            self.pending.pop(next(iter(self.pending)))
        return None


# ═══════════════════════ post-traitement batch (MATLAB) ════════════════════

def jump_filter(d, thresh=JUMP_THRESH):
    d = np.asarray(d, float).copy()
    j = np.abs(np.diff(d))
    j[~(np.isfinite(d[:-1]) & np.isfinite(d[1:]))] = 0
    d[np.flatnonzero(j > thresh) + 1] = np.nan
    return d


def movmedian(d, win=MED_WIN):
    d = np.asarray(d, float)
    out = np.full_like(d, np.nan)
    h = win // 2
    for k in range(d.size):
        w = d[max(0, k - h):k + h + 1]
        w = w[np.isfinite(w)]
        if w.size:
            out[k] = np.median(w)
    out[~np.isfinite(d)] = np.nan
    return out


def process_json(path):
    """Batch : lit le JSON de cs_console → {beacon: dict de séries numpy}."""
    with open(path, "r", encoding="utf-8") as f:
        records = json.load(f)["records"]
    est, series = IQEstimator(max_pending=10 ** 9), {}
    for rec in records:
        r = est.feed(rec)
        if r:
            s = series.setdefault(r["beacon"], {"t": [], "linreg": [],
                                                "ifft": [], "bayesian": [],
                                                "rtt": [],
                                                "freq_offset_ppm": []})
            s["t"].append(r["t_board_ms"])
            for k in ("linreg", "ifft", "bayesian", "rtt",
                      "freq_offset_ppm"):
                v = r.get(k)
                s[k].append(np.nan if v is None else v)
    for b, s in series.items():
        for k in s:
            s[k] = np.array(s[k], float)
        s["bayesian_filt"] = jump_filter(s["bayesian"])
        s["bayesian_med"] = movmedian(s["bayesian_filt"])
    return series


def main():
    import argparse
    p = argparse.ArgumentParser(description="Estimation IQ depuis un JSON cs_console")
    p.add_argument("json_file")
    p.add_argument("--beacon", type=int, default=None)
    p.add_argument("--plot", action="store_true")
    a = p.parse_args()

    series = process_json(a.json_file)
    if not series:
        print("Aucune paire IQL/IQP complète dans ce fichier — vérifier que "
              "les lignes IQL sont présentes (firmware >= v3.4, IQON actif).")
        sys.exit(1)
    for b, s in sorted(series.items()):
        if a.beacon is not None and b != a.beacon:
            continue
        n = s["bayesian"].size
        med = np.nanmedian(s["bayesian_med"])
        rtt_med = np.nanmedian(s["rtt"]) if np.isfinite(s["rtt"]).any() else float("nan")
        ppm_med = np.nanmedian(s["freq_offset_ppm"]) if np.isfinite(s["freq_offset_ppm"]).any() else float("nan")
        print(f"beacon {b}: {n} paires | bayésien médian = {med:.2f} m | "
              f"ifft médian = {np.nanmedian(s['ifft']):.2f} m | "
              f"linreg médian = {np.nanmedian(s['linreg']):.2f} m | "
              f"rtt médian = {rtt_med:.2f} m | dérive = {ppm_med:.2f} ppm")
    if a.plot:
        import matplotlib.pyplot as plt
        for b, s in sorted(series.items()):
            if a.beacon is not None and b != a.beacon:
                continue
            t = (s["t"] - s["t"][0]) / 1000.0 if s["t"].size else s["t"]
            plt.scatter(t, s["ifft"], s=8, label=f"b{b} IFFT")
            plt.scatter(t, s["bayesian"], s=8, label=f"b{b} bayésien")
            plt.plot(t, s["bayesian_med"], label=f"b{b} bayésien (médiane {MED_WIN})")
        plt.xlabel("t carte (s)"); plt.ylabel("distance (m)")
        plt.grid(True); plt.legend(); plt.title("Estimations IQ")
        plt.show()


if __name__ == "__main__":
    main()
