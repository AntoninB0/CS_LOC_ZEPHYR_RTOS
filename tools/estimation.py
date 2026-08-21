#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""estimation.py — estimation de distance (portage du pipeline MATLAB).

uart_console.py importe `estimate` et l'appelle pour CHAQUE mesure complète
(~5,5 par seconde et par beacon). `:reload` dans la console recharge ce
fichier à chaud, sans redémarrer. Développement hors ligne possible :

    python uart_console.py --file capture_ref_921600.txt

Contrat
-------
Entrée  : m, un cs_decoder.Measurement — procédure CS complète et validée :
    m.tones        liste de ToneIQ triée par canal croissant ; pour chaque t :
                     t.channel          canal CS (0..78)
                     t.freq_hz          fréquence de la tonalité (Hz)
                     t.i_loc, t.q_loc   IQ mesuré par l'initiateur (12 bits signés)
                     t.i_ref, t.q_ref   IQ mesuré par le réflecteur
                     t.tq_loc, t.tq_ref tone quality (0 = bon)
    m.rssi_loc     RSSI initiateur (dBm)   — garde-fou / pondération
    m.rssi_ref     RSSI réflecteur (dBm)
                   ⚠ 127 = « non disponible » (sentinelle BLE 0x7F), pas +127 dBm
    m.freq_offset_raw  offset de fréquence mode-0 (brut, 16 bits)
    m.rtt_pairs    paires RTT (ToA-ToD init, ToD-ToA réfl) en unités de 0,5 ns
                   (steps mode 1, ~5/mesure) — temps de vol = (Ti - Tr)/2 ;
                   sert à lever l'ambiguïté 50 m de la phase (étape 4)
    m.beacon, m.counter, m.t_ms, m.t_done_ms

Sortie  : distance estimée en mètres (float), ou None si la mesure est jugée
          inexploitable. Toute exception est encaissée par la console (erreur
          affichée une fois, mesures suivantes traitées normalement).

Pipeline (fidèle au script MATLAB d'origine) :
    IFFT (estimation globale, sans ambiguïté locale)
      -> gradient_refine (montée de vraisemblance von Mises, Bessel I0/I1)
      -> naboer_global3 (anti-lobe-secondaire : explore les alias, garde le
         meilleur score, re-raffine). Rmax = 50 m (alias c/(2·3 MHz) du
         thinning 3) ; période des lobes = c/(2·span) calculée par mesure.
"""
import math
from collections import defaultdict, deque

import numpy as np
from scipy.special import i0, i1

C = 299792458.0     # m/s

# ── Réglages ─────────────────────────────────────────────────────────────────
# delta_s MATLAB (0.01) divergeait ici : dmr ~ centaines -> pas de plusieurs
# mètres, sautant des dizaines de lobes de 6 cm. Pas réduit + borné au quart
# de lobe (vérifié sur capture_ref : converge en < 50 itérations).
DELTA_S   = 5e-5    # pas de la montée de gradient (m par unité de dérivée)
GAMMA     = 1.0     # ~SNR linéaire du modèle von Mises (1 = neutre)
R_MAX     = 50.0    # m — alias grossier c/(2·3 MHz) : au-delà, repli du spectre
MIN_TONES = 5       # en-deçà, la mesure ne vaut rien (MATLAB : 2)

# ── Calibration (à renseigner via la série au ruban — doc/CHECKLIST_TESTS.md) ─
# Offsets SOUSTRAITS à la sortie : d_affichée = d_mesurée - global - beacon.
# Réglables en live depuis la console (:cal / :cal <m> / :cal <beacon> <m>) —
# ⚠ un :reload recharge ce fichier et revient aux valeurs écrites ici.
CAL_OFFSET_M = 0.80           # biais commun (group delay) — calibré au ruban
CAL_OFFSET_PAR_BEACON = {}    # ex. {0: 0.35, 2: -0.10} — s'ajoute au global

# Distance de la DERNIÈRE mesure AVANT la médiane glissante (calibrée), lue
# par la console après chaque appel d'estimate() — sert aux tracés « bruts »
# des rapports de test. None si la mesure a été rejetée.
DERNIERE_BRUTE = None


def lobe_period(freq):
    """Pas des lobes fins de la vraisemblance : c/(2·f_porteuse) ≈ 6,15 cm.
    (Mesuré sur capture_ref : 6,1-6,2 cm — c'est la période de la porteuse
    aller-retour, PAS l'enveloppe c/(2·span) ≈ 2 m du script MATLAB.)"""
    return C / (2.0 * float(np.mean(freq)))


def calcREG_sparse(transfer2, freq_hz):
    """Distance (m) par régression de la phase déroulée vs fréquence (Hz)."""
    if len(transfer2) < 2:
        return np.nan                       # pas assez de points pour une droite
    ang = np.unwrap(np.angle(transfer2))
    m, _ = np.polyfit(freq_hz, ang, 1)      # pente en rad/Hz
    return -C * m / (4 * np.pi)


def calcIFFTDist(transfer2, ch):
    """Distance (m) au pic du profil de retard (IFFT sur la grille canal 1 MHz)."""
    H = np.zeros(79, dtype=complex)
    H[ch] = transfer2                    # canaux absents -> 0
    N = 2048
    yf = np.abs(np.fft.ifft(H, n=N)[:N // 2])
    return np.argmax(yf) * C / (2 * N * 1e6)     # 1e6 = pas de la grille, constante


# ── Estimateur bayésien (vraisemblance von Mises sur les phases) ─────────────

def find_amp(r, freq, ang1, ang2):
    """Log-vraisemblance de la distance r au vu des phases (Σ log I0).
    r peut être un scalaire ou un vecteur de candidats (vectorisé)."""
    r = np.atleast_1d(np.asarray(r, dtype=float))
    phase = np.outer(r, 2 * np.pi * freq / C) - (ang1 + ang2) / 2
    amp = np.sum(np.log(i0(2.0 * GAMMA * np.cos(phase))), axis=1)
    return float(amp[0]) if amp.size == 1 else amp


def gradient_refine(ang1, ang2, freq, r_init, delta_s=DELTA_S):
    """Montée de gradient sur la vraisemblance : affine r vers le sommet du
    lobe le plus proche de r_init (optimisation LOCALE — le choix du bon lobe
    appartient à naboer_global3). Pas borné au quart de lobe : sans cette
    borne, dmr ~ centaines -> sauts de plusieurs mètres (divergence constatée
    du portage MATLAB brut)."""
    two_pi_f_c = 2 * np.pi * freq / C
    mid = (ang1 + ang2) / 2
    max_step = lobe_period(freq) / 4

    def dmr_sum(r_now):
        phase = two_pi_f_c * r_now - mid
        arg = 2.0 * GAMMA * np.cos(phase)
        num = two_pi_f_c * (-2.0 * GAMMA) * np.sin(phase) * i1(arg)
        den = i0(arg)
        return float(np.nansum(num / den))

    r = float(r_init)
    dmr = dmr_sum(r)
    counter = 0
    while abs(dmr) >= 1e-4 and counter < 200:
        step = delta_s * dmr / GAMMA
        r += min(max(step, -max_step), max_step)
        dmr = dmr_sum(r)
        counter += 1
    return r


def naboer_global3(ang1, ang2, freq, first_est):
    """Anti-mauvais-lobe : énumère les lobes r = first_est + k·(c/2f̄) sur tout
    [0, R_MAX], score par find_amp (vectorisé), raffine les 3 meilleurs, rend
    le raffiné au meilleur score. (« naboer » = « voisins » en norvégien.)
    Diffère du MATLAB sur la grille : au PAS RÉEL des lobes (~6,15 cm), pas
    à period/5 ≈ 46 cm qui tombait entre les lobes (scores aléatoires)."""
    r_min, r_max, top_k = 0.0, R_MAX, 3
    period = lobe_period(freq)

    kmin = math.ceil((r_min - first_est) / period)
    kmax = math.floor((r_max - first_est) / period)
    if kmin > kmax:
        return min(max(first_est, r_min), r_max)

    candidates = first_est + np.arange(kmin, kmax + 1) * period
    scores = find_amp(candidates, freq, ang1, ang2)

    best_r, best_score = None, -np.inf
    for r0 in candidates[np.argsort(scores)[::-1][:top_k]]:
        r_ref = gradient_refine(ang1, ang2, freq, r0)
        if not np.isfinite(r_ref):
            continue
        r_ref = min(max(r_ref, r_min), r_max)
        score = find_amp(r_ref, freq, ang1, ang2)
        if score > best_score:
            best_score, best_r = score, r_ref
    return best_r if best_r is not None else first_est


# ── Point d'entrée appelé par la console ─────────────────────────────────────

def estimate(m):
    """Estime la distance (m) à partir d'une mesure CS complète."""
    global DERNIERE_BRUTE
    DERNIERE_BRUTE = None                    # pas de valeur périmée si rejet
    ch  = np.array([t.channel for t in m.tones])
    Y_I = np.array([complex(t.i_loc, t.q_loc) for t in m.tones])   # ≡ l_i + 1i*l_q
    Y_R = np.array([complex(t.i_ref, t.q_ref) for t in m.tones])   # ≡ p_i + 1i*p_q
    f   = np.array([t.freq_hz for t in m.tones])

    transfer2 = Y_I * Y_R
    ang1 = -np.angle(Y_I)
    ang2 = -np.angle(Y_R)

    tq  = np.array([t.tq_loc for t in m.tones])
    tqr = np.array([t.tq_ref for t in m.tones])

    valid = (tq == 0) & (tqr == 0) & (np.abs(Y_I) > 0) & (np.abs(Y_R) > 0)
    if np.count_nonzero(valid) < MIN_TONES:
        return None                          # mesure inexploitable

    ch_v   = ch[valid]
    f_v    = f[valid]
    t2_v   = transfer2[valid]
    ang1_v = ang1[valid]
    ang2_v = ang2[valid]

    # 1. Estimation globale grossière (résolution ~2 m, pas d'unwrap)
    d_ifft = calcIFFTDist(t2_v, ch_v)

    # 2. Raffinement local par la vraisemblance des phases
    d_bay = gradient_refine(ang1_v, ang2_v, f_v, d_ifft)

    # 3. Correction de lobe (l'IFFT accroche parfois un trajet réfléchi :
    #    naboer re-cherche le lobe globalement le plus vraisemblable)
    d_bay = naboer_global3(ang1_v, ang2_v, f_v, d_bay)

    if not np.isfinite(d_bay):
        return None

    # 4. Levée d'ambiguïté par RTT (steps mode 1) : la phase est précise mais
    #    modulo 50 m (repliement du thinning 3) ; le RTT est grossier (~±3 m
    #    LOS, pire en NLOS) mais absolu -> il choisit la tranche de 50 m.
    #    Correction nulle tant que |d_rtt - d_bay| < 25 m (même tranche).
    #    Validé banc : beacon à ~58 m, phase repliée 8,6 m -> fusion 58,6 m.
    if m.rtt_pairs:
        d_rtt = float(np.median([(ti - tr) * 0.25e-9 * C
                                 for ti, tr in m.rtt_pairs]))
        d_bay += 50.0 * round((d_rtt - d_bay) / 50.0)
        d_bay = max(d_bay, 0.0)   # échappée de lobe recalée -> peut être < 0

    # 5. Calibration : offsets mesurés au ruban, soustraits à la sortie.
    d_bay -= CAL_OFFSET_M + CAL_OFFSET_PAR_BEACON.get(m.beacon, 0.0)
    d_bay = max(d_bay, 0.0)
    DERNIERE_BRUTE = float(d_bay)

    # 6. Médiane glissante PAR BEACON (≡ movmedian(7) du MATLAB) : écrase les
    #    sauts isolés de lobe/multipath. Latence ~0,6 s à 5,5 Hz. État module,
    #    remis à zéro par :reload.
    hist = _median_hist[m.beacon]
    hist.append(float(d_bay))
    return float(np.median(hist))
    # Pour comparer les estimateurs : return calcREG_sparse(t2_v, f_v),
    # d_ifft ou d_bay brut — la même mesure, plusieurs lectures.


_median_hist = defaultdict(lambda: deque(maxlen=7))
