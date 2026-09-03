#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""estimation.py — distance estimation (port of the MATLAB pipeline).

uart_console.py imports `estimate` and calls it for EACH complete measurement
(~5.5 per second per beacon). `:reload` in the console hot-reloads this file
without restarting. Offline development is possible:

    python uart_console.py --file capture_ref_921600.txt

Contract
--------
Input : m, a cs_decoder.Measurement — complete and validated CS procedure:
    m.tones        list of ToneIQ sorted by increasing channel; for each t:
                     t.channel          CS channel (0..78)
                     t.freq_hz          tone frequency (Hz)
                     t.i_loc, t.q_loc   IQ measured by the initiator (12-bit signed)
                     t.i_ref, t.q_ref   IQ measured by the reflector
                     t.tq_loc, t.tq_ref tone quality (0 = good)
    m.rssi_loc     initiator RSSI (dBm)   — guard / weighting
    m.rssi_ref     reflector RSSI (dBm)
                   ⚠ 127 = "unavailable" (BLE sentinel 0x7F), not +127 dBm
    m.freq_offset_raw  mode-0 frequency offset (raw, 16 bits)
    m.rtt_pairs    RTT pairs (ToA-ToD init, ToD-ToA refl) in units of 0.5 ns
                   (mode-1 steps, ~5/measurement) — time of flight = (Ti - Tr)/2;
                   used to resolve the phase's 50 m ambiguity (step 4)
    m.beacon, m.counter, m.t_ms, m.t_done_ms

Output : estimated distance in meters (float), or None if the measurement is
         deemed unusable. Any exception is caught by the console (error printed
         once, following measurements processed normally).

Pipeline (faithful to the original MATLAB script):
    IFFT (global estimate, no local ambiguity)
      -> gradient_refine (von Mises likelihood ascent, Bessel I0/I1)
      -> naboer_global3 (anti-side-lobe: explores the aliases, keeps the best
         score, re-refines). Rmax = 50 m (alias c/(2*3 MHz) of thinning 3);
         lobe period = c/(2*span) computed per measurement.
"""
import math
from collections import defaultdict, deque

import numpy as np
from scipy.special import i0, i1

C = 299792458.0     # m/s

# ── Settings ─────────────────────────────────────────────────────────────────
# MATLAB delta_s (0.01) diverged here: dmr ~ hundreds -> steps of several
# meters, jumping over dozens of 6 cm lobes. Reduced step + capped at a quarter
# lobe (verified on capture_ref: converges in < 50 iterations).
DELTA_S   = 5e-5    # gradient-ascent step (m per derivative unit)
GAMMA     = 1.0     # ~linear SNR of the von Mises model (1 = neutral)
R_MAX     = 50.0    # m — coarse alias c/(2*3 MHz): beyond it, spectrum folds back
MIN_TONES = 5       # below this, the measurement is worthless (MATLAB: 2)

# ── Calibration (to fill in over serial with a tape measure — doc/CHECKLIST_TESTS.md) ─
# Offsets SUBTRACTED from the output: d_shown = d_measured - global - beacon.
# Adjustable live from the console (:cal / :cal <m> / :cal <beacon> <m>) —
# ⚠ a :reload reloads this file and reverts to the values written here.
CAL_OFFSET_M = 0.80           # common bias (group delay) — calibrated with tape
CAL_OFFSET_PER_BEACON = {}    # e.g. {0: 0.35, 2: -0.10} — adds to the global

# Distance of the LAST measurement BEFORE the sliding median (calibrated), read
# by the console after each estimate() call — used for the "raw" plots of the
# test reports. None if the measurement was rejected.
LAST_RAW = None


def lobe_period(freq):
    """Fine-lobe spacing of the likelihood: c/(2*f_carrier) ≈ 6.15 cm.
    (Measured on capture_ref: 6.1-6.2 cm — this is the round-trip carrier
    period, NOT the c/(2*span) ≈ 2 m envelope of the MATLAB script.)"""
    return C / (2.0 * float(np.mean(freq)))


def calcREG_sparse(transfer2, freq_hz):
    """Distance (m) by regression of the unwrapped phase vs frequency (Hz)."""
    if len(transfer2) < 2:
        return np.nan                       # not enough points for a line
    ang = np.unwrap(np.angle(transfer2))
    m, _ = np.polyfit(freq_hz, ang, 1)      # slope in rad/Hz
    return -C * m / (4 * np.pi)


def calcIFFTDist(transfer2, ch):
    """Distance (m) at the peak of the delay profile (IFFT on the 1 MHz channel grid)."""
    H = np.zeros(79, dtype=complex)
    H[ch] = transfer2                    # missing channels -> 0
    N = 2048
    yf = np.abs(np.fft.ifft(H, n=N)[:N // 2])
    return np.argmax(yf) * C / (2 * N * 1e6)     # 1e6 = grid step, constant


# ── Bayesian estimator (von Mises likelihood over the phases) ────────────────

def find_amp(r, freq, ang1, ang2):
    """Log-likelihood of distance r given the phases (sum log I0).
    r may be a scalar or a vector of candidates (vectorized)."""
    r = np.atleast_1d(np.asarray(r, dtype=float))
    phase = np.outer(r, 2 * np.pi * freq / C) - (ang1 + ang2) / 2
    amp = np.sum(np.log(i0(2.0 * GAMMA * np.cos(phase))), axis=1)
    return float(amp[0]) if amp.size == 1 else amp


def gradient_refine(ang1, ang2, freq, r_init, delta_s=DELTA_S):
    """Gradient ascent on the likelihood: refines r towards the top of the lobe
    nearest to r_init (LOCAL optimization — choosing the right lobe belongs to
    naboer_global3). Step capped at a quarter lobe: without this cap, dmr ~
    hundreds -> jumps of several meters (observed divergence of the raw MATLAB
    port)."""
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
    """Anti-wrong-lobe: enumerates the lobes r = first_est + k*(c/2f_mean) over
    all of [0, R_MAX], scores with find_amp (vectorized), refines the top 3, and
    returns the refined one with the best score. ("naboer" = "neighbors" in
    Norwegian.) Differs from MATLAB on the grid: at the REAL lobe spacing
    (~6.15 cm), not at period/5 ≈ 46 cm which fell between the lobes (random
    scores)."""
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


# ── Entry point called by the console ─────────────────────────────────────────

def estimate(m):
    """Estimate the distance (m) from a complete CS measurement."""
    global LAST_RAW
    LAST_RAW = None                          # no stale value on reject
    ch  = np.array([t.channel for t in m.tones])
    Y_I = np.array([complex(t.i_loc, t.q_loc) for t in m.tones])   # == l_i + 1i*l_q
    Y_R = np.array([complex(t.i_ref, t.q_ref) for t in m.tones])   # == p_i + 1i*p_q
    f   = np.array([t.freq_hz for t in m.tones])

    transfer2 = Y_I * Y_R
    ang1 = -np.angle(Y_I)
    ang2 = -np.angle(Y_R)

    tq  = np.array([t.tq_loc for t in m.tones])
    tqr = np.array([t.tq_ref for t in m.tones])

    valid = (tq == 0) & (tqr == 0) & (np.abs(Y_I) > 0) & (np.abs(Y_R) > 0)
    if np.count_nonzero(valid) < MIN_TONES:
        return None                          # unusable measurement

    ch_v   = ch[valid]
    f_v    = f[valid]
    t2_v   = transfer2[valid]
    ang1_v = ang1[valid]
    ang2_v = ang2[valid]

    # 1. Coarse global estimate (~2 m resolution, no unwrap)
    d_ifft = calcIFFTDist(t2_v, ch_v)

    # 2. Local refinement by the phase likelihood
    d_bay = gradient_refine(ang1_v, ang2_v, f_v, d_ifft)

    # 3. Lobe correction (the IFFT sometimes latches onto a reflected path:
    #    naboer re-searches for the globally most likely lobe)
    d_bay = naboer_global3(ang1_v, ang2_v, f_v, d_bay)

    if not np.isfinite(d_bay):
        return None

    # 4. RTT ambiguity resolution (mode-1 steps): the phase is precise but
    #    modulo 50 m (thinning-3 folding); the RTT is coarse (~±3 m LOS, worse
    #    in NLOS) but absolute -> it picks the 50 m slice.
    #    Zero correction as long as |d_rtt - d_bay| < 25 m (same slice).
    #    Bench-validated: beacon at ~58 m, folded phase 8.6 m -> merge 58.6 m.
    if m.rtt_pairs:
        d_rtt = float(np.median([(ti - tr) * 0.25e-9 * C
                                 for ti, tr in m.rtt_pairs]))
        d_bay += 50.0 * round((d_rtt - d_bay) / 50.0)
        d_bay = max(d_bay, 0.0)   # lobe escape re-aligned -> may be < 0

    # 5. Calibration: offsets measured with tape, subtracted from the output.
    d_bay -= CAL_OFFSET_M + CAL_OFFSET_PER_BEACON.get(m.beacon, 0.0)
    d_bay = max(d_bay, 0.0)
    LAST_RAW = float(d_bay)

    # 6. Sliding median PER BEACON (== movmedian(7) of the MATLAB): crushes
    #    isolated lobe/multipath jumps. Latency ~0.6 s at 5.5 Hz. Module state,
    #    reset by :reload.
    hist = _median_hist[m.beacon]
    hist.append(float(d_bay))
    return float(np.median(hist))
    # To compare estimators: return calcREG_sparse(t2_v, f_v), d_ifft or raw
    # d_bay — the same measurement, several readings.


_median_hist = defaultdict(lambda: deque(maxlen=7))
