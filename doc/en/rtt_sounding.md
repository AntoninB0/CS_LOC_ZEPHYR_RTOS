# RTT sounding sequence — report

**Project:** BLE Channel Sounding localization (PARNAV), nRF54L15, NCS v3.3.0.
English version of `doc/RAPPORT_RTT_SOUNDING.md`.

## Context and problem
In the drone profile the fine phase-based (PBR) estimate is ambiguous modulo its
non-ambiguous range — 50 m with thinning 3 — and the mode-1 **RTT** measurement
selects the correct fold. That is only correct if the RTT error stays well below
±25 m (half the alias period). RTT was configured as `AA_ONLY`: the CS_SYNC
packets are dated on their access address only, a pattern designed to *identify*
the packet, not to *time* it. Real data confirmed the weakness: on one frame the
per-step ToA/ToD spread from −117 to +49 half-nanoseconds (≈ ±25 m per step);
only the median over ~8 steps brought it back to a few metres — with no safety
margin. A single alias error is a 50 m jump, the worst case for a position
filter.

## Change
The drone profiles switch to `CS_RTT_TYPE = 32_BIT_SOUNDING`: the CS_SYNC packets
embed a 32-bit sequence dedicated to timing (sharp autocorrelation peak). The
precision profiles (docking, indoor) stay `AA_ONLY` — pure PBR emits no mode-1
step, so no point negotiating an unused capability. The parameter is centralized
per profile in `cs_config.h` (`CS_RTT_TYPE`).

On the Python side the estimator exposes the **NADM** (Normalized Attack
Detector Metric) of the peer mode-1 steps: `0xFF` (unavailable) with `AA_ONLY`,
it becomes meaningful with the sounding sequence — the controller compares the
received pattern to the expected one. Scale 0 (tampering very unlikely) to 6
(very likely), median per measurement.

## Expected effects
Per-step RTT precision improved ~3–5×: the alias arbiter becomes reliable even
in NLOS or at long range, and the number of mode-1 steps could later be reduced
(airtime returned to PBR). NADM gives a per-measurement integrity indicator —
severe multipath, a relay or spoofing distort the pattern — directly relevant to
navigation in GNSS-denied environments. Cost: a few tens of µs of airtime per
mode-1 step, absorbed by the 4500 µs subevent.

## Test plan and fallbacks
Reflash **both** initiator and reflectors (the capability is negotiated by both
ends; nRF54L15 supports it). Verify in order: (1) no 0x11/0x12 at Create Config
/ Procedure Enable — else fall back one line to `AA_ONLY`; (2) distances stable,
timings unchanged; (3) with IQON, the per-step ToA/ToD dispersion visibly
tightens and `nadm` is no longer None; (4) recalibrate the RTT offset at a known
distance (the group delay changes with the dating type). Possible next steps:
`96_BIT_SOUNDING` if airtime allows, and `CS_SYNC_PHY = 2M_2BT` (sounding gives
the pattern to time, BT=2 sharpens its edges) — test one variable at a time.
