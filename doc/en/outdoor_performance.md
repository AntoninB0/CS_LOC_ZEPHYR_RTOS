# Performance report — outdoor tests (2026-07-31)

English version of `doc/RAPPORT_PERFORMANCES_EXTERIEUR.md`. Tests run outdoors,
fixed initiator, **board rotated 90°** (rot90, to probe antenna-orientation
sensitivity). Session cut short by rain → few drone-mode tests.

⚠️ **Ground truth = GPS**, so imprecise (±a few metres). This report therefore
judges mainly **precision/repeatability** (dispersion), not absolute accuracy —
laser calibration is still to be done. A systematic bias (~+2 m seen indoors) is
not yet corrected.

## Summary: outdoor is conclusive
Sharp contrast with indoors. Outdoors the measurements are **stable at the
centimetre**; indoors (Elektro hall) multipath makes them erratic.

| Context | typical σ | min σ | verdict |
|---|---|---|---|
| Indoor hall (drone fast) | 2–5 m | 0.06 m | multipath → unusable |
| **Outdoor drone fast** | 0.2–2 m | 0.16 m | **conclusive** (few tests) |
| **Outdoor boat/docking** | 0.05–0.15 m | **0.03 m** | **very good** on a good link |

## Docking (boat) outdoor — 12 tests
Fixed geometry, per-beacon consensus distances:

| Beacon | Distance | typical σ | min σ | RSSI |
|---|---|---|---|---|
| b2 | 5.03 m | 0.08 m | **0.03 m** | −59 dBm |
| b1 | 24.98 m | 0.13 m | 0.07 m | −80 dBm |
| b0 | 16.19 m | 0.07 m (good link) | 0.07 m | −82 dBm |

**Strengths:**
- **b2 (~5 m, LOS) is flawless** over the 12 tests: σ 3–20 cm, no outliers.
  Centimetre repeatability confirmed.
- **b1 at ~25 m held to ±7–15 cm** despite a weak link (−80 dBm): the
  docking mode's frequency diversity (all channels) pays off outdoors.

**Identified limit — orientation sensitivity:**
- b0 is **bimodal** with rotation: ~16 m (σ ~0.1 m, good) on some tests, but
  jumps to ~37 m (σ up to 10 m) on other orientations, with RSSI dropping
  (−75 → −84 dBm). b1 also degrades (up to 47 m) on the last 3 tests.
- Interpretation: at some rot90 orientations the **antenna pattern attenuates
  the link** to the far/weak beacons and the estimator then locks onto a
  reflected path. Antenna orientation matters for already-marginal links.

## Drone fast outdoor — 3 tests (cut short, rain)
| Beacon | Distance | σ | RSSI | rate |
|---|---|---|---|---|
| b0 | 5.04 m | 0.2–0.7 m | −55 dBm | 5.2 Hz |
| b1 | 16.18 m | 0.6–2.4 m | −71 dBm | 5.3 Hz |
| b2 | 24.92 m | 0.16–0.5 m | −77 dBm | 5.2 Hz |

**Strengths:** excellent sampling (~5.2–5.3 Hz/beacon, well above docking's
~3.5–4 Hz); remarkable repeatability (medians agree to the cm across the 3
tests); robust to antenna rotation (unlike docking, rot90 barely changes the
distances). **Caveat:** very small sample (rain); b1 (~16 m) shows a higher σ
(up to 2.4 m) on 2 of 3 tests.

## Conclusions
1. **Outdoors validates the approach:** away from multipath, cm precision in
   docking and 5 Hz + good repeatability in drone fast.
2. **Indoor multipath is limiting factor #1**, confirmed by the ~×30 σ contrast.
3. **Antenna orientation (rot90)** hits the docking mode on weak links; drone
   fast is robust.
4. **To do:** laser calibration (absolute bias not measured here, GPS too
   coarse); a full outdoor drone-fast campaign; a controlled angular sweep to
   characterize antenna orientation.
