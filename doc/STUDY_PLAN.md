# Study plan — IQ consistency of a multi-reflector BLE Channel Sounding acquisition

Report-ready. Sections 1–4 are meant to be lifted into the report; sections 5–6
are the field checklist.

---

## 1. Objectives

**Primary objective.** Characterize the **consistency (repeatability) of the raw
Channel Sounding IQ** as a function of the acquisition configuration and the link
conditions — i.e. evaluate how well a given CS configuration *samples the
channel*, independently of any distance estimator.

**Rationale (why not distance).** Distance is a downstream, estimator-dependent
quantity whose error is dominated by factors outside the acquisition (lobe
selection, multipath, calibration); it is therefore not a reliable metric for an
acquisition/sampling study, and it precludes mobile tests (the IQ assumes a
static channel). The consistency of the IQ measures the acquisition **directly**,
at the physical layer, with a standard metric.

**Secondary objectives.**
- Identify the physical driver of consistency (link budget / frequency-selective
  fading) and quantify it.
- **Rank the delivered application profiles** — `DRONE_FAST_OUTDOOR`,
  `BOAT_DOCKING`, `DRONE_INDOOR` — by acquired-IQ quality, each in its target
  environment and cross-environment (robustness). Within a profile, evaluate
  only the few decision-relevant parameters that carry an open design question.
- Define the **reliable channel subset** (top-X%) per profile/condition, to
  inform an adaptive channel map.

---

## 2. Metric

Per channel *k*, over a **static** capture:
> `R_k = | (1/T) · Σ_t exp( j · arg( Y_I(t) · Y_R(t) ) ) |`  ∈ [0, 1]

`R_k` is the **mean resultant length** (circular statistics) = the
**interferometric coherence**: 1 = perfectly stable phase over time, 0 = random.
The two-way product `Y_I·Y_R` cancels the common local-oscillator drift.

- **Global score:** median of `R_k`; 10th percentile (worst channels);
  amplitude-weighted mean.
- **Reliability-vs-fraction (top-X%) curve:** the coherence quantile — "the best
  X % of channels all have `R ≥ y`". Summarizes how many channels are usable and
  ranks configurations (area under the curve).

**Key property:** `R` is invariant to any *constant* phase offset — the group
delay **and** the absolute distance both cancel. **No group-delay calibration is
required** for this study. Distance still matters, but only as an experimental
*condition* (it sets the link budget → SNR → fading → `R`), recorded as metadata.

Tool: `python iq_consistency.py --json <test.json> --report` (single) or several
`--json` (compare).

---

## 3. Hypotheses / expected outcomes

- **H1** — `R` is governed by the per-tone SNR (amplitude): all conditions
  collapse onto a single `R(amplitude)` curve.
- **H2** — multipath (indoor / NLOS / low antenna) produces frequency-selective
  fading → per-channel `R` dips at the fade nulls → lower global `R`.
- **H3** — configuration trade-offs: higher TX → higher SNR → higher `R`; channel
  thinning trades density for airtime without changing per-channel `R` (only the
  count/diversity); `N` (reflectors) changes the connection interval and the
  per-procedure channel coverage.
- **H4** — the reliable subset (top-X%) is config- and environment-dependent → an
  adaptive channel map (keep reliable channels, ≥ 15, drop dead ones) beats a
  fixed sparse pattern.

---

## 4. Experimental design

- **The primary factor is the application profile** (`DRONE_FAST_OUTDOOR`,
  `BOAT_DOCKING`, `DRONE_INDOOR`) — the coherent, *delivered* configurations. A
  profile is a bundle of coupled parameters (channel map, subevent, RTT, TX)
  tuned for a use case; sweeping them individually breaks that coupling, so
  single-parameter variations are **secondary** and used only where a real design
  question remains (e.g. boat channel thinning, TX for range).
- **One factor at a time.** Fix the geometry to attribute an effect to the
  **profile/config**; fix the profile to attribute an effect to the
  **environment**.
- **The number of reflectors (N) is a factor, not just a scaling knob.** `N = 1`
  sweeps the full channel map every procedure (best per-channel statistics — the
  characterization mode), whereas the delivered trilateration mode (`N ≥ 3`)
  samples a *random channel subset* per procedure and adds cross-link scheduling
  (possible 0x3 aborts). Both single-beacon and multi-beacon must be
  characterized. Note: at `N ≥ 2` a given channel is sampled in only some
  procedures → use a longer capture so each channel still has enough samples.
- **Static and quiet only.** The metric requires a stationary channel; movement,
  foot traffic and heavy 2.4 GHz interference are excluded from the reference
  (kept as a separate robustness test).
- **Each test** = 30 s (~150 procedures × ~72 channels × N beacons → a large
  sample; no per-point repetition needed for statistical power). The IQ and a
  `config.txt` snapshot are archived automatically under
  `tests/YYYYMMDD/test_<stamp>/`.
- **Volume:** ~30–40 tests total. The cost is the **reflash** per config change
  (compile-time), not the number of tests → batch all config changes at one
  geometry in a single session; minimize physical re-positioning (the main error
  source).

---

## 5. Test matrix — field checklist

The **application profile is the primary axis.** Session order: profile
comparison at one clean geometry first (the study's conclusion), then each
profile in its target environment, then cross-environment robustness, then the
few targeted within-profile questions.

### Before starting
- [ ] Both boards flashed with the same build; version noted (`git describe --tags --dirty`, no `-dirty`).
- [ ] `--test` verified on a trial acquisition (IQ present in the JSON).
- [ ] Static, quiet spot; distances measured (metadata, **not** for calibration).
- [ ] Antenna height and orientation fixed and noted.
- [ ] ⚠ Changing profile **family** (drone ↔ boat/indoor) = reflash the **reflectors** too (`CS_EVENT_LEN`).

### Scenario 1 — Profile comparison ⭐ (same clean static geometry, e.g. 2 reflectors @ 5 m LOS)
- [ ] `DRONE_FAST_OUTDOOR` — reflash, `--test 30`
- [ ] `BOAT_DOCKING` — reflash, `--test 30`
- [ ] `DRONE_INDOOR` — reflash, `--test 30`
→ the core result: which *delivered* profile acquires the most consistent IQ at a
matched, clean condition (dense vs thinned map, subevent length, TX).

### Scenario 2 — Each profile in its target environment
- [ ] `DRONE_FAST_OUTDOOR` @ outdoor LOS, a few distances — cadence/robustness domain
- [ ] `BOAT_DOCKING` @ close range 2–5 m LOS — short-range precision domain
- [ ] `DRONE_INDOOR` @ indoor multipath / NLOS — multipath domain
→ does each profile hold up where it was designed to be used?

### Scenario 3 — Cross-environment robustness (profile × environment)
Run each profile **outside** its native environment to see how `R` degrades:
- [ ] `BOAT_DOCKING` outdoors at range (does the dense map fade at range?)
- [ ] `DRONE_FAST_OUTDOOR` indoors (does thinning-3 hurt in dense multipath?)
- [ ] `DRONE_INDOOR` outdoors (baseline)
→ the profile × environment matrix; which profile is most robust.

### Scenario 4 — Number of reflectors: single vs multi-beacon (profile + geometry fixed)
- [ ] **N = 1** — full 72-channel sweep per procedure (characterization mode)
- [ ] **N = 2**
- [ ] **N = 3** — the delivered trilateration mode (partial per-procedure coverage + cross-link)
→ does the **per-beacon** consistency at N ≥ 3 match single-beacon? watch the
0x3 aborts and the per-channel sample count (use a longer `--test`, e.g. 60 s, so
each channel is sampled enough despite the partial coverage). `iq_consistency`
already reports per beacon (`--beacon N` to isolate one).

### Scenario 5 — Targeted within-profile tuning (only the open questions)
- [ ] `BOAT_DOCKING` thinning **1 vs 2** at fixed geometry (55-vs-72-tone / fading trade-off)
- [ ] TX **0 vs +8 dBm** at range on the profile under test (link budget)

### Scenario 6 — Antenna (profile + distance fixed)
- [ ] height **low** vs **high** (ground reflection)  · [ ] orientation **0°** vs **90°** (rot90)

### Scenario 7 — Reproducibility / stationarity
- [ ] repeat one Scenario-1 profile **×3** (repeatability of `R`)
- [ ] one **30 min** static test (`--test 1800`) — drift check

### Scenario 8 — Robustness (separate, not part of the reference)
- [ ] quiet vs busy 2.4 GHz / with foot traffic — expect transient `R` drops + aborts

### Field notes (record for each test)
- [ ] profile, true distance(s), antenna height/orientation, environment, anything abnormal.

---

## 6. Per-test procedure
1. Set the config (`cs_config.h`), reflash, wait for all reflectors connected.
2. Static and quiet; wait ~1 s after arming.
3. `python uart_console.py --port /dev/ttyACM0 --test 30`.
4. Note the metadata; the folder (`tests/YYYYMMDD/test_<stamp>/`) archives IQ + config.
5. (Offline) `iq_consistency.py --json <...> --report` — check the stationarity
   panel; discard the test if R drifts across the capture (movement/traffic).

---

## 7. Analysis & deliverables (report figures)
- **Per config:** the 4-panel consistency report (per-channel coherence,
  reliability-vs-fraction, distribution, coherence-vs-amplitude).
- **Cross-config:** the comparison figure (overlaid top-X% curves) + a summary
  table (`R_median`, `R_p10`, area-under-top-X%) → the **configuration ranking**.
- **Physical mechanism:** the universal `R(amplitude)` curve (all conditions on
  one curve) + a per-channel fading example (nulls ↔ coherence dips).

## 8. Validity & caveats
- Static-channel assumption — validated per test by the stationarity panel.
- No absolute-distance claim (calibration not part of the study).
- Antenna orientation is a confound on weak links — controlled in Scenario 5.
- Interference / foot traffic excluded from the reference (Scenario 7 only).
