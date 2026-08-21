# Section 4 — continuation: derived configuration and the boat-docking channel-sweep correction

*Draft continuation of section 4 (v2). Grounded in the committed configuration
of `CS_LOC_ZEPHYR_RTOS@main` (`initiator/src/cs_config.h`, `cs_config.c`,
`initiator/prj.conf`, the two precision overlays, and `reflector/prj.conf`).
Passages in **[R.x]** are open points for the author. Passages marked
**PROPOSAL** are configuration changes not yet applied.*

This continuation adds two things the v2 draft left implicit. First, section
4.14 turns the profile table (4.3) into the **derived runtime quantities** the
reader actually cares about — connection interval, subevents per procedure,
expected tones, alias period, delay resolution, cadence — computed from the two
compile-time symbols so that no number is asserted without its derivation.
Second, section 4.15 resolves the open question raised by the docking campaign,
namely why a dense-map profile returns only 55 of the 72 available tones, and
gives a decision-ready correction with its firmware, reflector and build-guard
consequences.

---

## 4.14 Derived runtime parameters

Everything below is a preprocessor consequence of exactly two symbols, the
application profile and `CS_NUM_BEACONS = N`. Table A evaluates the derivation
at the number of reflectors each profile is dimensioned for, so the report can
be read without expanding the macros by hand.

The chain is: `interval = N × slot` (multi-link) with `slot = CS_SPACING_UNITS ×
1.25 ms`; one CS subevent is placed per connection event; a procedure spans
`ceil(CS_PROC_LEN / interval)` connection events, hence that many subevents;
procedures repeat every `procedure_interval = 4` connection events; per-reflector
rate is `η / (4 × interval)`.

| Derived quantity | Drone fast (N=3) | Boat docking (N=3) | Indoor drone (N=3) |
|---|---|---|---|
| Slot per link | 12 u = 15 ms | 16 u = 20 ms | 16 u = 20 ms |
| Connection interval | 3×15 = **45 ms** | 3×20 = **60 ms** | 3×20 = **60 ms** |
| Subevent length | 4500 µs | 8000 µs | 8000 µs |
| CS event reservation | 5000 µs (base prj.conf) | 8500 µs (precision overlay) | 8500 µs (precision overlay) |
| Max procedure length | 40 u = 25 ms | 100 u = 62.5 ms | 100 u = 62.5 ms |
| Subevents per procedure | 1 | **2** | 2 |
| Channel thinning | 3 (≈24 ch) | 1 (72 ch) | 1 (72 ch) |
| Channel-map repetition | 1 | 1 (N>2) | 1 |
| Sounded tones / procedure | ≈24 (full thinned grid) | **≈55 of 72** [R3] | ≈55 of 72 [R3] |
| Alias period `c/(2·Δf)` | 3 MHz → **50 m** | 1 MHz → **150 m** | 150 m |
| Delay resolution `c/(2·span)` | full band → ≈2 m | ≈2 m | ≈2 m |
| Procedure interval | 4 events = 180 ms | 4 events = 240 ms | 240 ms |
| Per-reflector rate (η→1) | 5.56 Hz | **4.17 Hz** | 4.17 Hz |
| Aggregate throughput | 16.7 /s | **12.5 /s** | 12.5 /s |
| `CS_MAX_STEPS_PER_PROC` | 64 | **96** (N=3) | 96 (N=3) |

*Table A: runtime quantities derived from the two compile-time symbols, evaluated
at N=3. The docking column is the one validated by the 31 July 2026 campaign
(measured 4.10–4.17 Hz per link, 12.45 /s aggregate).*

Two rows carry the whole of section 4.15: **2 subevents per procedure** and
**≈55 of 72 tones**. They are not independent — the second is a consequence of
the first, and the first is a consequence of `CS_PROC_LEN (62.5 ms)` being barely
larger than the connection interval (60 ms).

---

## 4.15 The boat-docking channel-sweep correction

### 4.15.1 Root cause, exactly

The docking profile leaves the map dense (`CS_CHANNEL_THINNING = 1`), so 72
channels are eligible. Yet every procedure returned 55 tones in the campaign.
The deficit is a **timing budget** effect, not a rejection of samples:

- The controller reserves one CS event (`CS_EVENT_LEN = 8500 µs`) per connection
  event and per link. A subevent of 8000 µs therefore consumes exactly one
  connection event of the anchor grid.
- `CS_PROC_LEN = 100 u = 62.5 ms` against a `60 ms` connection interval lets a
  procedure straddle only **two** connection events, i.e. two subevents.
- Each 8000 µs subevent carries ≈27 main-mode-2 steps once the calibration
  steps are removed (3 mode-0 in the first subevent). Two subevents → ≈54–55
  tones, matching the observation.
- The remaining budget is real but unused: procedures are spaced 4 connection
  events = **240 ms** apart, so ≈74 % of the window is idle.

The damaging part is not the count but the **draw**. Under CSA #3b the 55
sounded channels are a *random subset of the 72*, redrawn every procedure, so
the grating-lobe structure of the delay profile changes measurement to
measurement. This is the leading candidate for the raw outliers observed on the
weak link b0 (11 m and 23 m for a true 16 m), which are too small to be 50 m/150 m
alias jumps and are consistent with the estimator locking onto a reflected path
whose lobe pattern shifts between procedures. [R3: confirm by logging the
per-procedure channel indices — experiment T7.]

### 4.15.2 The three corrections, quantified

| | Option 1 — thinning 2 | Option 2 — longer procedure | Option 3 — longer subevent |
|---|---|---|---|
| Change | `CS_CHANNEL_THINNING → 2` in docking | `CS_PROC_LEN 100 → 200…256 u` | `CS_SUBEVENT_LEN 8000 → 12000 µs` |
| Sounded channels | 36, **complete & deterministic** | 72, complete | 72, complete |
| Subevents / procedure | 2 (36 ≤ 2×27 capacity) | 3–4 | 2 (fewer, wider) |
| Measurement aperture | 62.5 ms (unchanged) | 125–160 ms | 62.5 ms |
| Alias period | 2 MHz → **75 m** | 150 m | 150 m |
| Delay resolution (span) | ≈2 m (span unchanged) | ≈2 m | ≈2 m |
| Buffer need vs `CS_MAX_STEPS` | 36+overhead ≪ 96 ✓ | ≈72+3×4 = 84 < 96 at N=3 ✓, **> 64 at N≥4 ✗** | ≈72+overhead, check per N |
| Reflector reflash | no | no | **yes** (`CS_EVENT_LEN 8500→12500`) |
| Overlay / build-assert | none | none | spacing guard tight: 3750+12500+3000 = 19250 ≤ 20000 (**750 µs margin**) |
| Cost | **none** | aperture ×2.5 | invasive, both ends |

Notes that decide it:

- **Option 1** removes the *variability* at its source: 36 channels fit inside
  the two available subevents (capacity ≈54), so every procedure sweeps the
  **identical, complete** thinned grid — no redrawn subset, stable lobe
  structure. Span is unchanged (every other channel across the whole band), so
  the ≈2 m delay resolution is preserved; only the alias period halves to 75 m,
  which is still far beyond any docking envelope. It costs nothing in time and
  needs no reflector change. It also **generalises the policy already in the
  code**, which imposes thinning 2 for docking at N≥4 for RAM reasons.
- **Option 2** is the one to add *if and only if* the full-span 72-channel grid
  is genuinely needed (finer lobe rejection) and the target is slow enough to
  accept a 125–160 ms aperture — acceptable for docking, not for a drone. It is
  free of reflector/overlay changes but is **capped by `CS_MAX_STEPS_PER_PROC`**:
  fine at N≤3 (96), unsafe at N≥4 (64) where it would risk a *Step data buffer
  overflow* abort.
- **Option 3** trades calibration overhead per tone for a wider subevent, but
  requires reflashing every reflector with a matching `CS_EVENT_LEN` and leaves
  only 750 µs of guard in the multi overlay spacing assertion. Lowest priority.

### 4.15.3 Recommendation and proposed change

Apply **Option 1** as the default docking correction, and keep Option 2 as a
bench-selectable variant for the case where full-span resolution is required.

**PROPOSAL** (one line in `initiator/src/cs_config.h`, docking block):

```c
/* before */
#define CS_CHANNEL_THINNING     (CS_NUM_BEACONS >= 4 ? 2 : 1)
/* after — thinning 2 from N=3 up: 36-channel grid fully swept every
 * procedure (deterministic), alias 75 m ≫ docking envelope, aperture and
 * reflector config unchanged. N≤2 left at 1 for now: the deficit was only
 * measured at N=3, the shorter interval fits more subevents, and chmap
 * repetition 2 adds frequency averaging — revisit after T7. */
#define CS_CHANNEL_THINNING     (CS_NUM_BEACONS >= 3 ? 2 : 1)
```

Reflector side: **unchanged** — `CS_EVENT_LEN` stays 8500, no reflash needed
(subevent length and CS event reservation are untouched). Build side: no overlay
or assertion change; the existing `overlay-precision-multi.conf` still applies.

Rebuild is initiator-only: `./build.sh --boat-n --initiator-only`.

Expected effect: sounded tones become a constant 36 on a fixed grid; the raw
dispersion of the weak links should drop because the lobe pattern no longer
moves between procedures; per-reflector cadence is unchanged (aperture
unchanged). This is the hypothesis that experiment T7 must confirm.

---

## 4.16 Construction zone — to be closed by the author

*Not for the final document. Working notes and decisions.*

- **[R3 / T7] Confirm the mechanism.** Log the channel indices actually sounded
  per procedure, docking thinning 1 vs thinning 2, and check: (a) that thinning 1
  really draws a *varying* ~55-channel subset, (b) that thinning 2 sweeps the
  *same* 36 channels every time, (c) whether the b0 raw outliers disappear with
  thinning 2. Until logged, 4.15.1 remains an inference, not a measurement.
- **Decision — Option 2 as a variant?** If the delay-resolution argument matters
  for multipath rejection at the dock, add a docking sub-variant with
  `CS_PROC_LEN = 200` and verify at the target N that step count stays under
  `CS_MAX_STEPS_PER_PROC`. Left open on purpose.
- **N≥4 docking.** With thinning 2 already imposed there and `CS_MAX_STEPS = 64`,
  36 channels are safe; a full 72-channel Option 2 is not. State this as a hard
  limit rather than a tuning knob.
- **Result table.** Re-run the dispersion table (draft Table 5) after the
  thinning-2 change on the same geometry, and report raw σ side by side with the
  thinning-1 baseline — this is the single number that proves or refutes 4.15.

[bench results to be inserted here]
