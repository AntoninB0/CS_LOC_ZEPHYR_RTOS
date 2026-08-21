# Multi-reflector BLE Channel Sounding — Guide

nRF54L15 initiator that ranges against several reflectors over Bluetooth LE
Channel Sounding (CS). The firmware **acquires** timestamped IQ and streams it
over UART; all **estimation and analysis run off-board in Python**. Acquisition
and estimation are deliberately decoupled: the raw IQ is stored, so any
estimator can be replayed offline on the same data.

---

## 1. Main commands

### Build & flash — `build.sh`
The application profile and the number of reflectors are set at compile time in
`initiator/src/cs_config.h` (one `CONF_*` family + `CS_NUM_BEACONS`). `build.sh`
then builds and flashes with the matching overlay.

```bash
./build.sh                      # build + flash everything (drone profile, base prj.conf)
./build.sh --boat-1             # boat/indoor, 1 reflector  (overlay-precision-single)
./build.sh --boat-n             # boat/indoor, N>=2 reflectors (overlay-precision-multi)
./build.sh --initiator-only     # rebuild/flash only the initiator (profile/N/scheduler change)
./build.sh --reflectors-only    # rebuild/flash only the reflectors (CS_EVENT_LEN family change)
./build.sh --clean              # wipe build dirs first (mandatory when changing family/profile)
./build.sh --no-flash           # build only
```
Rule of thumb: change **N only** → `--initiator-only`; change **family**
(drone ↔ boat/indoor, i.e. `CS_EVENT_LEN`) → reflash reflectors too, with
`--clean`.

### UART console + firmware commands — `uart_console.py`
The data UART (`uart30`, VCOM Serial Port 0, **921600 baud**) carries the IQ
stream *and* accepts one-line commands. The other VCOM (`uart20`, Serial Port 1)
carries the Zephyr logs.

```bash
cd tools && source .venv/bin/activate        # venv: see install.txt (pyserial numpy scipy matplotlib)
python uart_console.py --list                # find the data /dev/ttyACM* (the one printing IQL/IQP)
python uart_console.py --port /dev/ttyACM0   # live: decoded measurements + estimated distance
python uart_console.py --file capture.txt    # offline replay of a raw capture
python uart_console.py --port /dev/ttyACM0 --test 30   # record 30 s -> tests/YYYYMMDD/test_.../
```

Firmware commands (type them in the console; handled by `manager.c`):

| Command | Effect |
|---|---|
| `IQON` / `IQOFF` | enable / disable the IQ dump (on by default) |
| `WL:ON` / `WL:OFF` | enable / disable the connection whitelist |
| `WL:ADD <addr> [public\|random]` | allow a reflector (random = default) |
| `WL:DEL <addr> [public\|random]` | remove one |
| `WL:CLR` / `WL:LIST` | clear / print the whitelist |
| `CHMAP:THIN <n>` / `CHMAP:FULL` | stage a decimated / full channel map (applied at next config) |
| `HELP` | list commands |

Console-side commands: `:test [s]` (record), `:cal [b] [m]` (calibration offset),
`:show`/`:nshow`, `:raw`, `:stats`, `:reload` (hot-reload `estimation.py`), `:q`.

A `--test` produces `tests/YYYYMMDD/test_YYYYMMDD_HHMMSS/` (grouped by day,
anchored at the repo root whatever the working directory) containing: the JSON (per-measurement
distance **and raw IQ**, `t_done_ms`, `n_subevents`), the time-series PNG,
`config.txt` (firmware config snapshot), and `estimateur/` (per-beacon IFFT +
Bayesian figures).

### Final analysis
```bash
# tests live under ../tests/YYYYMMDD/test_<stamp>/ ; a JSON path below is e.g.
#   ../tests/20260818/test_20260818_150449/test_20260818_150449.json

# IQ consistency — the acquisition-quality metric (static setup, no distance involved)
python iq_consistency.py --json ../tests/<day>/<test>/<test>.json                 # global + per-channel plot
python iq_consistency.py --json ../tests/<day>/<A>/<A>.json ../tests/<day>/<B>/<B>.json   # compare configs

# Estimation pipeline, interactive (IFFT + Bayesian + decision, time slider, beacon select)
python viz_live.py --json ../tests/<day>/<test>/<test>.json

# Static estimator figures (one per beacon; --worst = the multipath failure case)
python viz_estimateur.py --json ../tests/<day>/<test>/<test>.json --worst
python viz_estimateur.py --file capture.txt          # from a raw capture (also decodes RTT)
```
`estimation.py` is the estimator module (phase IFFT + von Mises Bayesian + RTT
fusion + calibration `CAL_OFFSET_M`); the tools above import it, so what they
show is exactly what the estimator computes.

---

## 2. Architecture

- **`initiator/`** — scans for the Ranging Service (UUID `0x185B`), connects to
  the reflectors, runs CS, and streams timestamped IQ over UART. No distance
  computation on-board.
- **`reflector/`** — CS reflector + Ranging Service (RAS) responder.
- **`tools/`** — the Python chain: decode → (estimate) → analyze.

Firmware modules: `cs_config.*` (compile-time profile), `cs_ranging.*` (CS setup
+ free-running measurement + IQ dump), `manager.*` (UART commands, whitelist,
channel-map staging), `beacon.*` / `pairing.*` (connection/CS-setup lifecycle),
`if.*` (UART), `main.c` (glue + measurement loop).

---

## 3. CS configuration (`cs_config.h`)

Two knobs drive everything: the **profile family** and **`CS_NUM_BEACONS` (N)**.
All timing, buffers and channel map derive from them; `BUILD_ASSERT`s in
`cs_config.c` reject inconsistent combinations.

| Parameter | Drone fast (outdoor) | Boat docking | Indoor drone |
|---|---|---|---|
| CS mode | mode-2 + sub-mode-1 | mode-2, no sub-mode | mode-2, no sub-mode |
| Channel thinning | 3 (~24 ch) | 1 (2 if N≥3) | 1 (2 if N≥4) |
| Subevent length | 4500 µs | 8000 µs | 8000 µs |
| Max procedure len | 40 u (25 ms) | 100 u (62.5 ms) | 100 u (62.5 ms) |
| RTT | 32-bit random | none (pure PBR) | none |
| Max TX power | max (+8 dBm) | configurable (0/+4/+8) | max |
| Slot per link | 15 ms | 20 ms | 20 ms |
| Overlay | none (base prj.conf) | precision-single/multi | precision-single/multi |

**Scaling with N.** The controller places each central connection's ACL anchor
on a grid of pitch `CENTRAL_ACL_EVENT_SPACING`. One cell holds one ACL event +
one CS event + a guard, so for N≥2 the connection interval is **N × slot**
(not free). The per-link update rate is `η / (procedure_interval × N × slot)`;
the **total** throughput is independent of N (adding reflectors distributes the
measurements, buying geometry/GDOP, not rate). Measured: **12.45 meas/s** on 3
reflectors (docking) vs 0.5 Hz for the Nordic single-reflector sample.

**Startup optimization.** Enabling CS makes the SDC silently raise the ACL
anchor spacing to 90 000 µs, which does not fit a small connection interval →
the first subevent was delayed ~700 ms. Resizing the three reservations
(`CS_EVENT_LEN`, `CENTRAL_ACL_EVENT_SPACING`, `MAX_CONN_EVENT_LEN`) to the real
footprint brought startup to ~340 ms. The remaining ~340 ms is a fixed
link-layer handshake (~11 connection events); it is **hidden** by free-running
(`max_procedure_count = 0`): each procedure repeats on its own, so the handshake
is paid once at arming, never per measurement.

**Per-connection-event layout** (one grid cell): `[ ACL event ][ CS subevent ]
[ guard ]`. ACL and CS never overlap by design; an abort with subevent reason
`0x3` marks an ACL that overran into the guard, reason `0x2` marks a missing
CS_SYNC (link/sync issue).

---

## 4. IQ dump format

On each completed procedure (with `cs_iq_dump` on) the firmware emits ASCII
lines on the data UART:

```
IQL,<beacon>,<ranging_counter>,<t_ms>,<HEX>   one line PER local subevent
IQP,<beacon>,<ranging_counter>,<t_ms>,<HEX>   reflector RAS ranging data
```
- `t_ms` = board `k_uptime` (ms) at the **subevent's** HCI arrival — the finest
  timestamp the controller exposes (per subevent, **not** per tone/step;
  per-tone timing must be reconstructed from the deterministic step durations).
- Local HEX: stream of HCI steps `{mode(1), channel(1), len(1), data(len)}`. A
  mode-2 step = `antenna_permutation(1)` then N×`[PCT(3)+tone_quality(1)]`; the
  PCT packs I (low 12 bits) and Q (high 12 bits), signed.
- Peer HEX: RAS ranging data, decoded in Python (`cs_decoder`).

The decoder pairs a local snapshot with the peer half by `(beacon,
ranging_counter)`. One CS procedure sweeps ~27 channels per 8 ms subevent; a
full 72-channel sweep spans ~3 subevents (≈40 ms wall-clock, ~24 ms airtime),
which fits per procedure at N=1 but only ~55 channels at N≥2.

---

## 5. Python toolchain (`tools/`)

| File | Role |
|---|---|
| `uart_console.py` | live console: send commands, decode + display measurements, record `--test` folders |
| `cs_decoder.py` | parse/validate/pair the IQL/IQP lines into `Measurement` objects |
| `estimation.py` | distance estimator: IFFT seed → von Mises Bayesian (anti-alias) → RTT fusion → calibration → median. `CAL_OFFSET_M` = group-delay offset |
| `iq_consistency.py` | **acquisition-quality score** from the raw IQ (per-channel phase stability), global + per-channel, compare mode |
| `viz_live.py` | interactive pipeline viewer (IFFT + Bayesian + decision, time slider, beacon select) |
| `viz_estimateur.py` | static per-measurement figures (IFFT delay profile + Bayesian lobes) |

Venv: `cd tools && python3 -m venv .venv && source .venv/bin/activate &&
pip install -r install.txt` (pyserial, numpy, scipy, matplotlib).

---

## 6. Test protocol

Record the firmware version (`git describe --tags --dirty`, must not be
`-dirty`) and the physical setup (antenna, height, TX power) before each
campaign; `config.txt` in every `--test` folder snapshots the firmware config.

- **Group-delay calibration** (before any exploited measurement): two boards at
  a precisely known distance (1 m), `--test 30`, `offset = median − true`, set
  `CAL_OFFSET_M` in `estimation.py`. Per board-pair; verify the offset is
  **constant** across distances (2 points), otherwise it is a scale error, not a
  group delay.
- **Campaign A — precision** (outdoor LOS, 1 reflector): bias/σ vs distance.
- **Campaign B — cadence & scaling** (fixed geometry, N = 1..4): per-link rate,
  completion ratio, aborts, startup latency. Measure cadence with `IQOFF` so the
  IQ dump does not throttle the captured rate.
- **Campaign C — indoor / multipath**: LOS / partial / NLOS; keep the captures
  where a reflected IFFT peak dominates the direct one.

⚠ 2.4 GHz environment matters: passing bodies shadow the direct path (transient
outliers, RSSI dips) and phones/Wi-Fi add interference (aborts). A precision
reference must be **static and quiet**.

---

## 7. Study methodology — consistency, not distance

The study optimizes the **acquisition** (sampling + CS configuration), not the
estimation. Reported distances are dominated by downstream factors outside the
acquisition (estimator lobe selection, multipath, calibration), so the results
are reported on **IQ consistency**, which measures the acquisition directly.

For a **static** setup the channel is constant, so each tone's phase must be
stable over time. Per channel k:
> `R_k = | mean_t( exp(j·arg(Y_I·Y_R)) ) |`  ∈ [0,1] — 1 = stable, 0 = random.

The two-way product `Y_I·Y_R` cancels the common LO drift. Global score = median
of `R_k`; the 10th percentile flags faded channels. Measured examples: 2 m clean
(−54 dBm) → **0.97**, 5 m (−71 dBm) → **0.70**, 10 m faded (−78 dBm) → **0.35**;
the per-channel curve tracks the amplitude (fading) exactly. Comparing configs
(thinning, subevent length, TX power, N) on this score at fixed geometry ranks
them by acquired-IQ quality — the conclusion of the sampling-optimization study.

Because the raw IQ is stored in each test JSON, any estimator can be replayed
offline on past data (`viz_live --json`, `viz_estimateur --json`). Limit: the
JSON stores the mode-2 tones + RSSI, not the mode-1 RTT / mode-0 offset — keep
the raw `.txt` capture for full RTT replay (drone profile).

---

## 8. Validated results (bench)

- **Outdoor precision (docking).** With a good link, centimetre-level: b2 at 5 m
  → σ 3–8 cm; b1 at 25 m held to ±7–15 cm even at −80 dBm (dense-map frequency
  diversity pays off at range).
- **Outdoor cadence (drone fast).** ~5.2 Hz per beacon with cm-level
  repeatability across runs (RSSI ranking follows distance ranking = healthy
  measurements).
- **Indoor multipath is the #1 limiter.** Same profile, indoor σ is ~30× the
  outdoor σ (0.06 m → 2–5 m); a reflected IFFT peak can dominate the direct one.
- **Antenna orientation** matters for weak links: a 90° rotation makes the
  docking profile bimodal on the far/weak beacons (correct distance vs a +20 m
  reflection), while the drone-fast profile is largely robust to it.
- **RTT anti-alias (drone).** The drone profile uses `32_BIT_SOUNDING` (not
  `AA_ONLY`): the CS_SYNC packets carry a dedicated dating sequence, tightening
  the per-step RTT dispersion so the alias arbiter stays reliable (< half the
  alias period) in NLOS / at range, and exposing the NADM integrity metric
  (mode-1 steps). Precision profiles stay `AA_ONLY` (no mode-1 in pure PBR).
  Detailed report: `doc/RAPPORT_RTT_SOUNDING.md` (EN: `doc/en/rtt_sounding.md`).

Detailed outdoor campaign: `doc/RAPPORT_PERFORMANCES_EXTERIEUR.md`
(EN: `doc/en/outdoor_performance.md`). Test protocol / checklist:
`doc/CHECKLIST_TESTS.md` (EN: `doc/en/test_checklist.md`).

## 9. Open items

- **Channel-map hot-swap (live):** `CHMAP:*` stages a map applied at the next CS
  config (i.e. at reconnection). A *live* swap without reconnection requires
  re-creating the CS config mid-connection (disable → create_config → re-enable)
  = a procedure restart — to validate on bench.
- **Connection whitelist:** application-level filter by address (`WL:*`), for
  static-random reflector addresses; would need bonding/IRK for rotating RPAs.
- **Manager over BLE/GATT** (instead of UART) for a remote controller.
