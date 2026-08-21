# Test checklist — campaign & improvements

English version of `doc/CHECKLIST_TESTS.md` (working checklist, state 2026-07-15;
predates the switch to IQ-consistency as the primary study metric — see
`GUIDE.md` §7). Bench-validated: IQR 6–40 cm precision (LOS, static), RTT
anti-alias demonstrated at ~58 m, NLOS detectable by phase/RTT divergence.
Remaining: absolute accuracy (bias), dynamics, radio profiles.

## Tests to run
- [ ] **Accuracy calibration** — top priority. Boards raised, clear LOS, one
      beacon at a time by tape: 1 / 2 / 4 / 8 / 15 m (+ one far point). At each:
      `python uart_console.py --port /dev/ttyACM0 --test 30` → note
      (true, median_m from JSON).
      * constant offset → `:cal <offset>` then set `CAL_OFFSET_M` in
        estimation.py;
      * offset ∝ distance → investigate (scale error, report it);
      * offset differs per beacon → `CAL_OFFSET_PAR_BEACON`.
- [ ] **Index ↔ physical board mapping**: move one beacon by 1 m, identify its
      index in the console, label the boards (b0/b1/b2).
- [ ] **Static drift**: nothing moving, `--test 1800` (30 min). Median stable?
      plot d(t) from the JSON `mesures` (crystal thermal drift).
- [ ] **Dynamic test**: walk with a beacon along a measured round trip, ~1 m/s.
      Check tracking, the sliding-median lag (~0.6 s), RTT-slice jumps in motion.
- [ ] **Deliberate NLOS**: mask a beacon (body, pillar, bag). Confirm the
      signature: RSSI drop + phase/RTT divergence + spread IQR. Note the distance
      overestimation.
- [ ] **Range limit**: move a beacon away until link loss (~ −85 dBm around 58 m
      indoors). Note max range and measurement rate.
- [ ] **Radio profiles/modes** (if time): thinning 3 → 2 → 1 in cs_config.h
      (reflash initiator only), compare precision/rate on the same 5 m point.
      Family change (BOAT/INDOOR) = reflash the reflectors too.

## Code — done
- [x] Calibration offset: `CAL_OFFSET_M` + `CAL_OFFSET_PAR_BEACON` (estimation.py),
      live `:cal` command, offsets logged in each test JSON (`calibration` field).
- [x] `:test [s]` / `--test N`: windowed capture, per-beacon report (median, IQR,
      σ, outliers, RSSI), raw data + report in JSON (now with per-tone IQ too).
- [x] RTT fusion (50 m ambiguity resolution), per-beacon sliding median.

## Code — to improve (by priority)
- [ ] **Amplitude weighting of `find_amp`** (γ per tone ∝ |z|): fewer lobe
      escapes. estimation.py, ~10 lines.
- [ ] **Configurable median window** (`:med N`?): 7 = static comfort, 3 = flight
      reactivity. Decide after the dynamic test.
- [ ] **Explicit NLOS gate**: expose |d_rtt − d_phase| and RSSI as a confidence
      indicator (rich dict return from estimate()).
- [ ] **Beacon identity on the data UART**: `MSG,<idx>,<addr>` line at connection
      + REPORTON/REPORTOFF commands.
- [ ] **CRC8 at end of IQ lines** (firmware + decoder) if losses reappear.
- [ ] Cadence: try `procedure_interval` 4 → 3 once the rest is validated — watch
      for 0x3 aborts.
