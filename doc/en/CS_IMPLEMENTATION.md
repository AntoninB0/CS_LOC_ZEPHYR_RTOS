# Channel Sounding multi-reflector ranging — implementation guide

Single source of information for the firmware and host tools built during this
internship. It explains how we went from the nRF Connect SDK Channel Sounding
sample to the current multi-reflector, free-running acquisition system, and lists
everything needed to reproduce and extend it.

The narrative context (why CS, what ranging is) lives in the report. This
document is the technical reference.

---

## 1. Repository, branch, and version

| Item | Value |
|---|---|
| Repository | `git@github.com:AntoninB0/CS_LOC_ZEPHYR_RTOS.git` |
| Branch | `main` |
| Commit (HEAD) | `e7c81243bf9474be842c8dc597c1ee8c0c35132a` |

Note: at the time of writing the working tree also contains uncommitted doc and
tool additions (this file, `tools/report_figures.py`, `doc/figures/`) and the
removal of old test folders. Commit them before archiving so the SHA above
points to the complete state.

To mirror this to the group server, add the remote and push a branch, for
example:

```bash
git remote add ntnu git@git.ntnu.no:uavlab/parnav-nrf-apps.git
git push ntnu main:cs-loc-zephyr
```

## 2. Reproducibility

```
Software
  nRF Connect SDK (NCS)   v3.3.0     (Zephyr 4.3.99)
  Toolchain               NCS bundle 911f4c5c26
  Flash / debug           nrfutil (west runner), SEGGER J-Link (onboard DK)
  Host OS                 Linux (Ubuntu-class), bash
  Python                  3.12.3
    numpy 2.5.1  ·  scipy 1.18.0  ·  matplotlib 3.11.0  ·  pyserial 3.5

Hardware
  Boards                  Nordic nRF54L15DK  x4  (1 initiator + 3 reflectors)
  Initiator J-Link SNR    1057778811
  Reflector J-Link SNRs   <fill in the three reflector SNRs>
  Distance reference      laser rangefinder  <fill in model>
  Host link               USB-CDC (VCOM): data @ 921600, console @ 115200
```

## 3. Starting point and what changed

The firmware started from the two NCS samples:

- `samples/bluetooth/channel_sounding_ras_initiator`
- `samples/bluetooth/channel_sounding_ras_reflector`

Those samples connect one initiator to one reflector, run a fixed number of CS
procedures on demand, fetch the reflector data over the Ranging Service (RAS),
and compute a single distance on the device. The changes we made, from the
sample to the current system:

1. **Multi-reflector.** The initiator connects to N reflectors (1 to 5), runs CS
   on all of them concurrently, and keeps per-beacon state. See section 5 and 8.
2. **Free-running acquisition.** Procedures are armed once and repeated by the
   controller, instead of one enable per measurement. This is the largest
   measurement-rate change. See section 6.
3. **Real-time RAS pipeline.** The reflector data is pushed by notification and
   paired to the local data by ranging counter, instead of a sequential GATT
   fetch per measurement. See section 7.
4. **On-device distance removed.** The device no longer estimates distance. It
   streams the raw IQ (local and peer) over UART; the estimation and all the
   analysis are done off-board in Python. See section 7.3.
5. **Configuration profiles.** A single header selects an application profile and
   the number of reflectors; all timing, buffers, and CS parameters are derived
   with compile-time guards. See section 4.
6. **Runtime manager.** UART commands to toggle the IQ dump, manage a connection
   whitelist, and stage a channel-map hot-swap. See section 9.

## 4. Configuration system (`initiator/src/cs_config.h`)

Two knobs drive everything:

- **Application profile** (exactly one `CONF_*`): `CONF_DRONE_FAST_OUTDOOR`,
  `CONF_BOAT_DOCKING`, or `CONF_DRONE_INDOOR`.
- **`CS_NUM_BEACONS`** (1 to 5): number of reflectors.

From these, the header derives the channel map thinning, subevent length,
procedure length, mode and RTT type, TX power, connection interval, and all the
buffer sizes. `initiator/src/cs_config.c` turns them into the four HCI parameter
structs (`bt_le_cs_create_config_params`, `bt_le_cs_set_procedure_parameters_param`,
`bt_le_cs_set_default_settings_param`, `bt_le_conn_param`).

Two protections prevent the classic misconfigurations:

- **Compile-time guards** (`BUILD_ASSERT` in `cs_config.c`) reject a build where
  the SoftDevice Controller timeline reservations do not cover the profile, for
  example `SPACING < ACL + CS event + 3 ms guard` (guaranteed 0x3 aborts) or
  `CS_EVENT_LEN_DEFAULT < max_subevent_len` (procedure parameters rejected with
  0x12). Each profile family that uses a longer CS event ships a `prj.conf`
  overlay (`overlay-precision-single.conf` for N=1, `overlay-precision-multi.conf`
  for N>=2); the drone profile uses the base `prj.conf`.
- **Runtime capability fallback** (`cs_apply_local_caps` in `cs_ranging.c`) reads
  the local CS capabilities at boot and downgrades any optional feature the
  controller does not support, so Create Config never fails with 0x11: CS_SYNC
  2M_2BT falls back to 2M, channel selection 3C to 3B, and RTT sounding to
  AA_ONLY. On this hardware (nRF54L15 SDC) sounding sequence, CSA 3C, 2M_2BT and
  NADM are not supported, so the usable RTT lever is the random payload.

## 5. Architecture

Roles: the **initiator** is the BLE central; each **reflector** is a peripheral
that advertises the Ranging Service and answers CS. Threads on the initiator:

- **`main` (measurement loop).** Round-robin over the beacons: arm a beacon once,
  then collect its paired measurements and dump the IQ. Runs after all reflectors
  are connected.
- **`pairing`.** Scans, connects, runs pairing and security, and gates the start
  of the measurement loop until `CS_NUM_BEACONS` reflectors are connected and
  secured. Restarts scanning while the count is below the target.
- **`display`.** One log line per second listing connected beacons and their
  state (setup or ranging).
- **manager UART line callback.** Parses runtime commands (section 9).

Per-beacon connection lifecycle, driven by `cs_setup_beacon` in `cs_ranging.c`:

1. MTU exchange (without it the RAS transfer stays at 23-byte ATT MTU and is very
   slow).
2. GATT discovery of the Ranging Service and RAS client handle allocation.
3. Read remote CS capabilities.
4. Set default CS settings (initiator role, TX power).
5. Subscribe to RAS (real-time notification subscription in the default build).
6. Create CS config (mode, RTT, channel map with optional thinning).
7. Enable CS security (ranging keys).
8. Set procedure parameters (free-running, subevent length).

## 6. Free-running acquisition (the main rate change)

The sample used `max_procedure_count = 1`: one Link Layer handshake
(`LL_CS_REQ/RSP/IND`, about 11 connection events, roughly 495 ms) per
measurement. With three beacons the measured one-shot cycle was about 1485 ms.

We set `max_procedure_count = 0` (`cs_config.c`). The controller then repeats the
procedure every `procedure_interval` connection events on its own, and the LL
handshake is paid only once, at arming. The steady-state per-beacon sampling
period becomes `procedure_interval x connection_interval`. For the drone profile
at N=3 that is `4 x 45 ms = 180 ms` per beacon.

The measurement loop (`main.c`) therefore only arms each beacon once (tracked by
`armed[i]`) and then spends its time collecting paired measurements. When a
beacon disconnects, `armed[i]` is cleared and the beacon is re-armed on its
return.

## 7. Real-time RAS pipeline and IQ output

### 7.1 Pairing local and peer data

Each completed procedure produces local step data (initiator side, delivered in
`subevent_result_cb`) and, a connection event or two later, a RAS notification
with the peer step data (reflector side, delivered in `rt_data_cb`). Both carry a
**ranging counter**. Because a live procedure keeps writing into the local
buffer, the completed procedure is **snapshotted** immediately inside the local
callback (`cs_snapshot_from_local`), and the peer notification is matched to that
snapshot by ranging counter. A mismatch means the collector fell behind and that
measurement is dropped, not corrupted.

`snap_busy` protects a snapshot during the UART dump: rather than overwrite it,
the callbacks skip the next measurement. This is the `dump en cours, mesure
sautee` log line.

### 7.2 Real-time versus on-demand

`CS_RAS_REALTIME = 1` (default) uses the pushed real-time subscription. The
fallback (`= 0`) uses the on-demand path (`rd_ready` then a GATT Get Ranging Data
on the RAS control point). On-demand adds a sequential fetch per measurement and
is kept only as a validated fallback.

### 7.3 IQ output format

The device does not compute distance. When the IQ dump is on (default), each
measurement is emitted on the data UART as two line types:

```
IQL,<beacon>,<counter>,<t_ms_subevent>,<hex local step data>   (one per subevent)
IQP,<beacon>,<counter>,<t_ms_done>,<hex peer RAS ranging data>
```

The host tools parse these lines. All distance estimation and the consistency
study are done off-board.

## 8. Multi-beacon scheduling

The connection interval is fixed (`interval_min == interval_max`):

- **N = 1:** a floor of `ACL + CS event + margin` (the SPACING grid does not
  apply to a single central link).
- **N >= 2:** `CS_NUM_BEACONS x SPACING`, so the SDC spaces the ACL anchors of
  the links deterministically and avoids collisions.

Getting this wrong is the source of the structural 0x3 aborts, which is why the
build guard checks it. The per-beacon rate scales as 1/N; the aggregate rate over
all reflectors stays close to the single-link rate.

## 9. Runtime manager (UART commands)

`initiator/src/manager.c` parses lines from the console UART:

- `IQON` / `IQOFF` / `AUTO` / `ORDER` toggle and steer the IQ dump.
- Whitelist: `WL:ON/OFF/ADD/DEL/CLR/LIST`. Disabled by default (all connections
  allowed). When enabled, `connected_cb` rejects any address not on the list.
- `CHMAP:*` stages a channel map that overrides the compile-time map at the next
  config creation (a live hot-swap needs the procedure to be re-created).

## 10. Measurement-rate impact of each change

Rates below are per beacon, measured during the campaign, unless stated.

| Change | Effect on rate | Note |
|---|---|---|
| One-shot to **free-running** (`max_procedure_count` 1 to 0) | ~1485 ms 3-beacon cycle to ~180 ms per beacon | the dominant win, LL startup paid once |
| On-demand to **real-time RAS** | removes the per-measurement GATT fetch | lets procedures run continuously |
| **Channel thinning** (72 to 24 channels) | shorter procedure, higher rate | trades frequency diversity |
| **Channel-map repetition x2** (boat N=1) | roughly halves the rate | each channel sounded twice |
| **Adding a beacon** | per-beacon rate / N, aggregate ~constant | see section 8 |

Measured single-link baselines: drone 24-channel N=1 = 16.5 Hz; full 72-channel
N=1 = about 11 Hz; boat 72-channel with repetition 2 = 6.2 Hz; drone N=3 = 5.0 Hz
per beacon (about 15 Hz aggregate).

## 11. Configuration cost table (for scenario planning)

Rough cost of the main levers, to make ballpark configuration decisions.

| Lever | Measurement rate | Consistency / accuracy | Confidence |
|---|---|---|---|
| Include RTT (versus PBR only) | ~1% (measured N=1 A/B: 16.47 Hz on vs 16.67 Hz off) | no gain in per-tone coherence; enables unambiguous range beyond the PBR limit (~50 m) — without it the distance estimate aliases (sigma 0.24 m -> 18 m in the same A/B) | measured |
| Reduce number of frequencies | rate up (shorter procedure) | per-channel coherence unchanged; fewer channels means worse worst case (10th percentile) and less resolution / shorter ambiguity period | measured (study) |
| Add a beacon | per-link rate / N, aggregate ~constant | per-link consistency unchanged | measured (study) |

Caveat: the delivered profiles bundle several parameters at once (channels,
subevent length, RTT, repetition), so a clean per-lever A/B (change one define,
hold the rest) is still needed to turn these into exact numbers.

## 12. Robustness and known limitations

- **Reconnection after a drop.** The reflector re-advertises automatically on
  disconnect (a delayed workqueue, never from the disconnect callback directly),
  and the initiator re-arms a beacon when it comes back. Walking out of range and
  back should therefore recover on its own once the link supervision timeout
  (3.2 s) fires. This path has been reasoned from the code but not yet stress
  tested; it is the first thing to verify before flight.
- **Reflash of the initiator requires a reflector power-cycle.** When the
  initiator is reflashed it resets without a clean disconnect, so the reflector
  keeps the stale link and does not re-advertise until its supervision timeout or
  a power-cycle. On the bench we power-cycle the reflectors after every initiator
  flash. A cleaner fix is to shorten the supervision timeout or force a
  disconnect on the reflector side.
- **Low SNR.** As the link weakens the per-tone phase becomes noise dominated and
  the coherence collapses (this is the main result of the study). Deep
  frequency-selective fades produce dead channels even when the average RSSI is
  acceptable.
- **0x3 aborts.** A subevent can be aborted when it collides with another link's
  ACL anchor. The connection-interval grid and the build guard are designed to
  avoid the structural case; sporadic aborts remain and cost a measurement, not a
  crash.
- **No connection reference during measurement.** The measurement loop does not
  hold a `bt_conn_ref` while measuring, which is fine on the bench but should be
  hardened for production (a disconnect mid-cycle is not fully protected).

## 13. Reflector firmware (`reflector/src/main.c`)

Minimal by design: set the reflector CS role, advertise the Ranging Service, and
re-advertise on disconnect. The CS event length is reserved at 8500 in
`reflector/prj.conf`, which covers every profile (drone subevent 4500 and indoor
or boat subevent 8000), so the reflectors do not need to be reflashed when the
initiator profile changes.

## 14. Host tools (`tools/`)

- `uart_console.py` reads the data UART, records a timed capture with
  `--test <seconds>`, and archives a JSON (with the raw IQ), a config snapshot,
  and the auto figures under `tests/YYYYMMDD/test_<stamp>/`.
- `iq_consistency.py` computes the coherence metric R from a capture JSON and
  produces the per-beacon consistency report.
- `report_figures.py` builds the synthesis figures used in the report.

Set up: `python -m venv tools/.venv && tools/.venv/bin/pip install numpy scipy
matplotlib pyserial`.

## 15. Build and flash (`build.sh`)

`build.sh` detects the J-Link boards by serial, flashes the pinned initiator, and
flashes the rest as reflectors. Key flags:

- no flag: drone profile (base `prj.conf`, no overlay)
- `--boat-1`: single-link overlay (N=1 for the boat or indoor family)
- `--boat-n`: multi-link overlay (N>=2 for the boat or indoor family)
- `--initiator-only` / `--reflectors-only` / `--full-reflector`
- `--clean` (needed after a family change, to drop a stale overlay cache),
  `--no-flash`, `--init-snr <SNR>`

The flag names refer to the overlay (single or multi), not to the profile family:
the family is chosen in `cs_config.h`. The initiator SNR is pinned in `build.sh`
(`INITIATOR_SNR_PIN`); update it if the initiator board changes.

## 16. Future recommendations

- **Harden reconnection for flight.** Verify and stress test loss and recovery of
  coverage, shorten or tune the supervision timeout, and hold a connection
  reference during measurement. Fix the reflector-restart-on-init-reflash case.
- **Adaptive channel map.** Use the low 10th percentile of the coherence to drop
  the faded channels and keep the reliable ones (at least 15 for the ambiguity
  period). This raises fidelity and frees airtime, which can go to the update
  rate. The manager already has the channel-map hot-swap hook.
- **Clean per-lever A/B tests.** Measure RTT on/off, thinning, and repetition one
  at a time to replace the ballpark cost table with numbers.
- **Sensor fusion.** On a moving platform, fuse the CS ranging with an IMU in a
  Kalman filter: the IMU covers the fast motion between updates, the CS ranging
  corrects the IMU drift. This relaxes the sampling-rate requirement.
- **Controlled profile ranking.** Run the three profiles at one matched clean
  geometry with a few repetitions to turn the campaign trends into a firm ranking
  with error bars.

### 16.1 Longer-term ideas (brain dump)

- **Fingerprinting.** Instead of (or alongside) geometric trilateration, build a
  map of the per-channel phase/coherence signature at known locations and
  estimate position by matching a live measurement against that map. Attractive
  indoors where multipath is stable but breaks the geometric model; the same IQ
  we already stream is the feature vector. Cost: a survey phase and a database,
  and it is only valid while the environment stays static.
- **Online mode switch (calibration then cruise).** Run a short startup/calib
  stage that uses ALL channels (assume stationarity) for ~10 s to fix the
  position with high accuracy and, at the same time, assess which channels are
  good (high coherence, low multipath). Then switch to a cruise mode with fewer
  channels and a higher measurement rate: since the position is already known,
  the ambiguity window can be small, so fewer channels are enough. The
  channel-map hot-swap hook in the manager is the mechanism.
  *Limitation:* a mode switch is NOT microseconds. Changing the channel map
  requires re-creating the CS config and restarting the procedure (disable ->
  create_config(new map) -> re-enable), which costs on the order of the LL
  startup (hundreds of ms, effectively ~seconds once settled), so it is a
  phase change, not a per-measurement knob.

## 17. Design notes (reviewer questions)

**ACL.** ACL (Asynchronous Connection-oriented Logical transport) is the ordinary
BLE data connection between the initiator (central) and a reflector (peripheral).
Channel Sounding rides on top of an ACL link: the CS procedures are scheduled
relative to the ACL connection events, and the reflector's ranging data is
carried back over that same ACL via GATT (the Ranging Service). This is why the
timeline reservations are expressed as "ACL event + CS event + guard".

**The Ranging Service is part of the SoftDevice / NCS stack.** RAS is not
application code we wrote: it ships with the nRF Connect SDK Bluetooth stack
(`bluetooth/services/ras.h`, server `RRSP` on the reflector, client `RREQ` on the
initiator), and the CS controller support underneath it is provided by the
SoftDevice Controller (nrfxlib SDC). We only configure and consume them.

**Including RTT and its effect on the measurement rate.** RTT adds a small number
of mode-1 (time-of-flight) steps to each procedure, interleaved with the mode-0
and mode-2 (PBR tone) steps. A direct A/B (single reflector, drone profile, 24
channels, only the RTT toggled: mode-2 sub-mode-1 + 32-bit random vs mode-2
no-submode / pure PBR) gave **16.47 Hz with RTT vs 16.67 Hz without** — about a
1% difference, within measurement noise. The reason it is not more affected: the
procedure airtime is dominated by the PBR tones (channels x mode-2 steps) plus
the fixed per-subevent and per-connection-event overhead; the RTT is only a
handful of extra steps on top of that, so it is a small fraction of the total.
RTT earns its place because it resolves the phase ambiguity beyond the PBR
non-ambiguous range for almost no rate cost: in the same A/B, removing it made the
distance estimate alias badly (sigma 0.24 m -> 18 m), which is exactly the failure
RTT prevents.

**How the results are sent over UART when `max_procedure_count = 0`.** In
free-running there is no per-measurement enable, so nothing gates the output on a
command. Instead the controller keeps producing procedures, and for each one the
real-time RAS notification pairs with the local snapshot (by ranging counter);
the main loop's `cs_collect_beacon()` picks up each paired measurement as it
completes and emits the `IQL`/`IQP` lines on the data UART. The stream is
therefore continuous and paced by the procedure rate, round-robin across the
beacons, with no start/stop handshake per sample.

**Maximum number of BLE connections (multi-point limit) and memory.** The knob is
`CONFIG_BT_MAX_CONN` (currently 5). A build can raise it (the SDC supports on the
order of ~20 concurrent links), but two limits bite before any hard controller
ceiling:
- *Timeline / rate.* The connection interval must hold the whole grid,
  `interval >= N x SPACING` (SPACING is ~15-20 ms per link), and the per-beacon
  rate scales as 1/N. So more links directly cost update rate: e.g. at N=20 with
  a 20 ms slot the interval is ~400 ms, i.e. a few Hz aggregate spread over 20
  reflectors. The application runs out of useful rate long before it runs out of
  connection slots.
- *Memory.* Each link costs RAM that scales with N: ACL TX/RX buffers, one CS
  reassembly buffer (`CONFIG_BT_CHANNEL_SOUNDING_REASSEMBLY_BUFFER_CNT`), a key
  pool slot (`CONFIG_BT_MAX_PAIRED`), and on our side a per-beacon `cs_meas`
  struct holding the local + peer IQ buffers (sized by `CS_MAX_STEPS_PER_PROC`,
  which is why the profiles shrink it at high N). RAM is the practical ceiling for
  large N, and it is the reason `CS_MAX_STEPS_PER_PROC` and the buffer counts are
  tuned per profile rather than left at a fixed maximum.

**Why not increase the ACL latency.** Peripheral latency lets a link skip
connection events to save power, but it directly delays and de-paces the CS
schedule: CS procedures are anchored to the connection events, so allowing the
link to skip them would stretch and jitter the measurement cadence and slow the
RAS data return. We keep `latency = 0` (see `cs_cp_active` in cs_config.c) so
every connection event is available to the CS timeline; the update rate, not
power, is the priority here.

**Single vs multi-beacon coherence.** See `doc/figures/fig5_single_vs_multi.png`:
at a matched 5 m geometry, the per-link top-X% coherence curve at N=3 sits on top
of the N=1 curve for both the drone (24 ch) and indoor (72 ch) profiles (the two
whose channel map does not change with N), confirming that multi-beacon does not
degrade the per-link acquisition quality.

**Coherence — reference.** The metric R is the mean resultant length of circular
statistics (Mardia and Jupp, *Directional Statistics*, Wiley, 2000) and is the
same quantity as the interferometric coherence used in SAR/InSAR (Bamler and
Hartl, "Synthetic aperture radar interferometry", *Inverse Problems*, 1998) and
as a phase-lock indicator in GNSS.

## 18. Known issue: single-reflector bug

There is a reported single-reflector bug (possibly specific to BOAT mode). It is
not yet root-caused or documented here in detail: the exact symptom, the profile
it triggers on, and the trigger conditions still need to be captured. Placeholder
until confirmed — see the report TODOs.

## 19. References

- NCS v3.3.0 Channel Sounding RAS samples (initiator and reflector).
- Bluetooth Core Specification, Channel Sounding (Vol 6, Part H).
- Repository `CS_LOC_ZEPHYR_RTOS`, branch `main`, commit
  `e7c81243bf9474be842c8dc597c1ee8c0c35132a`.
- Study method and results: `doc/METHODE_ETUDE.md`, `doc/STUDY_PLAN.md`,
  `doc/RESULTATS_CAMPAGNE.md`, figures in `doc/figures/`.
