#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cs_decoder.py — cleanup + decoding of the CS initiator IQ stream.

Input: raw UART lines (live or file) in the format documented in GUIDE.md (section 4):
    IQL,<beacon>,<counter>,<t_ms>,<HEX>   (one line PER local SUBEVENT)
    IQP,<beacon>,<counter>,<t_ms>,<HEX>   (reflector RAS ranging data)

Output: COMPLETE and VALIDATED `Measurement` objects — the two halves (local
and reflector) of the same procedure, aligned channel by channel — ready for
distance estimation. Anything corrupt, incomplete or inconsistent is dropped
and counted (see `DropStats`).

Library usage (the estimation function is the caller's responsibility):

    from cs_decoder import measurements_from_serial

    for m in measurements_from_serial("/dev/ttyACM0", 921600):
        d = estimate(m)         # <- your function
        # m.tones: list of ToneIQ sorted by increasing channel
        #   t.freq_hz             tone frequency (Hz)
        #   t.i_loc, t.q_loc      IQ measured by the initiator (12-bit signed)
        #   t.i_ref, t.q_ref      IQ measured by the reflector
        #   t.tq_loc, t.tq_ref    tone quality (0 = good)
        # m.rssi_loc / m.rssi_ref (dBm), m.t_ms, m.beacon, m.counter

CLI usage (decoding control, no estimation):
    python cs_decoder.py --file capture_raw.txt --stats
    python cs_decoder.py --port /dev/ttyACM0 --baud 921600 --stats
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter, OrderedDict
from dataclasses import dataclass, field

# ── Protocol constants ───────────────────────────────────────────────────────

CS_CHANNEL_MAX = 78              # valid CS channels: 0..78 (2402..2480 MHz)
CS_FREQ_BASE_HZ = 2_402_000_000  # channel k -> 2402 + k MHz

# Data sizes per mode, LOCAL side (HCI, initiator role).
# mode 2: 1 + (n_paths+1)*4 -> allowed sizes for 1..4 antenna paths.
LOCAL_MODE0_LEN = 5
LOCAL_MODE1_LENS = (6, 14)       # 14 = with sounding PCT (not used here)
LOCAL_MODE2_LENS = (9, 13, 17, 21)

RAS_RANGING_HEADER_LEN = 4       # counter(12b)+config(4b), tx_power, aa_mask
RAS_SUBEVENT_HEADER_LEN = 8      # start_evt(2) freq_comp(2) done(1) abort(1)
                                 # ref_power(1) num_steps(1)


class DecodeError(ValueError):
    """Structurally invalid line/payload. `reason` feeds the stats."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


# ── Structures ───────────────────────────────────────────────────────────────

@dataclass
class Step:
    mode: int
    channel: int          # -1 on the reflector side (RAS does not emit the channel)
    payload: bytes
    aborted: bool = False


@dataclass
class ToneIQ:
    """One paired PBR tone: the two halves of the same mode-2 step."""
    channel: int
    freq_hz: float
    i_loc: int
    q_loc: int
    tq_loc: int
    i_ref: int
    q_ref: int
    tq_ref: int


@dataclass
class Measurement:
    """A complete, validated and aligned CS procedure."""
    beacon: int
    counter: int
    t_ms: int                    # board k_uptime at the arrival of the 1st subevent
    t_done_ms: int               # board k_uptime at the procedure end (IQP)
    tones: list[ToneIQ]          # sorted by increasing channel
    rssi_loc: int                # dBm, 1st local mode-0 step (127 = unavailable)
    rssi_ref: int                # dBm, 1st reflector mode-0 step (127 = unavailable)
    freq_offset_raw: int         # local mode-0 frequency offset (raw, 16 bits)
    n_subevents: int             # number of aggregated IQL lines
    # RTT steps (mode 1): pairs (ToA-ToD initiator, ToD-ToA reflector) in units
    # of 0.5 ns. Time of flight tau = (Ti - Tr)/2 -> d = (Ti - Tr)*0.075 m.
    # Coarse (~meters) but WITHOUT aliasing: used to resolve the phase's 50 m
    # ambiguity. Steps at the 0x8000 sentinel (unavailable) already dropped.
    rtt_pairs: list[tuple[int, int]] = field(default_factory=list)


@dataclass
class DropStats:
    """Count of everything that did not end up as a Measurement."""
    lines: int = 0
    measurements: int = 0
    dropped: Counter = field(default_factory=Counter)

    def drop(self, reason: str):
        self.dropped[reason] += 1

    def summary(self) -> str:
        total_drop = sum(self.dropped.values())
        out = [f"{self.lines} lines -> {self.measurements} complete measurements, "
               f"{total_drop} drops"]
        for reason, n in self.dropped.most_common():
            out.append(f"  {reason}: {n}")
        return "\n".join(out)


# ── Stage 1: ASCII envelope ──────────────────────────────────────────────────

def parse_envelope(line: str):
    """'IQL,b,rc,t,HEX' -> (tag, beacon, counter, t_ms, bytes). DecodeError otherwise."""
    parts = line.split(",", 4)
    if len(parts) != 5 or parts[0] not in ("IQL", "IQP"):
        raise DecodeError("envelope")
    try:
        beacon, counter, t_ms = int(parts[1]), int(parts[2]), int(parts[3])
    except ValueError:
        raise DecodeError("envelope") from None
    hexstr = parts[4].strip()
    if not hexstr or len(hexstr) % 2:
        raise DecodeError("hex-odd")
    try:
        data = bytes.fromhex(hexstr)
    except ValueError:
        raise DecodeError("hex-invalid") from None
    # fromhex tolerates spaces: reject them explicitly
    if " " in hexstr:
        raise DecodeError("hex-invalid")
    return parts[0], beacon, counter, t_ms, data


# ── Stage 2a: local steps (HCI format: mode|channel|len|data) ────────────────

def decode_local_steps(data: bytes) -> list[Step]:
    steps, pos = [], 0
    while pos + 3 <= len(data):
        mode, chan, dlen = data[pos], data[pos + 1], data[pos + 2]
        if chan > CS_CHANNEL_MAX:
            raise DecodeError("iql-channel")
        if (mode == 0 and dlen != LOCAL_MODE0_LEN) or \
           (mode == 1 and dlen not in LOCAL_MODE1_LENS) or \
           (mode == 2 and dlen not in LOCAL_MODE2_LENS) or mode > 2:
            raise DecodeError("iql-mode-len")
        payload = data[pos + 3:pos + 3 + dlen]
        if len(payload) < dlen:
            raise DecodeError("iql-truncated")
        steps.append(Step(mode, chan, payload))
        pos += 3 + dlen
    if pos != len(data):
        raise DecodeError("iql-leftover")
    if not steps:
        raise DecodeError("iql-empty")
    return steps


# ── Stage 2b: reflector RAS ranging data ─────────────────────────────────────

def _peer_step_sizes(aa_mask: int) -> dict[int, int]:
    n_ap = bin(aa_mask).count("1")
    if not 1 <= n_ap <= 4:
        raise DecodeError("iqp-aa-mask")
    return {0: 3, 1: 6, 2: 1 + (n_ap + 1) * 4}


def decode_peer(data: bytes):
    """RAS ranging data -> (counter12, [Step...]). Structure:
    ranging header (4 bytes), then per subevent: header (8 bytes) + num_steps
    steps 'mode(1)|data', WITHOUT channel or len field (size implied by mode).
    A step with mode bit 7 set is aborted and carries no data."""
    if len(data) < RAS_RANGING_HEADER_LEN + RAS_SUBEVENT_HEADER_LEN:
        raise DecodeError("iqp-short")
    counter12 = int.from_bytes(data[0:2], "little") & 0x0FFF
    aa_mask = data[3]
    sizes = _peer_step_sizes(aa_mask)

    steps, pos = [], RAS_RANGING_HEADER_LEN
    while pos < len(data):
        if pos + RAS_SUBEVENT_HEADER_LEN > len(data):
            raise DecodeError("iqp-header-truncated")
        num_steps = data[pos + 7]
        pos += RAS_SUBEVENT_HEADER_LEN
        for _ in range(num_steps):
            if pos >= len(data):
                raise DecodeError("iqp-steps-missing")
            mode_byte = data[pos]
            pos += 1
            aborted = bool(mode_byte & 0x80)
            mode = mode_byte & 0x7F
            if mode not in sizes:
                raise DecodeError("iqp-mode")
            if aborted:
                steps.append(Step(mode, -1, b"", aborted=True))
                continue
            payload = data[pos:pos + sizes[mode]]
            if len(payload) < sizes[mode]:
                raise DecodeError("iqp-step-truncated")
            steps.append(Step(mode, -1, payload))
            pos += sizes[mode]
    if pos != len(data):
        raise DecodeError("iqp-leftover")
    if not steps:
        raise DecodeError("iqp-empty")
    return counter12, steps


# ── Stage 3: payload decoding ────────────────────────────────────────────────

def _s12(v: int) -> int:
    return v - 4096 if v >= 2048 else v


def _s8(v: int) -> int:
    return v - 256 if v >= 128 else v


def pct_to_iq(pct3: bytes) -> tuple[int, int]:
    """PCT 24-bit little-endian -> (I, Q) 12-bit signed."""
    v = int.from_bytes(pct3, "little")
    return _s12(v & 0xFFF), _s12(v >> 12)


def mode2_first_path(payload: bytes) -> tuple[int, int, int]:
    """(I, Q, tone_quality) of the 1st antenna path (extension slot ignored)."""
    i, q = pct_to_iq(payload[1:4])
    return i, q, payload[4]


# ── Stage 4: local <-> reflector pairing ─────────────────────────────────────

def _build_measurement(beacon, counter, iqls, peer_steps, t_done_ms) -> Measurement:
    """Merge the local subevents and the peer half. DecodeError if the mode
    sequences do not match step by step."""
    local_steps: list[Step] = []
    for _t, steps in iqls:
        local_steps.extend(steps)

    if [s.mode for s in local_steps] != [s.mode for s in peer_steps]:
        raise DecodeError("seq-misaligned")

    tones, rssi_loc, rssi_ref, freq_off = [], None, None, None
    rtt_pairs = []
    for loc, ref in zip(local_steps, peer_steps):
        if loc.mode == 0:
            if rssi_loc is None:
                rssi_loc = _s8(loc.payload[1])
                freq_off = int.from_bytes(loc.payload[3:5], "little")
            if rssi_ref is None and not ref.aborted:
                rssi_ref = _s8(ref.payload[1])
        elif loc.mode == 1 and not ref.aborted:
            # payload: quality(1) NADM(1) RSSI(1) ToX(2 LE signed) antenna(1)
            t_i = int.from_bytes(loc.payload[3:5], "little", signed=True)
            t_r = int.from_bytes(ref.payload[3:5], "little", signed=True)
            if t_i != -0x8000 and t_r != -0x8000:   # 0x8000 = unavailable
                rtt_pairs.append((t_i, t_r))
        elif loc.mode == 2 and not ref.aborted:
            i_l, q_l, tq_l = mode2_first_path(loc.payload)
            i_r, q_r, tq_r = mode2_first_path(ref.payload)
            tones.append(ToneIQ(loc.channel,
                                CS_FREQ_BASE_HZ + loc.channel * 1_000_000,
                                i_l, q_l, tq_l, i_r, q_r, tq_r))
    if rssi_loc is None or rssi_ref is None or not tones:
        raise DecodeError("measurement-incomplete")

    tones.sort(key=lambda t: t.channel)
    return Measurement(beacon=beacon, counter=counter, t_ms=iqls[0][0],
                       t_done_ms=t_done_ms, tones=tones, rssi_loc=rssi_loc,
                       rssi_ref=rssi_ref, freq_offset_raw=freq_off,
                       n_subevents=len(iqls), rtt_pairs=rtt_pairs)


class Pairer:
    """Accumulates the lines and produces the complete measurements.

    The firmware emits, per procedure: the IQLs (one per subevent) THEN the IQP
    of the same counter — so the arrival of the IQP triggers the assembly.
    Procedures whose one half was lost (corrupt line) are evicted when the
    beacon's counter has advanced by MAX_LAG."""

    MAX_LAG = 4

    def __init__(self, stats: DropStats):
        self.stats = stats
        self.pending: OrderedDict = OrderedDict()   # (beacon, counter) -> [iqls]

    def _evict_stale(self, beacon: int, counter: int):
        for key in list(self.pending):
            if key[0] == beacon and ((counter - key[1]) & 0xFFF) > self.MAX_LAG:
                del self.pending[key]
                self.stats.drop("peer-half-lost")

    def feed(self, line: str):
        """One UART line -> complete Measurement or None."""
        self.stats.lines += 1
        try:
            tag, beacon, counter, t_ms, data = parse_envelope(line)
            if tag == "IQL":
                steps = decode_local_steps(data)
                key = (beacon, counter & 0xFFF)
                self.pending.setdefault(key, []).append((t_ms, steps))
                self._evict_stale(beacon, counter & 0xFFF)
                return None

            counter12, peer_steps = decode_peer(data)
            if counter12 != counter & 0xFFF:
                raise DecodeError("iqp-counter-crossed")
            key = (beacon, counter12)
            iqls = self.pending.pop(key, None)
            if iqls is None:
                raise DecodeError("local-half-lost")
            m = _build_measurement(beacon, counter, iqls, peer_steps, t_ms)
            self.stats.measurements += 1
            return m
        except DecodeError as e:
            self.stats.drop(e.reason)
            return None


# ── Sources: file, serial port ───────────────────────────────────────────────

def measurements_from_lines(lines, stats: DropStats | None = None):
    """Generator of Measurement from an iterable of text lines."""
    pairer = Pairer(stats if stats is not None else DropStats())
    for line in lines:
        line = line.strip()
        if line:
            m = pairer.feed(line)
            if m is not None:
                yield m


def measurements_from_serial(port: str, baud: int, stats: DropStats | None = None):
    """Generator of Measurement from the data UART (automatic reconnection)."""
    import time
    import serial

    pairer = Pairer(stats if stats is not None else DropStats())
    while True:
        try:
            with serial.Serial(port, baud, timeout=0.5) as ser:
                buf = b""
                first = True
                while True:
                    buf += ser.read(4096)
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        if first:       # first partial line: ignore
                            first = False
                            continue
                        line = raw.decode("ascii", "replace").strip()
                        if line:
                            m = pairer.feed(line)
                            if m is not None:
                                yield m
        except serial.SerialException as e:
            print(f"[port lost] {e} — retrying in 2 s", file=sys.stderr)
            time.sleep(2)


# ── Control CLI ──────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Decode/clean the CS IQ stream")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="replay a file of raw UART lines")
    src.add_argument("--port", help="serial port (e.g. /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--stats", action="store_true",
                    help="print the drop summary at the end (--file) or "
                         "every 50 measurements (--port)")
    args = ap.parse_args()

    stats = DropStats()
    if args.file:
        f = open(args.file, encoding="utf-8", errors="replace")
        gen = measurements_from_lines(f, stats)
    else:
        gen = measurements_from_serial(args.port, args.baud, stats)

    try:
        for m in gen:
            print(f"b{m.beacon} rc={m.counter} t={m.t_ms}ms "
                  f"tones={len(m.tones)} rssi={m.rssi_loc}/{m.rssi_ref}dBm "
                  f"subevents={m.n_subevents}")
            if args.stats and args.port and stats.measurements % 50 == 0:
                print(stats.summary(), file=sys.stderr)
    except KeyboardInterrupt:
        pass
    if args.stats:
        print(stats.summary(), file=sys.stderr)


if __name__ == "__main__":
    main()
