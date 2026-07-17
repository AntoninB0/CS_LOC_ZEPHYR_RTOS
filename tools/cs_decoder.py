#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cs_decoder.py — nettoyage + décodage du flux IQ de l'initiateur CS.

Entrée : lignes UART brutes (live ou fichier) au format documenté dans
IQ_DUMP.md :
    IQL,<beacon>,<counter>,<t_ms>,<HEX>   (une ligne PAR SUBEVENT local)
    IQP,<beacon>,<counter>,<t_ms>,<HEX>   (ranging data RAS du réflecteur)

Sortie : objets `Measurement` COMPLETS et VALIDÉS — les deux moitiés (locale
et réflecteur) de la même procédure, alignées canal par canal — prêts pour
l'estimation de distance. Tout ce qui est corrompu, incomplet ou incohérent
est écarté et compté (voir `DropStats`).

Usage bibliothèque (la fonction d'estimation est à la charge de l'appelant) :

    from cs_decoder import measurements_from_serial

    for m in measurements_from_serial("/dev/ttyACM0", 921600):
        d = estimate(m)         # <- ta fonction
        # m.tones : liste de ToneIQ triée par canal croissant
        #   t.freq_hz             fréquence de la tonalité (Hz)
        #   t.i_loc, t.q_loc      IQ mesuré par l'initiateur (12 bits signés)
        #   t.i_ref, t.q_ref      IQ mesuré par le réflecteur
        #   t.tq_loc, t.tq_ref    tone quality (0 = bon)
        # m.rssi_loc / m.rssi_ref (dBm), m.t_ms, m.beacon, m.counter

Usage CLI (contrôle du décodage, sans estimation) :
    python cs_decoder.py --file capture_raw.txt --stats
    python cs_decoder.py --port /dev/ttyACM0 --baud 921600 --stats
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter, OrderedDict
from dataclasses import dataclass, field

# ── Constantes protocole ─────────────────────────────────────────────────────

CS_CHANNEL_MAX = 78              # canaux CS valides : 0..78 (2402..2480 MHz)
CS_FREQ_BASE_HZ = 2_402_000_000  # canal k -> 2402 + k MHz

# Tailles de data par mode, côté LOCAL (HCI, rôle initiateur).
# mode 2 : 1 + (n_chemins+1)*4 -> tailles admises pour 1..4 chemins d'antenne.
LOCAL_MODE0_LEN = 5
LOCAL_MODE1_LENS = (6, 14)       # 14 = avec sounding PCT (non utilisé ici)
LOCAL_MODE2_LENS = (9, 13, 17, 21)

RAS_RANGING_HEADER_LEN = 4       # counter(12b)+config(4b), tx_power, aa_mask
RAS_SUBEVENT_HEADER_LEN = 8      # start_evt(2) freq_comp(2) done(1) abort(1)
                                 # ref_power(1) num_steps(1)


class DecodeError(ValueError):
    """Ligne/payload structurellement invalide. `reason` alimente les stats."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


# ── Structures ───────────────────────────────────────────────────────────────

@dataclass
class Step:
    mode: int
    channel: int          # -1 côté réflecteur (le RAS n'émet pas le canal)
    payload: bytes
    aborted: bool = False


@dataclass
class ToneIQ:
    """Une tonalité PBR appariée : les deux moitiés du même step mode 2."""
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
    """Une procédure CS complète, validée et alignée."""
    beacon: int
    counter: int
    t_ms: int                    # k_uptime carte à l'arrivée du 1er subevent
    t_done_ms: int               # k_uptime carte à la fin de procédure (IQP)
    tones: list[ToneIQ]          # triées par canal croissant
    rssi_loc: int                # dBm, 1er step mode 0 local (127 = indispo)
    rssi_ref: int                # dBm, 1er step mode 0 réflecteur (127 = indispo)
    freq_offset_raw: int         # offset fréquence mode 0 local (brut, 16 bits)
    n_subevents: int             # nb de lignes IQL agrégées
    # Steps RTT (mode 1) : paires (ToA-ToD initiateur, ToD-ToA réflecteur) en
    # unités de 0,5 ns. Temps de vol τ = (Ti - Tr)/2 -> d = (Ti - Tr)·0,075 m.
    # Grossier (~mètres) mais SANS repliement : sert à lever l'ambiguïté 50 m
    # de la phase. Steps au sentinel 0x8000 (indisponible) déjà écartés.
    rtt_pairs: list[tuple[int, int]] = field(default_factory=list)


@dataclass
class DropStats:
    """Comptage de tout ce qui n'a pas fini en Measurement."""
    lines: int = 0
    measurements: int = 0
    dropped: Counter = field(default_factory=Counter)

    def drop(self, reason: str):
        self.dropped[reason] += 1

    def summary(self) -> str:
        total_drop = sum(self.dropped.values())
        out = [f"{self.lines} lignes -> {self.measurements} mesures complètes, "
               f"{total_drop} rejets"]
        for reason, n in self.dropped.most_common():
            out.append(f"  {reason}: {n}")
        return "\n".join(out)


# ── Étage 1 : enveloppe ASCII ────────────────────────────────────────────────

def parse_envelope(line: str):
    """'IQL,b,rc,t,HEX' -> (tag, beacon, counter, t_ms, bytes). DecodeError sinon."""
    parts = line.split(",", 4)
    if len(parts) != 5 or parts[0] not in ("IQL", "IQP"):
        raise DecodeError("enveloppe")
    try:
        beacon, counter, t_ms = int(parts[1]), int(parts[2]), int(parts[3])
    except ValueError:
        raise DecodeError("enveloppe") from None
    hexstr = parts[4].strip()
    if not hexstr or len(hexstr) % 2:
        raise DecodeError("hex-impair")
    try:
        data = bytes.fromhex(hexstr)
    except ValueError:
        raise DecodeError("hex-invalide") from None
    # fromhex tolère les espaces : les refuser explicitement
    if " " in hexstr:
        raise DecodeError("hex-invalide")
    return parts[0], beacon, counter, t_ms, data


# ── Étage 2a : steps locaux (format HCI : mode|canal|len|data) ───────────────

def decode_local_steps(data: bytes) -> list[Step]:
    steps, pos = [], 0
    while pos + 3 <= len(data):
        mode, chan, dlen = data[pos], data[pos + 1], data[pos + 2]
        if chan > CS_CHANNEL_MAX:
            raise DecodeError("iql-canal")
        if (mode == 0 and dlen != LOCAL_MODE0_LEN) or \
           (mode == 1 and dlen not in LOCAL_MODE1_LENS) or \
           (mode == 2 and dlen not in LOCAL_MODE2_LENS) or mode > 2:
            raise DecodeError("iql-mode-len")
        payload = data[pos + 3:pos + 3 + dlen]
        if len(payload) < dlen:
            raise DecodeError("iql-tronque")
        steps.append(Step(mode, chan, payload))
        pos += 3 + dlen
    if pos != len(data):
        raise DecodeError("iql-reliquat")
    if not steps:
        raise DecodeError("iql-vide")
    return steps


# ── Étage 2b : ranging data RAS du réflecteur ────────────────────────────────

def _peer_step_sizes(aa_mask: int) -> dict[int, int]:
    n_ap = bin(aa_mask).count("1")
    if not 1 <= n_ap <= 4:
        raise DecodeError("iqp-aa-mask")
    return {0: 3, 1: 6, 2: 1 + (n_ap + 1) * 4}


def decode_peer(data: bytes):
    """RAS ranging data -> (counter12, [Step...]). Structure :
    en-tête ranging (4 o), puis par subevent : en-tête (8 o) + num_steps steps
    'mode(1)|data', SANS canal ni champ len (taille implicite du mode).
    Un step au bit 7 du mode levé est avorté et ne porte pas de data."""
    if len(data) < RAS_RANGING_HEADER_LEN + RAS_SUBEVENT_HEADER_LEN:
        raise DecodeError("iqp-court")
    counter12 = int.from_bytes(data[0:2], "little") & 0x0FFF
    aa_mask = data[3]
    sizes = _peer_step_sizes(aa_mask)

    steps, pos = [], RAS_RANGING_HEADER_LEN
    while pos < len(data):
        if pos + RAS_SUBEVENT_HEADER_LEN > len(data):
            raise DecodeError("iqp-header-tronque")
        num_steps = data[pos + 7]
        pos += RAS_SUBEVENT_HEADER_LEN
        for _ in range(num_steps):
            if pos >= len(data):
                raise DecodeError("iqp-steps-manquants")
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
                raise DecodeError("iqp-step-tronque")
            steps.append(Step(mode, -1, payload))
            pos += sizes[mode]
    if pos != len(data):
        raise DecodeError("iqp-reliquat")
    if not steps:
        raise DecodeError("iqp-vide")
    return counter12, steps


# ── Étage 3 : décodage des payloads ──────────────────────────────────────────

def _s12(v: int) -> int:
    return v - 4096 if v >= 2048 else v


def _s8(v: int) -> int:
    return v - 256 if v >= 128 else v


def pct_to_iq(pct3: bytes) -> tuple[int, int]:
    """PCT 24 bits little-endian -> (I, Q) 12 bits signés."""
    v = int.from_bytes(pct3, "little")
    return _s12(v & 0xFFF), _s12(v >> 12)


def mode2_first_path(payload: bytes) -> tuple[int, int, int]:
    """(I, Q, tone_quality) du 1er chemin d'antenne (slot d'extension ignoré)."""
    i, q = pct_to_iq(payload[1:4])
    return i, q, payload[4]


# ── Étage 4 : appariement local <-> réflecteur ───────────────────────────────

def _build_measurement(beacon, counter, iqls, peer_steps, t_done_ms) -> Measurement:
    """Fusionne les subevents locaux et la moitié pair. DecodeError si les
    séquences de modes ne se correspondent pas step à step."""
    local_steps: list[Step] = []
    for _t, steps in iqls:
        local_steps.extend(steps)

    if [s.mode for s in local_steps] != [s.mode for s in peer_steps]:
        raise DecodeError("seq-desalignee")

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
            # payload : quality(1) NADM(1) RSSI(1) ToX(2 LE signé) antenne(1)
            t_i = int.from_bytes(loc.payload[3:5], "little", signed=True)
            t_r = int.from_bytes(ref.payload[3:5], "little", signed=True)
            if t_i != -0x8000 and t_r != -0x8000:   # 0x8000 = indisponible
                rtt_pairs.append((t_i, t_r))
        elif loc.mode == 2 and not ref.aborted:
            i_l, q_l, tq_l = mode2_first_path(loc.payload)
            i_r, q_r, tq_r = mode2_first_path(ref.payload)
            tones.append(ToneIQ(loc.channel,
                                CS_FREQ_BASE_HZ + loc.channel * 1_000_000,
                                i_l, q_l, tq_l, i_r, q_r, tq_r))
    if rssi_loc is None or rssi_ref is None or not tones:
        raise DecodeError("mesure-incomplete")

    tones.sort(key=lambda t: t.channel)
    return Measurement(beacon=beacon, counter=counter, t_ms=iqls[0][0],
                       t_done_ms=t_done_ms, tones=tones, rssi_loc=rssi_loc,
                       rssi_ref=rssi_ref, freq_offset_raw=freq_off,
                       n_subevents=len(iqls), rtt_pairs=rtt_pairs)


class Pairer:
    """Accumule les lignes et produit les mesures complètes.

    Le firmware émet, par procédure : les IQL (un par subevent) PUIS l'IQP du
    même compteur — l'arrivée de l'IQP déclenche donc l'assemblage. Les
    procédures dont une moitié s'est perdue (ligne corrompue) sont évincées
    quand le compteur du beacon a avancé de MAX_LAG."""

    MAX_LAG = 4

    def __init__(self, stats: DropStats):
        self.stats = stats
        self.pending: OrderedDict = OrderedDict()   # (beacon, counter) -> [iqls]

    def _evict_stale(self, beacon: int, counter: int):
        for key in list(self.pending):
            if key[0] == beacon and ((counter - key[1]) & 0xFFF) > self.MAX_LAG:
                del self.pending[key]
                self.stats.drop("moitie-pair-perdue")

    def feed(self, line: str):
        """Une ligne UART -> Measurement complet ou None."""
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
                raise DecodeError("iqp-counter-croise")
            key = (beacon, counter12)
            iqls = self.pending.pop(key, None)
            if iqls is None:
                raise DecodeError("moitie-locale-perdue")
            m = _build_measurement(beacon, counter, iqls, peer_steps, t_ms)
            self.stats.measurements += 1
            return m
        except DecodeError as e:
            self.stats.drop(e.reason)
            return None


# ── Sources : fichier, port série ────────────────────────────────────────────

def measurements_from_lines(lines, stats: DropStats | None = None):
    """Générateur de Measurement depuis un itérable de lignes texte."""
    pairer = Pairer(stats if stats is not None else DropStats())
    for line in lines:
        line = line.strip()
        if line:
            m = pairer.feed(line)
            if m is not None:
                yield m


def measurements_from_serial(port: str, baud: int, stats: DropStats | None = None):
    """Générateur de Measurement depuis l'UART data (reconnexion automatique)."""
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
                        if first:       # première ligne partielle : ignorer
                            first = False
                            continue
                        line = raw.decode("ascii", "replace").strip()
                        if line:
                            m = pairer.feed(line)
                            if m is not None:
                                yield m
        except serial.SerialException as e:
            print(f"[port perdu] {e} — retente dans 2 s", file=sys.stderr)
            time.sleep(2)


# ── CLI de contrôle ──────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Décodage/nettoyage du flux IQ CS")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="rejouer un fichier de lignes UART brutes")
    src.add_argument("--port", help="port série (ex. /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--stats", action="store_true",
                    help="afficher le bilan des rejets à la fin (--file) ou "
                         "toutes les 50 mesures (--port)")
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
