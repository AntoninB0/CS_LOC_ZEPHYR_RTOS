#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_console.py — control console for the CS initiator: SENDS the commands
(IQON, IQOFF, AUTO, ORDER:0,1,2...) and DISPLAYS the decoded measurements
continuously, with the distance estimated by YOUR `estimate()` function
(tools/estimation.py).

Chain: UART -> cs_decoder (validation, local/reflector pairing)
     -> complete Measurement -> estimation.estimate(m) -> display.

⚠ The DK has TWO VCOMs: Serial Port 0 (uart30 = data + commands, this script)
and Serial Port 1 (uart20 = Zephyr logs). Connect to the right one. `--list` to
see the ports. One process at a time on the port (including cs_decoder --port).

Usage:
  python uart_console.py --port /dev/ttyACM0                # 921600 by default
  python uart_console.py --file capture_ref_921600.txt      # offline replay
  python uart_console.py --list

Commands once running (Enter to submit):
  IQON / IQOFF / AUTO / ORDER:0,1,2   sent to the firmware
  :test [s]     collect for s seconds (default 30) then print a per-beacon
                report and create a timestamped FOLDER test_YYYYMMDD_HHMMSS/
                containing: the JSON (data+report, per-tone IQ included ->
                re-analyzable), the PNG plot, config.txt (firmware config
                snapshot) and estimateur/ (IFFT + Bayesian figures of the
                typical and outlier measurement of each beacon, to illustrate
                uncertainty/multipath).
  :cal [b] [m]  show / set the calibration offsets (see estimation.py)
  :show/:nshow  show / hide the per-measurement line
  :raw          also show the raw UART lines (toggle)
  :stats        decoding summary (measurements, drops by reason)
  :reload       reload estimation.py without restarting (hot dev)
  :q  or Ctrl-D quit

Script mode (non-interactive): --test N with --port -> collect N seconds,
create the timestamped folder (JSON+IQ, PNG, config.txt, estimateur/ figures),
then quit. Handy for calibration runs.

Dependency: pyserial (venv: see install.txt). The estimation function lives in
tools/estimation.py — see the contract in that file.
"""

import argparse
import importlib
import json
import os
import statistics
import sys
import threading
import time

from cs_decoder import DropStats, Pairer

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing. Activate the venv and install (see install.txt):\n"
             "  cd tools && source .venv/bin/activate && pip install -r install.txt")


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial port detected (board plugged in?).", file=sys.stderr)
    for p in ports:
        print(f"{p.device}\t{p.description}", file=sys.stderr)


def load_estimate():
    """Import (or re-import) estimation.estimate. None if unavailable."""
    try:
        import estimation
        importlib.reload(estimation)
        fn = getattr(estimation, "estimate", None)
        if fn is None:
            print("[estimation.py found but no estimate(m) function]",
                  file=sys.stderr, flush=True)
        return fn
    except Exception as e:
        print(f"[estimation.py not loaded: {type(e).__name__}: {e}]",
              file=sys.stderr, flush=True)
        return None


def write_config_snapshot(path):
    """Write to `path` a .txt concatenating the CURRENT firmware config
    (cs_config.h + prj.conf + precision overlays + reflector/prj.conf), so each
    test keeps the exact trace of the config it was run with. Best-effort:
    missing files are noted, never fatal. Paths are relative to this script's
    location (not the cwd)."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)                       # tools/ -> repo root
    rel_files = [
        "initiator/src/cs_config.h",                   # profile + N (the 2 knobs)
        "initiator/prj.conf",                          # base SDC reservations
        "initiator/overlay-precision-multi.conf",      # boat/indoor N>=2
        "initiator/overlay-precision-single.conf",     # boat/indoor N==1
        "reflector/prj.conf",                          # reflector-side CS_EVENT_LEN
    ]
    out = [f"# Firmware config snapshot — {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
           "# (the actually compiled overlay depends on the build.sh flag: "
           "--boat-n = multi, --boat-1 = single, drone = none)\n"]
    for rel in rel_files:
        out.append("\n" + "=" * 78 + f"\n# {rel}\n" + "=" * 78 + "\n")
        try:
            with open(os.path.join(repo, rel), encoding="utf-8", errors="replace") as f:
                out.append(f.read())
        except OSError as e:
            out.append(f"[not found: {e}]\n")
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(out)


class Display:
    """Pushes each UART line into the decoder; displays each complete
    measurement, augmented with the distance returned by estimate(m). No
    exception from the user function must kill the reader thread: everything is
    caught, and the error is printed only on its first change (not 17x per
    second)."""

    def __init__(self):
        self.stats = DropStats()
        self.pairer = Pairer(self.stats)
        self.estimate = load_estimate()
        self.show_raw = False
        self._last_err = None
        self.test_until = None      # time.time() deadline of the current test, else None
        self.test_secs = 0
        self.test_data = {}         # {beacon: [record dicts]}
        self.test_done = threading.Event()  # set when report+JSON+PNG written
        self.show_distances = False # per-measurement line hidden by default (:show)
        self.note = ""              # config label of the test (--note / :note),
                                    # written into the JSON -> labels the comparisons

    def start_test(self, secs: int):
        self.test_data = {}
        self.test_secs = secs
        self.test_done.clear()
        self.test_until = time.time() + secs
        print(f"[test started: {secs} s of collection, report at the end]",
              file=sys.stderr, flush=True)

    def _build_report(self):
        """Per-beacon statistics over the collected distances.
        NOTE: the dict keys below are the archived JSON schema (also present in
        the existing test files), kept stable on purpose — do not rename."""
        report = {}
        for b, recs in sorted(self.test_data.items()):
            ds = sorted(r["d_m"] for r in recs)
            n = len(ds)
            med = statistics.median(ds)
            rssis = [r["rssi_loc"] for r in recs if r["rssi_loc"] != 127]
            # real sampling rate, on the board clock
            span_ms = recs[-1]["t_ms"] - recs[0]["t_ms"]
            freq = round((n - 1) * 1000.0 / span_ms, 2) if n > 1 and span_ms > 0 else None
            report[f"b{b}"] = {
                "n": n,
                "freq_hz": freq,
                "mediane_m": round(med, 3),
                "moyenne_m": round(statistics.fmean(ds), 3),
                "sigma_m": round(statistics.pstdev(ds), 3) if n > 1 else 0.0,
                "min_m": round(ds[0], 3),
                "max_m": round(ds[-1], 3),
                "iqr_m": [round(ds[n // 4], 3), round(ds[(3 * n) // 4], 3)],
                "outliers_sup_1m": sum(1 for v in ds if abs(v - med) > 1.0),
                "rssi_moyen_dbm": round(statistics.fmean(rssis), 1) if rssis else None,
            }
        return report

    def _finish_test(self):
        """End of window: print the report and save data+report as JSON."""
        report = self._build_report()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        day = stamp[:8]                             # YYYYMMDD
        # All tests go under <repo>/tests/YYYYMMDD/ regardless of the launch
        # directory (path anchored on the script location).
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        outdir = os.path.join(repo, "tests", day, f"test_{stamp}")
        os.makedirs(outdir, exist_ok=True)
        fname = os.path.join(outdir, f"test_{stamp}.json")
        try:      # traceability: calibration offsets active during this test
            import estimation
            cal = {"global_m": getattr(estimation, "CAL_OFFSET_M", 0.0),
                   "par_beacon": getattr(estimation, "CAL_OFFSET_PER_BEACON", {})}
        except ImportError:
            cal = None
        doc = {
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": self.note,                    # config label (compare tag)
            "duree_s": self.test_secs,
            "calibration": cal,
            "rapport": report,
            "mesures": {f"b{b}": recs for b, recs in sorted(self.test_data.items())},
        }
        with open(fname, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)

        print(f"\n=== REPORT ({self.test_secs} s) ===", flush=True)
        if not report:
            print("no measurement collected!", flush=True)
        for name, r in report.items():
            print(f"{name}: n={r['n']:3d} @ {r['freq_hz']} Hz  "
                  f"median={r['mediane_m']:6.2f} m  "
                  f"IQR=[{r['iqr_m'][0]:.2f}..{r['iqr_m'][1]:.2f}]  "
                  f"sigma={r['sigma_m']:.2f}  min/max={r['min_m']:.2f}/{r['max_m']:.2f}  "
                  f"outliers>1m={r['outliers_sup_1m']}  "
                  f"rssi={r['rssi_moyen_dbm']} dBm", flush=True)
        cfg = os.path.join(outdir, "config.txt")
        try:                                        # exact config of the test
            write_config_snapshot(cfg)
        except OSError as e:
            print(f"[config snapshot failed: {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
            cfg = None
        png = self._plot(fname.replace(".json", ".png"))
        est_figs = self._plot_estimator(outdir)     # IFFT/Bayesian per beacon
        cons_figs = self._plot_consistency(outdir)  # IQ consistency report per beacon
        contents = ["test_%s.json" % stamp]
        if png:
            contents.append("test_%s.png" % stamp)
        if cfg:
            contents.append("config.txt")
        if est_figs:
            contents.append(f"estimateur/ ({len(est_figs)} fig)")
        if cons_figs:
            contents.append(f"consistency_b* ({len(cons_figs)})")
        short = os.path.join("tests", day, f"test_{stamp}")   # short display
        print(f"[folder -> {short}/ : " + ", ".join(contents) + "]\n", flush=True)

    def _plot_estimator(self, outdir):
        """IFFT + Bayesian likelihood figures (uncertainty / multipath) for the
        TYPICAL and the most OUTLIER measurement of each beacon. Reuses
        viz_estimateur (same functions as the estimator). Best-effort: any error
        is caught, the report is still written. Returns the list of PNGs."""
        try:
            import numpy as np
            import viz_estimateur as vz
        except Exception as e:
            print(f"[estimator figures unavailable: {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
            return []
        made = []
        subdir = os.path.join(outdir, "estimateur")
        for b, recs in sorted(self.test_data.items()):
            cand = [r for r in recs if r.get("iq") and r.get("d_brut_m") is not None]
            if not cand:
                continue
            med = float(np.median([r["d_brut_m"] for r in cand]))
            typ = min(cand, key=lambda r: abs(r["d_brut_m"] - med))
            wor = max(cand, key=lambda r: abs(r["d_brut_m"] - med))
            picks = [("typical", typ)] + ([("outlier", wor)] if wor is not typ else [])
            for tag, r in picks:
                try:
                    os.makedirs(subdir, exist_ok=True)
                    png = os.path.join(subdir, f"b{b}_{tag}_rc{r['counter']}.png")
                    if vz.plot_measurement(vz.meas_from_record(b, r), png) is not None:
                        made.append(png)
                except Exception as e:
                    print(f"[estimator fig b{b} {tag} failed: "
                          f"{type(e).__name__}: {e}]", file=sys.stderr, flush=True)
        return made

    def _plot_consistency(self, outdir):
        """IQ consistency report (per-channel coherence, top-X%, R-vs-amplitude)
        per beacon, written into the test folder. Reuses iq_consistency.
        Best-effort. Returns the list of PNGs (consistency_bX.png)."""
        try:
            import iq_consistency as ic
        except Exception as e:
            print(f"[consistency report unavailable: {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
            return []
        label = self.note or f"test_{time.strftime('%Y%m%d_%H%M%S')}"
        results, made = {}, []
        for b, recs in self.test_data.items():
            results[(label, b)] = ic.consistency(recs)
        for b in sorted(self.test_data):
            try:
                out = os.path.join(outdir, f"consistency_b{b}.png")
                ic.report_figure(results, [label], b, out)
                made.append(out)
            except Exception as e:
                print(f"[consistency report b{b} failed: "
                      f"{type(e).__name__}: {e}]", file=sys.stderr, flush=True)
        return made

    def _plot(self, fname):
        """PNG: RAW distance (points, before the sliding median) and smoothed
        (line) of each beacon over time. None if matplotlib is missing."""
        try:
            import matplotlib
            matplotlib.use("Agg")             # headless: file only
            import matplotlib.pyplot as plt
        except ImportError:
            print("[matplotlib missing: no plot (pip install matplotlib)]",
                  file=sys.stderr, flush=True)
            return None
        beacons = sorted(self.test_data)
        if not beacons:
            return None
        t0 = min(recs[0]["t_ms"] for recs in self.test_data.values())
        fig, axes = plt.subplots(len(beacons), 1, sharex=True, squeeze=False,
                                 figsize=(10, 2.6 * len(beacons)))
        for ax, b in zip(axes[:, 0], beacons):
            recs = self.test_data[b]
            t = [(r["t_ms"] - t0) / 1000.0 for r in recs]
            pts = [(ti, r["d_brut_m"]) for ti, r in zip(t, recs)
                   if r.get("d_brut_m") is not None]
            if pts:
                ax.plot(*zip(*pts), ".", ms=4, label="raw")
            ax.plot(t, [r["d_m"] for r in recs], "-", lw=1.2, alpha=0.75,
                    label="smoothed (median 7)")
            ax.set_ylabel(f"b{b}  (m)")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
        axes[-1, 0].set_xlabel("time (s)")
        fig.suptitle(f"Distance per beacon — test {self.test_secs} s")
        fig.tight_layout()
        fig.savefig(fname, dpi=110)
        plt.close(fig)
        return fname

    def on_line(self, line: str):
        if self.show_raw:
            print(f"< {line}", flush=True)
        m = self.pairer.feed(line)
        if m is None:
            return

        d = None
        d_raw = None
        if self.estimate is not None:
            try:
                d = self.estimate(m)
                self._last_err = None
                try:      # value BEFORE the sliding median, exposed by estimation.py
                    import estimation
                    d_raw = estimation.LAST_RAW
                except ImportError:
                    pass
            except Exception as e:
                msg = f"{type(e).__name__}: {e}"
                if msg != self._last_err:
                    self._last_err = msg
                    print(f"[estimate() -> {msg}]", file=sys.stderr, flush=True)

        dtxt = f"d={d:6.2f} m" if isinstance(d, (int, float)) else "d=   ?   "
        if self.show_distances:
            print(f"b{m.beacon} rc={m.counter:4d}  {dtxt}  | {len(m.tones):2d} tones  "
                  f"rssi={m.rssi_loc}/{m.rssi_ref} dBm  t={m.t_ms} ms", flush=True)
        if self.test_until is not None and time.time() < self.test_until \
                and d is not None:
            self.test_data.setdefault(m.beacon, []).append({
                "counter": m.counter,
                "t_ms": m.t_ms,                       # k_uptime 1st subevent (ms)
                "t_done_ms": getattr(m, "t_done_ms", None),   # procedure end
                "n_subevents": getattr(m, "n_subevents", None),
                # -> procedure duration = t_done_ms - t_ms  (the logs' `meas`)
                "d_m": round(float(d), 4),
                "d_brut_m": round(d_raw, 4) if d_raw is not None else None,
                "rssi_loc": m.rssi_loc,
                "rssi_ref": m.rssi_ref,
                "tones": len(m.tones),
                # per-tone IQ -> the JSON stays re-analyzable (offline IFFT/
                # Bayesian figures, viz_estimateur.py --json). Compact:
                # [channel, i_loc, q_loc, i_ref, q_ref, tq_loc, tq_ref].
                "iq": [[t.channel, t.i_loc, t.q_loc, t.i_ref, t.q_ref,
                        t.tq_loc, t.tq_ref] for t in m.tones],
            })
        self.poll_test()

    def poll_test(self):
        """Fire the report at the deadline, EVEN if the stream is dead (also
        called by the reader thread on every 0.2 s tick). Protected: an
        exception in the report/plot must not kill the reader thread."""
        if self.test_until is None or time.time() < self.test_until:
            return
        self.test_until = None
        try:
            self._finish_test()
        except Exception as e:
            print(f"[report failed: {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
        finally:
            # signal the --test mode that report+JSON+PNG are written: it can
            # close the process without interrupting the plot (race resolved).
            self.test_done.set()


def reader_loop(ser, stop, disp: Display):
    """Thread: reads the UART, reassembles the lines, feeds the decoder."""
    buf = b""
    first = True
    while not stop.is_set():
        try:
            chunk = ser.read(4096)
        except serial.SerialException:
            print("\n[port lost — reading stopped]", file=sys.stderr, flush=True)
            stop.set()
            return
        if not chunk:
            disp.poll_test()          # :test deadline watched even without stream
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            if first:                     # first partial line: ignore
                first = False
                continue
            line = raw.decode("ascii", "replace").strip()
            if line:
                disp.on_line(line)


def replay_file(path: str, disp: Display):
    """Offline mode: replay a raw capture through the same pipeline."""
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line:
                disp.on_line(line)
    print(disp.stats.summary(), file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser(
        description="CS UART console: commands + decoded measurements + estimation")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port (e.g. /dev/ttyACM0, COM5)")
    src.add_argument("--file", help="replay a raw capture (offline)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate (default 921600 = overlay current-speed)")
    ap.add_argument("--list", action="store_true", help="list the ports and quit")
    ap.add_argument("--test", type=int, metavar="N",
                    help="script mode: collect N s -> timestamped folder "
                         "(JSON+IQ, PNG, config.txt, estimateur/ figures), then quit")
    ap.add_argument("--note", default="", metavar="LABEL",
                    help="config label of the test (e.g. 'boat_thin1_5m_N1') "
                         "written into the JSON -> labels the comparison curves")
    args = ap.parse_args()


    if args.list:
        list_serial_ports()
        return
    if args.file:
        replay_file(args.file, Display())
        return
    if not args.port:
        ap.error("--port or --file required (or --list to see the ports)")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    disp = Display()
    disp.note = args.note                         # config label (--note)
    print(f"[connected to {args.port} @ {args.baud} baud]  one line per measurement  |  "
          f"IQON/IQOFF/AUTO/ORDER for the firmware  |  :raw :stats :reload :q",
          file=sys.stderr, flush=True)

    stop = threading.Event()
    rx = threading.Thread(target=reader_loop, args=(ser, stop, disp), daemon=True)
    rx.start()

    # Script mode --test N: no interactive loop — collect, report, exit.
    # We wait on test_done (set AFTER the report/JSON/PNG are written by the
    # reader thread): otherwise we would close the process during the plot,
    # killing the daemon thread and losing the PNG. +10 s guard if the stream
    # stops.
    if args.test:
        disp.start_test(args.test)
        try:
            if not disp.test_done.wait(timeout=args.test + 10):
                print("[stream interrupted before the end of the test]", file=sys.stderr)
        except KeyboardInterrupt:
            pass
        stop.set()
        ser.close()
        return

    try:
        while not stop.is_set():
            try:
                cmd = input()
            except EOFError:              # Ctrl-D
                break
            cmd = cmd.strip()
            if not cmd:
                continue
            if cmd in (":q", ":quit", ":exit"):
                break
            if cmd.split()[0] == ":test":
                parts = cmd.split()
                try:
                    secs = int(parts[1]) if len(parts) > 1 else 30
                except ValueError:
                    print("[usage: :test <seconds>]", file=sys.stderr, flush=True)
                    continue
                disp.start_test(secs)
                continue
            if cmd.split()[0] == ":cal":
                # :cal            -> show the current offsets
                # :cal 0.35       -> global offset (m, subtracted from the output)
                # :cal 2 -0.10    -> additional offset of beacon 2
                try:
                    import estimation
                    parts = cmd.split()
                    if len(parts) == 2:
                        estimation.CAL_OFFSET_M = float(parts[1])
                    elif len(parts) == 3:
                        estimation.CAL_OFFSET_PER_BEACON[int(parts[1])] = float(parts[2])
                    print(f"[cal: global={estimation.CAL_OFFSET_M:+.3f} m, "
                          f"per beacon={estimation.CAL_OFFSET_PER_BEACON} "
                          f"(⚠ :reload resets them to the file values)]",
                          file=sys.stderr, flush=True)
                except (ValueError, ImportError):
                    print("[usage: :cal | :cal <offset_m> | :cal <beacon> <offset_m>]",
                          file=sys.stderr, flush=True)
                continue
            if cmd.split()[0] == ":note":
                disp.note = cmd[len(":note"):].strip()
                print(f"[test note = '{disp.note}']", file=sys.stderr, flush=True)
                continue
            if cmd == ":show":
                disp.show_distances = True
                continue
            if cmd == ":nshow":
                disp.show_distances = False
                continue
            if cmd == ":raw":
                disp.show_raw = not disp.show_raw
                print(f"[raw display {'ON' if disp.show_raw else 'OFF'}]",
                      file=sys.stderr, flush=True)
                continue
            if cmd == ":stats":
                print(disp.stats.summary(), file=sys.stderr, flush=True)
                continue
            if cmd == ":reload":
                disp.estimate = load_estimate()
                print(f"[estimation {'reloaded' if disp.estimate else 'missing'}]",
                      file=sys.stderr, flush=True)
                continue
            ser.write((cmd + "\n").encode("ascii"))
            ser.flush()
            print(f"> {cmd}", file=sys.stderr, flush=True)
    except KeyboardInterrupt:              # Ctrl-C
        pass
    finally:
        stop.set()
        time.sleep(0.25)
        ser.close()
        print("\n[closed]", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
