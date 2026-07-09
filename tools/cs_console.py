#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CS Console — outil PC pour le banc PARNAV (nRF54L15, Channel Sounding).

Trois fonctions dans une fenêtre :
  1. Terminaux RTT (un onglet par carte : initiateur, réflecteurs) via pylink.
  2. Console UART vers l'initiateur : envoi des commandes du scheduler
     (ORDER:0,1,1 / AUTO / IQON / IQOFF) + affichage du flux reçu.
  3. Capture IQ : parse les lignes IQL/IQP émises par le firmware (commande
     IQON), décode les PCT I/Q des steps mode-2 locaux, et écrit un JSON.

Dépendances :  pip install pyserial pylink-square
  - pyserial : obligatoire (UART).
  - pylink-square : optionnel, seulement pour les onglets RTT (nécessite le
    driver SEGGER J-Link installé). Sans lui, l'UART et la capture IQ
    fonctionnent quand même.

Format des lignes IQ (voir IQ_DUMP.md) :
  IQL,<beacon>,<ranging_counter>,<t_ms>,<HEX>   un par SUBEVENT local, t_ms =
      k_uptime carte à l'arrivée du subevent (les steps d'un subevent
      partagent ce timestamp ; l'instant fin par step se reconstruit avec les
      durées de steps, Core Spec Vol 6 Part B §4.5.18).
  IQP,<beacon>,<ranging_counter>,<t_ms>,<HEX>   ranging data RAS du pair
      (format RAS brut, conservé en hex dans le JSON ; les steps du pair
      suivent le même ordre de canaux que les steps locaux).

HEX local = flux de steps HCI : {mode(1), canal(1), len(1), data(len)}…
  mode 2 : data = antenna_permutation(1) + N × [PCT(3) + tone_quality(1)]
  PCT 3 octets little-endian : I = 12 bits bas, Q = 12 bits hauts (signés).
"""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    import pylink
except ImportError:
    pylink = None

try:
    import iq_estimation  # estimation de distance live (numpy/scipy)
except ImportError:
    iq_estimation = None

RTT_DEVICE_DEFAULT = "NRF54L15_M33"   # nom J-Link de la cible, à ajuster
BAUD_DEFAULT = 115200                 # passer uart21 à 1 Mbaud si IQON continu


# ─────────────────────────────── décodage IQ ───────────────────────────────

def _s12(v):
    """12 bits signé -> int."""
    return v - 4096 if v & 0x800 else v


def parse_local_steps(hexstr):
    """Parse un flux de steps HCI local -> liste de dicts (IQ pour mode 2)."""
    data = bytes.fromhex(hexstr)
    steps, pos, idx = [], 0, 0
    while pos + 3 <= len(data):
        mode, chan, dlen = data[pos], data[pos + 1], data[pos + 2]
        pos += 3
        if pos + dlen > len(data):
            break  # tronqué
        payload = data[pos:pos + dlen]
        pos += dlen
        step = {"idx": idx, "mode": mode, "channel": chan,
                "raw": payload.hex().upper()}
        if mode == 2 and dlen >= 5:
            # antenna_permutation(1) + N x [PCT(3) + quality(1)]
            n_paths = (dlen - 1) // 4
            paths = []
            for p in range(n_paths):
                off = 1 + 4 * p
                pct = int.from_bytes(payload[off:off + 3], "little")
                paths.append({
                    "i": _s12(pct & 0xFFF),
                    "q": _s12((pct >> 12) & 0xFFF),
                    "tone_quality": payload[off + 3],
                })
            step["antenna_permutation"] = payload[0]
            step["paths"] = paths
        steps.append(step)
        idx += 1
    return steps


class IqCapture:
    """Accumule les lignes IQL/IQP et écrit le JSON."""

    def __init__(self):
        self.records = []
        self.enabled = False
        self.t0_host = None
        self.estimator = iq_estimation.IQEstimator() if iq_estimation else None
        self.last_estimate = None  # dict {beacon, bayesian, ...} ou None

    def feed_line(self, line):
        if not self.enabled:
            return False
        # Les trames binaires de la sérialisation ne finissent pas par \n :
        # elles préfixent la ligne IQ suivante. On cherche le tag n'importe où.
        idx = max(line.rfind("IQL,"), line.rfind("IQP,"))
        if idx < 0:
            return False
        line = line[idx:]
        try:
            tag, beacon, counter, t_ms, hexstr = line.split(",", 4)
            hexstr = "".join(c for c in hexstr.strip().upper()
                             if c in "0123456789ABCDEF")
            if len(hexstr) % 2:
                hexstr = hexstr[:-1]
            rec = {
                "type": "local_subevent" if tag == "IQL" else "peer_procedure",
                "beacon": int(beacon),
                "ranging_counter": int(counter),
                "t_board_ms": int(t_ms),
                "t_host": time.time(),
                "raw_hex": hexstr,
            }
            if tag == "IQL":
                rec["steps"] = parse_local_steps(hexstr)
            self.records.append(rec)
            if self.estimator:
                try:
                    r = self.estimator.feed(rec)
                    if r:
                        self.last_estimate = r
                except Exception:
                    pass  # l'estimation ne doit jamais casser la capture
            return True
        except (ValueError, IndexError):
            return False

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump({
                "note": ("t_board_ms = k_uptime carte a l'arrivee du subevent "
                         "(HCI). Steps d'un subevent = meme timestamp ; "
                         "reconstruction fine par durees de steps "
                         "(Vol 6 Part B 4.5.18). IQP = ranging data RAS du "
                         "pair, hex brut, meme ordre de canaux que le local."),
                "records": self.records,
            }, f, indent=1)


# ─────────────────────────────── console UART ──────────────────────────────

class UartPanel(ttk.Frame):
    def __init__(self, master, iq_capture):
        super().__init__(master)
        self.iq = iq_capture
        self.ser = None
        self.rxq = queue.Queue()
        self._rx_buf = b""
        self._build()
        self.after(50, self._drain)

    def _build(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=4, pady=4)
        ttk.Label(top, text="Port").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=18, values=self._ports())
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="↻", width=3,
                   command=lambda: self.port_cb.config(values=self._ports())
                   ).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(8, 0))
        self.baud_cb = ttk.Combobox(top, width=9, values=[
            "115200", "460800", "921600", "1000000"])
        self.baud_cb.set(str(BAUD_DEFAULT))
        self.baud_cb.pack(side="left", padx=4)
        self.conn_btn = ttk.Button(top, text="Connecter", command=self._toggle)
        self.conn_btn.pack(side="left", padx=8)

        self.txt = tk.Text(self, height=18, state="disabled",
                           bg="#101418", fg="#c8e6c9", font=("Consolas", 9))
        self.txt.pack(fill="both", expand=True, padx=4)

        send = ttk.Frame(self)
        send.pack(fill="x", padx=4, pady=4)
        self.entry = ttk.Entry(send)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda e: self._send())
        ttk.Button(send, text="Envoyer", command=self._send).pack(side="left",
                                                                  padx=4)
        quick = ttk.Frame(self)
        quick.pack(fill="x", padx=4, pady=(0, 4))
        for cmd in ("AUTO", "ORDER:0,1", "ORDER:0,1,1", "IQON", "IQOFF"):
            ttk.Button(quick, text=cmd, width=11,
                       command=lambda c=cmd: self._send(c)).pack(side="left",
                                                                 padx=2)

        # capture IQ
        iqf = ttk.LabelFrame(self, text="Capture IQ → JSON")
        iqf.pack(fill="x", padx=4, pady=4)
        self.iq_btn = ttk.Button(iqf, text="Démarrer capture",
                                 command=self._iq_toggle)
        self.iq_btn.pack(side="left", padx=4, pady=4)
        self.iq_lbl = ttk.Label(iqf, text="0 enregistrement")
        self.iq_lbl.pack(side="left", padx=8)
        self.iq_hide = tk.BooleanVar(value=True)
        ttk.Checkbutton(iqf, text="masquer les lignes IQ dans la console",
                        variable=self.iq_hide).pack(side="left", padx=8)
        ttk.Button(iqf, text="Sauver JSON…", command=self._iq_save
                   ).pack(side="right", padx=4)

    def _ports(self):
        if serial is None:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def _toggle(self):
        if self.ser:
            self.ser.close()
            self.ser = None
            self.conn_btn.config(text="Connecter")
            return
        if serial is None:
            messagebox.showerror("pyserial", "pip install pyserial")
            return
        try:
            self.ser = serial.Serial(self.port_cb.get(),
                                     int(self.baud_cb.get()), timeout=0.05)
        except Exception as e:
            messagebox.showerror("UART", str(e))
            return
        self.conn_btn.config(text="Déconnecter")
        threading.Thread(target=self._rx_thread, daemon=True).start()

    def _rx_thread(self):
        while self.ser:
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if chunk:
                self._rx_buf += chunk
                while b"\n" in self._rx_buf:
                    line, self._rx_buf = self._rx_buf.split(b"\n", 1)
                    self.rxq.put(line.decode("utf-8", "replace").rstrip("\r"))

    def _drain(self):
        try:
            while True:
                line = self.rxq.get_nowait()
                is_iq = self.iq.feed_line(line)
                if is_iq:
                    txt = f"{len(self.iq.records)} enregistrements"
                    est = self.iq.last_estimate
                    if est:
                        txt += (f"  |  b{est['beacon']}: "
                                f"{est['bayesian']:.2f} m (IQ, "
                                f"{est['n_tones']} tones)")
                    self.iq_lbl.config(text=txt)
                if not (is_iq and self.iq_hide.get()):
                    self._append(line + "\n")
        except queue.Empty:
            pass
        self.after(50, self._drain)

    def _append(self, text):
        self.txt.config(state="normal")
        self.txt.insert("end", text)
        if int(self.txt.index("end-1c").split(".")[0]) > 2000:
            self.txt.delete("1.0", "500.0")
        self.txt.see("end")
        self.txt.config(state="disabled")

    def _send(self, cmd=None):
        cmd = cmd if cmd is not None else self.entry.get().strip()
        if not cmd:
            return
        if not self.ser:
            messagebox.showwarning("UART", "Port non connecté")
            return
        self.ser.write((cmd + "\n").encode())
        self._append(f">>> {cmd}\n")
        if cmd is None:
            self.entry.delete(0, "end")

    def _iq_toggle(self):
        self.iq.enabled = not self.iq.enabled
        self.iq_btn.config(text="Arrêter capture" if self.iq.enabled
                           else "Démarrer capture")

    def _iq_save(self):
        if not self.iq.records:
            messagebox.showinfo("IQ", "Aucun enregistrement")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialfile=time.strftime("iq_capture_%Y%m%d_%H%M%S.json"))
        if path:
            self.iq.save(path)
            messagebox.showinfo(
                "IQ", f"{len(self.iq.records)} enregistrements → {path}")


# ─────────────────────────────── terminaux RTT ─────────────────────────────

class RttTab(ttk.Frame):
    def __init__(self, master, serial_no, device):
        super().__init__(master)
        self.serial_no = serial_no
        self.device = device
        self.q = queue.Queue()
        self.alive = True
        self.txt = tk.Text(self, state="disabled", bg="#0d1117",
                           fg="#e0e0e0", font=("Consolas", 9))
        self.txt.pack(fill="both", expand=True)
        threading.Thread(target=self._worker, daemon=True).start()
        self.after(50, self._drain)

    def _worker(self):
        try:
            jl = pylink.JLink()
            jl.open(serial_no=int(self.serial_no))
            jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
            jl.connect(self.device)
            jl.rtt_start()
            self.q.put(f"[RTT connecté : {self.serial_no} / {self.device}]\n")
            while self.alive:
                data = jl.rtt_read(0, 4096)
                if data:
                    self.q.put(bytes(data).decode("utf-8", "replace"))
                else:
                    time.sleep(0.03)
        except Exception as e:
            self.q.put(f"[RTT erreur : {e}]\n")

    def _drain(self):
        try:
            while True:
                self._append(self.q.get_nowait())
        except queue.Empty:
            pass
        if self.alive:
            self.after(50, self._drain)

    def _append(self, text):
        self.txt.config(state="normal")
        self.txt.insert("end", text)
        if int(self.txt.index("end-1c").split(".")[0]) > 3000:
            self.txt.delete("1.0", "1000.0")
        self.txt.see("end")
        self.txt.config(state="disabled")


class RttPanel(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=4, pady=4)
        ttk.Label(bar, text="Sonde (n° série)").pack(side="left")
        self.probe_cb = ttk.Combobox(bar, width=14, values=self._probes())
        self.probe_cb.pack(side="left", padx=4)
        ttk.Button(bar, text="↻", width=3,
                   command=lambda: self.probe_cb.config(values=self._probes())
                   ).pack(side="left")
        ttk.Label(bar, text="Cible").pack(side="left", padx=(8, 0))
        self.dev_entry = ttk.Entry(bar, width=16)
        self.dev_entry.insert(0, RTT_DEVICE_DEFAULT)
        self.dev_entry.pack(side="left", padx=4)
        ttk.Button(bar, text="+ Terminal RTT", command=self._add
                   ).pack(side="left", padx=8)
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=4, pady=4)
        if pylink is None:
            ttk.Label(self, foreground="orange", text=
                      "pylink-square absent : pip install pylink-square "
                      "(+ driver SEGGER J-Link) pour les terminaux RTT."
                      ).pack(pady=4)

    def _probes(self):
        if pylink is None:
            return []
        try:
            jl = pylink.JLink()
            return [str(e.SerialNumber) for e in
                    jl.connected_emulators()]
        except Exception:
            return []

    def _add(self):
        sn = self.probe_cb.get().strip()
        if not sn:
            messagebox.showwarning("RTT", "Choisir un numéro de série J-Link")
            return
        tab = RttTab(self.nb, sn, self.dev_entry.get().strip())
        self.nb.add(tab, text=f"RTT {sn}")
        self.nb.select(tab)


# ─────────────────────────────────── main ──────────────────────────────────

def main():
    root = tk.Tk()
    root.title("CS Console — PARNAV")
    root.geometry("1240x680")
    pw = ttk.PanedWindow(root, orient="horizontal")
    pw.pack(fill="both", expand=True)

    iq = IqCapture()
    uart = UartPanel(pw, iq)
    rtt = RttPanel(pw)
    left = ttk.LabelFrame(pw, text="UART initiateur (scheduler + IQ)")
    uart_container = uart  # déjà un Frame
    pw.add(uart_container, weight=1)
    pw.add(rtt, weight=2)
    _ = left

    root.mainloop()


if __name__ == "__main__":
    main()
