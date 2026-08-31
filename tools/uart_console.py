#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_console.py — poste de pilotage de l'initiateur CS : ENVOIE les commandes
(IQON, IQOFF, AUTO, ORDER:0,1,2…) et AFFICHE les mesures décodées en continu,
avec la distance estimée par TA fonction `estimate()` (tools/estimation.py).

Chaîne : UART → cs_decoder (validation, appariement local/réflecteur)
       → Measurement complet → estimation.estimate(m) → affichage.

⚠ Le DK a DEUX VCOM : Serial Port 0 (uart30 = data + commandes, ce script) et
Serial Port 1 (uart20 = logs Zephyr). Connecte-toi au bon. `--list` pour voir
les ports. Un seul process à la fois sur le port (cs_decoder --port compris).

Usage :
  python uart_console.py --port /dev/ttyACM0                # 921600 par défaut
  python uart_console.py --file capture_ref_921600.txt      # rejeu hors ligne
  python uart_console.py --list

Commandes une fois lancé (Entrée pour valider) :
  IQON / IQOFF / AUTO / ORDER:0,1,2   envoyées au firmware
  :test [s]     collecte pendant s secondes (défaut 30) puis imprime un
                rapport par beacon et crée un DOSSIER horodaté
                test_AAAAMMJJ_HHMMSS/ contenant : le JSON (données+rapport,
                IQ par tone inclus -> réanalysable), le tracé PNG, config.txt
                (snapshot config firmware) et estimateur/ (figures IFFT +
                bayésien de la mesure typique et aberrante de chaque beacon,
                pour illustrer incertitude/multipath). Alias :mesure.
  :cal [b] [m]  afficher / régler les offsets de calibration (voir estimation.py)
  :show/:nshow  afficher / masquer la ligne par mesure
  :raw          afficher aussi les lignes UART brutes (toggle)
  :stats        bilan du décodage (mesures, rejets par raison)
  :reload       recharger estimation.py sans redémarrer (dev à chaud)
  :q  ou Ctrl-D quitter

Mode script (sans interaction) : --test N avec --port -> collecte N secondes,
crée le dossier horodaté (JSON+IQ, PNG, config.txt, figures estimateur/), puis
quitte. Pratique pour les séries de calibration.

Dépendance : pyserial (venv : voir install.txt). La fonction d'estimation vit
dans tools/estimation.py — voir le contrat dans ce fichier.
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
    sys.exit("pyserial manquant. Active le venv et installe (voir install.txt) :\n"
             "  cd tools && source .venv/bin/activate && pip install -r install.txt")


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("Aucun port série détecté (carte branchée ?).", file=sys.stderr)
    for p in ports:
        print(f"{p.device}\t{p.description}", file=sys.stderr)


def load_estimate():
    """Importe (ou réimporte) estimation.estimate. None si indisponible."""
    try:
        import estimation
        importlib.reload(estimation)
        fn = getattr(estimation, "estimate", None)
        if fn is None:
            print("[estimation.py trouvé mais sans fonction estimate(m)]",
                  file=sys.stderr, flush=True)
        return fn
    except Exception as e:
        print(f"[estimation.py non chargé : {type(e).__name__}: {e}]",
              file=sys.stderr, flush=True)
        return None


def write_config_snapshot(path):
    """Écrit dans `path` un .txt qui concatène la config firmware ACTUELLE
    (cs_config.h + prj.conf + overlays précision + reflector/prj.conf), pour
    que chaque test garde la trace exacte de la config avec laquelle il a été
    lancé. Best-effort : les fichiers absents sont notés, jamais fatals.
    Les chemins sont relatifs à l'emplacement de ce script (pas au cwd)."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)                       # tools/ -> racine du repo
    rel_files = [
        "initiator/src/cs_config.h",                   # profil + N (les 2 knobs)
        "initiator/prj.conf",                          # réservations SDC de base
        "initiator/overlay-precision-multi.conf",      # boat/indoor N>=2
        "initiator/overlay-precision-single.conf",     # boat/indoor N==1
        "reflector/prj.conf",                          # CS_EVENT_LEN côté réflecteur
    ]
    out = [f"# Snapshot config firmware — {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
           "# (l'overlay réellement compilé dépend du flag build.sh : --boat-n "
           "= multi, --boat-1 = single, drone = aucun)\n"]
    for rel in rel_files:
        out.append("\n" + "=" * 78 + f"\n# {rel}\n" + "=" * 78 + "\n")
        try:
            with open(os.path.join(repo, rel), encoding="utf-8", errors="replace") as f:
                out.append(f.read())
        except OSError as e:
            out.append(f"[introuvable : {e}]\n")
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(out)


class Display:
    """Pousse chaque ligne UART dans le décodeur ; affiche chaque mesure
    complète, augmentée de la distance rendue par estimate(m). Aucune
    exception de la fonction utilisateur ne doit tuer le thread de lecture :
    tout est encaissé, et l'erreur n'est affichée qu'à son premier
    changement (pas 17× par seconde)."""

    def __init__(self):
        self.stats = DropStats()
        self.pairer = Pairer(self.stats)
        self.estimate = load_estimate()
        self.show_raw = False
        self._last_err = None
        self.test_until = None      # échéance time.time() du test en cours, sinon None
        self.test_secs = 0
        self.test_data = {}         # {beacon: [enregistrements dict]}
        self.test_done = threading.Event()  # levé quand rapport+JSON+PNG écrits
        self.show_distances = False # ligne par mesure masquée par défaut (:show)
        self.note = ""              # label de config du test (--note / :note),
                                    # écrit dans le JSON -> étiquette les comparaisons

    def start_test(self, secs: int):
        self.test_data = {}
        self.test_secs = secs
        self.test_done.clear()
        self.test_until = time.time() + secs
        print(f"[test lancé : {secs} s de collecte, rapport à la fin]",
              file=sys.stderr, flush=True)

    def _build_report(self):
        """Statistiques par beacon sur les distances collectées."""
        rapport = {}
        for b, recs in sorted(self.test_data.items()):
            ds = sorted(r["d_m"] for r in recs)
            n = len(ds)
            med = statistics.median(ds)
            rssis = [r["rssi_loc"] for r in recs if r["rssi_loc"] != 127]
            # fréquence d'échantillonnage réelle, sur l'horloge de la carte
            span_ms = recs[-1]["t_ms"] - recs[0]["t_ms"]
            freq = round((n - 1) * 1000.0 / span_ms, 2) if n > 1 and span_ms > 0 else None
            rapport[f"b{b}"] = {
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
        return rapport

    def _finish_test(self):
        """Fin de fenêtre : imprime le rapport et sauve données+rapport en JSON."""
        rapport = self._build_report()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        day = stamp[:8]                             # AAAAMMJJ
        # Tous les tests sont rangés dans <repo>/tests/AAAAMMJJ/ quel que soit
        # le répertoire de lancement (chemin ancré sur l'emplacement du script).
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        outdir = os.path.join(repo, "tests", day, f"test_{stamp}")
        os.makedirs(outdir, exist_ok=True)
        fname = os.path.join(outdir, f"test_{stamp}.json")
        try:      # traçabilité : offsets de calibration actifs pendant ce test
            import estimation
            cal = {"global_m": getattr(estimation, "CAL_OFFSET_M", 0.0),
                   "par_beacon": getattr(estimation, "CAL_OFFSET_PAR_BEACON", {})}
        except ImportError:
            cal = None
        doc = {
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": self.note,                    # label de config (étiquette compare)
            "duree_s": self.test_secs,
            "calibration": cal,
            "rapport": rapport,
            "mesures": {f"b{b}": recs for b, recs in sorted(self.test_data.items())},
        }
        with open(fname, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)

        print(f"\n=== RAPPORT ({self.test_secs} s) ===", flush=True)
        if not rapport:
            print("aucune mesure collectée !", flush=True)
        for name, r in rapport.items():
            print(f"{name}: n={r['n']:3d} @ {r['freq_hz']} Hz  "
                  f"médiane={r['mediane_m']:6.2f} m  "
                  f"IQR=[{r['iqr_m'][0]:.2f}..{r['iqr_m'][1]:.2f}]  "
                  f"σ={r['sigma_m']:.2f}  min/max={r['min_m']:.2f}/{r['max_m']:.2f}  "
                  f"outliers>1m={r['outliers_sup_1m']}  "
                  f"rssi={r['rssi_moyen_dbm']} dBm", flush=True)
        cfg = os.path.join(outdir, "config.txt")
        try:                                        # config exacte du test
            write_config_snapshot(cfg)
        except OSError as e:
            print(f"[snapshot config en échec : {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
            cfg = None
        png = self._plot(fname.replace(".json", ".png"))
        est_figs = self._plot_estimator(outdir)     # IFFT/bayésien par beacon
        cons_figs = self._plot_consistency(outdir)  # rapport consistance IQ par beacon
        contenu = ["test_%s.json" % stamp]
        if png:
            contenu.append("test_%s.png" % stamp)
        if cfg:
            contenu.append("config.txt")
        if est_figs:
            contenu.append(f"estimateur/ ({len(est_figs)} fig)")
        if cons_figs:
            contenu.append(f"consistency_b* ({len(cons_figs)})")
        short = os.path.join("tests", day, f"test_{stamp}")   # affichage court
        print(f"[dossier -> {short}/ : " + ", ".join(contenu) + "]\n", flush=True)

    def _plot_estimator(self, outdir):
        """Figures IFFT + vraisemblance bayésienne (incertitude / multipath)
        pour la mesure TYPIQUE et la plus ABERRANTE de chaque beacon. Réutilise
        viz_estimateur (mêmes fonctions que l'estimateur). Best-effort : toute
        erreur est encaissée, le rapport reste écrit. Renvoie la liste des PNG."""
        try:
            import numpy as np
            import viz_estimateur as vz
        except Exception as e:
            print(f"[figures estimateur indisponibles : {type(e).__name__}: {e}]",
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
            picks = [("typique", typ)] + ([("aberrante", wor)] if wor is not typ else [])
            for tag, r in picks:
                try:
                    os.makedirs(subdir, exist_ok=True)
                    png = os.path.join(subdir, f"b{b}_{tag}_rc{r['counter']}.png")
                    if vz.plot_measurement(vz.meas_from_record(b, r), png) is not None:
                        made.append(png)
                except Exception as e:
                    print(f"[fig estimateur b{b} {tag} en échec : "
                          f"{type(e).__name__}: {e}]", file=sys.stderr, flush=True)
        return made

    def _plot_consistency(self, outdir):
        """Rapport de consistance IQ (cohérence par canal, top-X%, R-vs-amplitude)
        par beacon, écrit dans le dossier du test. Réutilise iq_consistency.
        Best-effort. Renvoie la liste des PNG (consistency_bX.png)."""
        try:
            import iq_consistency as ic
        except Exception as e:
            print(f"[rapport consistance indisponible : {type(e).__name__}: {e}]",
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
                print(f"[rapport consistance b{b} en échec : "
                      f"{type(e).__name__}: {e}]", file=sys.stderr, flush=True)
        return made

    def _plot(self, fname):
        """PNG : distance BRUTE (points, avant médiane glissante) et lissée
        (trait) de chaque beacon au cours du temps. None si matplotlib absent."""
        try:
            import matplotlib
            matplotlib.use("Agg")             # sans écran : fichier seulement
            import matplotlib.pyplot as plt
        except ImportError:
            print("[matplotlib absent : pas de tracé (pip install matplotlib)]",
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
                ax.plot(*zip(*pts), ".", ms=4, label="brute")
            ax.plot(t, [r["d_m"] for r in recs], "-", lw=1.2, alpha=0.75,
                    label="lissée (médiane 7)")
            ax.set_ylabel(f"b{b}  (m)")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8)
        axes[-1, 0].set_xlabel("temps (s)")
        fig.suptitle(f"Distance par beacon — test {self.test_secs} s")
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
        d_brut = None
        if self.estimate is not None:
            try:
                d = self.estimate(m)
                self._last_err = None
                try:      # valeur AVANT médiane glissante, exposée par estimation.py
                    import estimation
                    d_brut = estimation.DERNIERE_BRUTE
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
                "t_ms": m.t_ms,                       # k_uptime 1er subevent (ms)
                "t_done_ms": getattr(m, "t_done_ms", None),   # fin de procédure
                "n_subevents": getattr(m, "n_subevents", None),
                # -> durée de procédure = t_done_ms - t_ms  (le `meas` des logs)
                "d_m": round(float(d), 4),
                "d_brut_m": round(d_brut, 4) if d_brut is not None else None,
                "rssi_loc": m.rssi_loc,
                "rssi_ref": m.rssi_ref,
                "tones": len(m.tones),
                # IQ par tone -> le JSON devient réanalysable (figures IFFT/
                # bayésien hors ligne, viz_estimateur.py --json). Compact :
                # [canal, i_loc, q_loc, i_ref, q_ref, tq_loc, tq_ref].
                "iq": [[t.channel, t.i_loc, t.q_loc, t.i_ref, t.q_ref,
                        t.tq_loc, t.tq_ref] for t in m.tones],
            })
        self.poll_test()

    def poll_test(self):
        """Fait tomber le rapport à l'échéance, MÊME si le flux est mort
        (appelé aussi par le thread lecteur à chaque tick de 0,2 s). Protégé :
        une exception du rapport/tracé ne doit pas tuer le thread lecteur."""
        if self.test_until is None or time.time() < self.test_until:
            return
        self.test_until = None
        try:
            self._finish_test()
        except Exception as e:
            print(f"[rapport en échec : {type(e).__name__}: {e}]",
                  file=sys.stderr, flush=True)
        finally:
            # signale au mode --test que rapport+JSON+PNG sont écrits : il peut
            # fermer le processus sans interrompre le tracé (course résolue).
            self.test_done.set()


def reader_loop(ser, stop, disp: Display):
    """Thread : lit l'UART, reconstitue les lignes, alimente le décodeur."""
    buf = b""
    first = True
    while not stop.is_set():
        try:
            chunk = ser.read(4096)
        except serial.SerialException:
            print("\n[port perdu — lecture arrêtée]", file=sys.stderr, flush=True)
            stop.set()
            return
        if not chunk:
            disp.poll_test()          # échéance de :test surveillée même sans flux
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            if first:                     # première ligne partielle : ignorer
                first = False
                continue
            line = raw.decode("ascii", "replace").strip()
            if line:
                disp.on_line(line)


def replay_file(path: str, disp: Display):
    """Mode hors ligne : rejoue une capture brute à travers le même pipeline."""
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line:
                disp.on_line(line)
    print(disp.stats.summary(), file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser(
        description="Console UART CS : commandes + mesures décodées + estimation")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="port série (ex. /dev/ttyACM0, COM5)")
    src.add_argument("--file", help="rejouer une capture brute (hors ligne)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="débit (défaut 921600 = current-speed de l'overlay)")
    ap.add_argument("--list", action="store_true", help="lister les ports et quitter")
    ap.add_argument("--test", type=int, metavar="N",
                    help="mode script : collecte N s -> dossier horodaté "
                         "(JSON+IQ, PNG, config.txt, figures estimateur/), puis quitte")
    ap.add_argument("--note", default="", metavar="LABEL",
                    help="label de config du test (ex. 'boat_thin1_5m_N1') "
                         "écrit dans le JSON -> étiquette les courbes de comparaison")
    args = ap.parse_args()


    if args.list:
        list_serial_ports()
        return
    if args.file:
        replay_file(args.file, Display())
        return
    if not args.port:
        ap.error("--port ou --file requis (ou --list pour voir les ports)")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Ouverture de {args.port} impossible : {e}")

    disp = Display()
    disp.note = args.note                         # label de config (--note)
    print(f"[connecté à {args.port} @ {args.baud} bauds]  une ligne par mesure  |  "
          f"IQON/IQOFF/AUTO/ORDER pour le firmware  |  :raw :stats :reload :q",
          file=sys.stderr, flush=True)

    stop = threading.Event()
    rx = threading.Thread(target=reader_loop, args=(ser, stop, disp), daemon=True)
    rx.start()

    # Mode script --test N : pas de boucle interactive — collecte, rapport, exit.
    # On attend test_done (levé APRÈS l'écriture du rapport/JSON/PNG par le
    # thread lecteur) : sinon on fermerait le processus pendant le tracé, tuant
    # le thread daemon et perdant le PNG. +10 s de garde si le flux s'arrête.
    if args.test:
        disp.start_test(args.test)
        try:
            if not disp.test_done.wait(timeout=args.test + 10):
                print("[flux interrompu avant la fin du test]", file=sys.stderr)
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
            if cmd.split()[0] in (":mesure", ":test"):
                parts = cmd.split()
                try:
                    secs = int(parts[1]) if len(parts) > 1 else 30
                except ValueError:
                    print("[usage : :test <secondes>]", file=sys.stderr, flush=True)
                    continue
                disp.start_test(secs)
                continue
            if cmd.split()[0] == ":cal":
                # :cal            -> affiche les offsets courants
                # :cal 0.35       -> offset global (m, soustrait à la sortie)
                # :cal 2 -0.10    -> offset additionnel du beacon 2
                try:
                    import estimation
                    parts = cmd.split()
                    if len(parts) == 2:
                        estimation.CAL_OFFSET_M = float(parts[1])
                    elif len(parts) == 3:
                        estimation.CAL_OFFSET_PAR_BEACON[int(parts[1])] = float(parts[2])
                    print(f"[cal : global={estimation.CAL_OFFSET_M:+.3f} m, "
                          f"par beacon={estimation.CAL_OFFSET_PAR_BEACON} "
                          f"(⚠ :reload les remet aux valeurs du fichier)]",
                          file=sys.stderr, flush=True)
                except (ValueError, ImportError):
                    print("[usage : :cal | :cal <offset_m> | :cal <beacon> <offset_m>]",
                          file=sys.stderr, flush=True)
                continue
            if cmd.split()[0] == ":note":
                disp.note = cmd[len(":note"):].strip()
                print(f"[note de test = '{disp.note}']", file=sys.stderr, flush=True)
                continue
            if cmd == ":show":
                disp.show_distances = True
                continue
            if cmd == ":nshow":
                disp.show_distances = False
                continue
            if cmd == ":raw":
                disp.show_raw = not disp.show_raw
                print(f"[affichage brut {'ON' if disp.show_raw else 'OFF'}]",
                      file=sys.stderr, flush=True)
                continue
            if cmd == ":stats":
                print(disp.stats.summary(), file=sys.stderr, flush=True)
                continue
            if cmd == ":reload":
                disp.estimate = load_estimate()
                print(f"[estimation {'rechargée' if disp.estimate else 'absente'}]",
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
        print("\n[fermé]", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
