#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""viz3d.py — visualiseur 3D de l'initiateur CS (pygame).

Les balises (réflecteurs) sont FIXES à des coordonnées connues ; l'initiateur
(sur le drone) est mobile. On mesure les distances initiateur↔balises en
direct, et on TRILATÈRE la position 3D de l'initiateur (moindres carrés sur
les N distances). La scène montre : les balises, une sphère par balise dont le
rayon est la distance mesurée (leur intersection ≈ l'initiateur), le point
trilatéré et sa trace.

  ┌─────────────────────────────────────────────────────────────────────┐
  │ RENSEIGNER LES COORDONNÉES DES BALISES ci-dessous (mètres, repère de │
  │ ton choix ; z = hauteur). La clé est l'INDEX beacon (b0/b1/b2…).     │
  └─────────────────────────────────────────────────────────────────────┘

Usage :
  python viz3d.py --port /dev/ttyACM0        # live
  python viz3d.py --demo                     # sans matériel : initiateur simulé
  python viz3d.py --file capture_ref_921600.txt   # rejeu d'une capture

Contrôles : glisser souris = tourner | molette = zoom | F = suivre l'initiateur
            G = grille on/off | S = sphères on/off | T = trace on/off | Échap = quitter

Dépendances : pygame, numpy, scipy (voir install.txt).
"""

import argparse
import math
import sys
import threading
import time
from collections import defaultdict, deque

import numpy as np

# ═══════════════════════════════════════════════════════════════════════════
#  COORDONNÉES DES BALISES — À RENSEIGNER (mètres). Index beacon -> (x, y, z).
#  Exemple : trois balises au sol formant un triangle de 6 m de côté.
# ═══════════════════════════════════════════════════════════════════════════
BEACONS = {
    0: (0.00, 0.00, 0.0),
    1: (3.35, -4.61, 0.0),
    2: (17.00, 0.00, 1.20),   # à 5.7 m de b0 ET de b1 (médiatrice), au sol
}

# Couleurs par balise (RGB), réutilisées pour sa sphère de distance.
BEACON_COLORS = {
    0: (240, 90, 90),
    1: (90, 200, 120),
    2: (90, 160, 240),
    3: (230, 200, 90),
    4: (200, 120, 230),
}

FLOOR_HALF = 8.0     # demi-côté de la grille au sol (m)
FLOOR_STEP = 1.0     # pas de la grille (m)


# ─────────────────────────────────────────────────────────────────────────────
#  Trilatération : position minimisant Σ (‖x - pᵢ‖ - dᵢ)²  (moindres carrés)
# ─────────────────────────────────────────────────────────────────────────────
def trilaterate(anchors, dists, guess):
    """anchors: (K,3), dists: (K,), guess: (3,). Retourne (3,) ou None."""
    from scipy.optimize import least_squares
    if len(anchors) < 3:
        return None
    A = np.asarray(anchors, float)
    d = np.asarray(dists, float)

    def resid(x):
        return np.linalg.norm(A - x, axis=1) - d

    try:
        sol = least_squares(resid, guess, method="lm", max_nfev=50)
        return sol.x if sol.success else None
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────────────────────
#  Caméra 3D : rotation azimut/élévation autour d'un point cible + projection
# ─────────────────────────────────────────────────────────────────────────────
class Camera:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.az = math.radians(35)      # azimut
        self.el = math.radians(25)      # élévation
        self.dist = 18.0                # distance caméra-cible
        self.target = np.array([3.0, 2.5, 0.5])
        self.fov = 650.0                # focale (px)

    def _basis(self):
        ca, sa = math.cos(self.az), math.sin(self.az)
        ce, se = math.cos(self.el), math.sin(self.el)
        # direction caméra -> cible
        fwd = np.array([ce * ca, ce * sa, se])
        right = np.array([-sa, ca, 0.0])
        up = np.cross(right, fwd)
        eye = self.target - fwd * self.dist
        return eye, right, up, fwd

    def project(self, p):
        """Point monde (3,) -> (sx, sy, depth) ; depth<=0 = derrière la caméra."""
        eye, right, up, fwd = self._basis()
        rel = np.asarray(p, float) - eye
        depth = rel @ fwd
        if depth <= 0.05:
            return None
        x = (rel @ right) / depth * self.fov + self.w / 2
        y = -(rel @ up) / depth * self.fov + self.h / 2
        return (x, y, depth)


# ─────────────────────────────────────────────────────────────────────────────
#  Source de distances : thread de fond alimentant un état partagé
# ─────────────────────────────────────────────────────────────────────────────
class DistanceSource:
    """Maintient {beacon: distance lissée} depuis le live, un fichier, ou une
    simulation. La distance de chaque balise est la MÉDIANE de ses `nsamples`
    dernières mesures (20 par défaut) — anti-saut avant la trilatération. Une
    balise dont le flux s'est arrêté depuis plus de `stale_s` secondes est
    considérée absente (plus de distance : sa vieille sphère ne fige plus)."""

    def __init__(self, nsamples=20, stale_s=3.0):
        self.nsamples = nsamples
        self.stale_s = stale_s
        self.lock = threading.Lock()
        self._hist = defaultdict(lambda: deque(maxlen=nsamples))  # (t, d)
        self.dist = {}
        self.stop = threading.Event()

    def _median(self, h, now):
        # h ne garde déjà que les nsamples derniers points (maxlen) ; on ne
        # renvoie une distance que si la plus récente n'est pas périmée.
        if not h or now - h[-1][0] > self.stale_s:
            return None
        return float(np.median([d for _, d in h]))

    def _update(self, beacon, d):
        now = time.monotonic()
        with self.lock:
            h = self._hist[beacon]
            h.append((now, d))
            self.dist[beacon] = self._median(h, now)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            out = {}
            for b, h in self._hist.items():
                m = self._median(h, now)
                if m is not None:
                    out[b] = m
                self.dist[b] = m
            return out

    # -- live / fichier : réutilise la chaîne cs_decoder + estimation ----------
    def run_stream(self, port=None, baud=921600, path=None):
        import estimation
        if path:
            from cs_decoder import measurements_from_lines
            src = measurements_from_lines(open(path, encoding="utf-8",
                                               errors="replace"))
        else:
            from cs_decoder import measurements_from_serial
            src = measurements_from_serial(port, baud)
        for m in src:
            if self.stop.is_set():
                break
            try:
                d = estimation.estimate(m)
            except Exception:
                d = None
            if isinstance(d, (int, float)):
                self._update(m.beacon, float(d))
            if path:
                time.sleep(0.05)      # rejeu à cadence ~réaliste

    # -- démo : initiateur en cercle, distances géométriques + bruit ----------
    def run_demo(self):
        anchors = {b: np.array(p, float) for b, p in BEACONS.items()}
        t = 0.0
        while not self.stop.is_set():
            pos = np.array([3 + 2.5 * math.cos(t), 2.5 + 2.5 * math.sin(t),
                            1.2 + 0.3 * math.sin(2 * t)])
            for b, a in anchors.items():
                d = np.linalg.norm(pos - a) + np.random.normal(0, 0.05)
                self._update(b, d)
            t += 0.03
            time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
#  Rendu
# ─────────────────────────────────────────────────────────────────────────────
def great_circles(center, radius, n=40):
    """3 grands cercles orthogonaux (wireframe de sphère) -> listes de points."""
    ang = np.linspace(0, 2 * math.pi, n)
    cs, sn = np.cos(ang), np.sin(ang)
    z0 = np.zeros(n)
    loops = [
        np.stack([cs, sn, z0], 1),      # plan XY
        np.stack([cs, z0, sn], 1),      # plan XZ
        np.stack([z0, cs, sn], 1),      # plan YZ
    ]
    return [center + radius * loop for loop in loops]


def run(source, follow_default=False):
    import pygame
    pygame.init()
    W, H = 1100, 760
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Initiateur CS — visualiseur 3D")
    font = pygame.font.SysFont("dejavusansmono", 15)
    big = pygame.font.SysFont("dejavusansmono", 18, bold=True)
    clock = pygame.time.Clock()
    cam = Camera(W, H)

    show = {"grid": True, "spheres": True, "trail": True, "points": True}
    follow = follow_default
    trail = deque(maxlen=240)
    last_pos = np.array([3.0, 2.5, 1.0])
    dragging = False
    lastm = (0, 0)

    def to2d(p):
        return cam.project(p)

    def draw_line3d(a, b, color, width=1):
        pa, pb = to2d(a), to2d(b)
        if pa and pb:
            pygame.draw.line(screen, color, pa[:2], pb[:2], width)

    def draw_polyline3d(pts, color, width=1):
        proj = [to2d(p) for p in pts]
        seg = [(p[0], p[1]) for p in proj if p]
        if len(seg) >= 2:
            pygame.draw.lines(screen, color, False, seg, width)

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_f:
                    follow = not follow
                elif e.key == pygame.K_g:
                    show["grid"] = not show["grid"]
                elif e.key == pygame.K_s:
                    show["spheres"] = not show["spheres"]
                elif e.key == pygame.K_t:
                    show["trail"] = not show["trail"]
                elif e.key == pygame.K_p:
                    show["points"] = not show["points"]
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1:
                    dragging = True
                    lastm = e.pos
                elif e.button == 4:
                    cam.dist = max(3.0, cam.dist - 1.2)
                elif e.button == 5:
                    cam.dist = min(60.0, cam.dist + 1.2)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                dragging = False
            elif e.type == pygame.MOUSEMOTION and dragging:
                dx, dy = e.pos[0] - lastm[0], e.pos[1] - lastm[1]
                lastm = e.pos
                cam.az += dx * 0.008
                cam.el = max(-1.5, min(1.5, cam.el + dy * 0.008))

        dist = source.snapshot()

        # trilatération sur les balises qui ont une distance
        anchors, ds = [], []
        for b, p in BEACONS.items():
            if b in dist:
                anchors.append(p)
                ds.append(dist[b])
        pos = None
        if len(anchors) >= 3:
            pos = trilaterate(anchors, ds, last_pos)
            if pos is not None:
                last_pos = pos
                if show["trail"]:
                    trail.append(pos.copy())
        if follow and pos is not None:
            cam.target = 0.85 * cam.target + 0.15 * pos

        # ── dessin ──
        screen.fill((18, 20, 26))

        if show["grid"]:
            g = (55, 60, 70)
            n = int(FLOOR_HALF / FLOOR_STEP)
            for i in range(-n, n + 1):
                x = i * FLOOR_STEP
                draw_line3d([x, -FLOOR_HALF, 0], [x, FLOOR_HALF, 0], g)
                draw_line3d([-FLOOR_HALF, x, 0], [FLOOR_HALF, x, 0], g)
            # axes
            draw_line3d([0, 0, 0], [1.5, 0, 0], (200, 80, 80), 2)
            draw_line3d([0, 0, 0], [0, 1.5, 0], (80, 200, 80), 2)
            draw_line3d([0, 0, 0], [0, 0, 1.5], (80, 120, 220), 2)

        # sphères de distance + balises
        for b, p in BEACONS.items():
            col = BEACON_COLORS.get(b, (200, 200, 200))
            p = np.array(p, float)
            if show["spheres"] and b in dist:
                dim = tuple(int(c * 0.45) for c in col)
                for loop in great_circles(p, dist[b]):
                    draw_polyline3d(loop, dim)
            pr = to2d(p)
            if pr:
                pygame.draw.circle(screen, col, (int(pr[0]), int(pr[1])), 7)
                pygame.draw.circle(screen, (255, 255, 255),
                                   (int(pr[0]), int(pr[1])), 7, 1)
                label = f"b{b}"
                if b in dist:
                    label += f"  {dist[b]:.2f} m"
                screen.blit(font.render(label, True, col),
                            (int(pr[0]) + 10, int(pr[1]) - 8))
            # ligne balise -> initiateur + POINT à la distance mesurée
            if pos is not None:
                draw_line3d(p, pos, tuple(int(c * 0.5) for c in col))
                if show["points"] and b in dist:
                    vec = pos - p
                    L = float(np.linalg.norm(vec))
                    if L > 1e-6:
                        ptd = p + dist[b] * vec / L    # point à d le long de p->init
                        # résidu = écart entre la sphère et l'initiateur trilatéré
                        draw_line3d(ptd, pos, (150, 150, 160))
                        pd = to2d(ptd)
                        if pd:
                            x, y = int(pd[0]), int(pd[1])
                            pygame.draw.circle(screen, col, (x, y), 5)
                            pygame.draw.circle(screen, (20, 20, 20), (x, y), 5, 1)

        # trace + initiateur
        if show["trail"] and len(trail) >= 2:
            draw_polyline3d(list(trail), (120, 130, 150))
        if pos is not None:
            ip = to2d(pos)
            if ip:
                pygame.draw.circle(screen, (255, 230, 120),
                                   (int(ip[0]), int(ip[1])), 9)
                pygame.draw.circle(screen, (30, 30, 30),
                                   (int(ip[0]), int(ip[1])), 9, 2)
                screen.blit(big.render("initiateur", True, (255, 230, 120)),
                            (int(ip[0]) + 12, int(ip[1]) + 6))

        # HUD
        hud = [
            "glisser=tourner molette=zoom F=suivre G=grille S=spheres P=points T=trace Echap=quitter",
            f"balises avec distance : {len(anchors)}/{len(BEACONS)}"
            + ("   position : (%.2f, %.2f, %.2f) m" % tuple(pos) if pos is not None
               else "   trilateration : >=3 balises requises"),
        ]
        for i, line in enumerate(hud):
            screen.blit(font.render(line, True, (180, 190, 205)), (12, 10 + 20 * i))
        # par balise : coordonnées, distance mesurée, et résidu vs point trilatéré
        for i, (b, p) in enumerate(BEACONS.items()):
            col = BEACON_COLORS.get(b, (200,) * 3)
            txt = f"b{b} @ ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})"
            if b in dist:
                txt += f"   d={dist[b]:.2f} m"
                if pos is not None:
                    res = dist[b] - float(np.linalg.norm(np.array(p, float) - pos))
                    txt += f"   res={res:+.2f} m"
            screen.blit(font.render(txt, True, col),
                        (12, H - 20 * (len(BEACONS) - i) - 8))

        pygame.display.flip()
        clock.tick(60)

    source.stop.set()
    pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="Visualiseur 3D de l'initiateur CS")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="port série (live)")
    src.add_argument("--file", help="rejouer une capture brute")
    src.add_argument("--demo", action="store_true", help="initiateur simulé (sans matériel)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--nsamples", type=int, default=20,
                    help="nb de derniers points pour la médiane des distances (défaut 20)")
    ap.add_argument("--follow", action="store_true", help="caméra suit l'initiateur")
    args = ap.parse_args()

    source = DistanceSource(nsamples=args.nsamples)
    if args.demo:
        target = source.run_demo
    elif args.file:
        target = lambda: source.run_stream(path=args.file)
    elif args.port:
        target = lambda: source.run_stream(port=args.port, baud=args.baud)
    else:
        ap.error("--port, --file ou --demo requis")

    th = threading.Thread(target=target, daemon=True)
    th.start()
    run(source, follow_default=args.follow)


if __name__ == "__main__":
    main()
