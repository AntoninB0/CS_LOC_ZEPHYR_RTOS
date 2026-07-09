# Optimisation du temps de mesure CS — attaque du startup ~700 ms

## Diagnostic (pourquoi le mur ne bougeait pas)

Le startup mesuré (~10-13 événements de connexion, ∝ intervalle) est la somme de :

1. **La procédure LL de démarrage CS** (Core Spec v6.x, Vol 6 Part B, §5.1.26
   *Channel Sounding Start procedure*) : LL_CS_REQ → LL_CS_RSP → LL_CS_IND.
   Point clé de la spec : **l'ancre du 1er subevent est l'événement de connexion
   qui porte le LL_CS_IND** — la spec n'impose PAS d'« instant » lointain comme
   un connection update. Les 3-4 premiers événements sont incompressibles
   (aller-retours LL), le reste est du choix du contrôleur.
2. **Le placement choisi par le SDC**, contraint par ses RÉSERVATIONS de
   timeline. Et c'est là que ça coinçait, par 3 Kconfig empilés :

   | Kconfig (SDC) | Défaut effectif | Rôle |
   |---|---|---|
   | `CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT` | **90 000 µs dès que `BT_CTLR_CHANNEL_SOUNDING=y`** | espacement imposé entre ancres ACL des liens centraux |
   | `CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT` | 7 500 µs | réservation ACL par lien et par intervalle |
   | `CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT` | 5 000 µs (était à **16 000** dans prj.conf) | réservation par événement CS |

   Le quadrillage caché à 90 ms était le vrai coupable : avec 2 liens à
   60/77,5 ms d'intervalle, le SDC ne pouvait pas honorer son espacement →
   placements dégradés, 1er subevent différé (~700 ms), plancher d'intervalle
   à ~50 ms (0x0c), et collisions 0x3 figées (que la désync +14×idx contournait).

## Changements appliqués

### A. Kconfig (initiator/prj.conf)
- `CS_EVENT_LEN_DEFAULT` 16000 → **5000** (subevent réel = 4500 µs) — aussi côté réflecteur
- `CENTRAL_ACL_EVENT_SPACING_DEFAULT` (90000 implicite) → **15000**
  (= ACL 3750 + CS 5000 + marge)
- `MAX_CONN_EVENT_LEN_DEFAULT` 7500 → **3750** (l'extension d'événement,
  active par défaut, préserve le débit du fetch RAS)

### B. Intervalle de connexion (cs_config.c)
- `cs_cp_active` : 48 → **24** (30 ms). 2 liens × 15 ms d'espacement = 30 ms,
  le quadrillage tombe juste.
- Désync : `CS_ACL_DESYNC_STEP` (cs_config.h) = **0** — à intervalles
  identiques, le SDC espace lui-même les ancres de 15 ms, déterministe et sans
  collision. Fallback si des 0x3 persistent : remettre 6.

### C. Pipeline continu (main.c)
Ré-enable **immédiat** de chaque beacon après sa collecte : le startup de la
procédure N+1 (pure attente de scheduling, radio libre) recouvre le fetch RAS
des autres beacons. Régime établi : cycle → max(startup, Σ fetch) au lieu de
startup + Σ fetch.

### D. Instrumentation (cs_ranging.c)
Par mesure : `b%d timing: startup=... meas=... fetch=...` + `cycle: ... ms`.

## Prédictions à vérifier (dans l'ordre)

| Étape | Attendu |
|---|---|
| A+B seuls (garder l'ancienne boucle si besoin d'isoler) | startup 700 → **~300-350 ms** (≈11 évts × 30 ms), fetch 340 → **~170-230 ms** |
| A+B+C | cycle 1500 → **~400-600 ms** (2 beacons) |

## Plan de mesure / replis
1. Flasher les 2 réflecteurs AUSSI (CS_EVENT_LEN a changé des deux côtés).
2. Si `Procedure Enable` → 0x0c : intervalle 24 → 28 → 32 (garder spacing 15000).
3. Si aborts 0x3 récurrents et toujours sur le même beacon :
   `CS_ACL_DESYNC_STEP` 0 → 6 (précession).
4. Si le fetch RAS ne descend pas ∝ intervalle : vérifier que l'extension
   d'événement est bien active (`CONFIG_BT_CTLR_SDC_CONN_EVENT_EXTEND_DEFAULT`
   ne doit PAS être à n).
5. Étape suivante (non codée, RAM 88 % à surveiller) : **RAS temps réel** —
   le RRSP de NCS 3.3.0 l'annonce (`RAS_FEAT_REALTIME_RD`) et le client expose
   `bt_ras_rreq_realtime_rd_subscribe()` : les données pair arrivent en
   notifications dès la fin de procédure, plus de rd_ready ni d'aller-retour
   RAS-CP → fetch réduit à ~2-3 intervalles. Coût : un buffer peer PAR beacon
   (le buffer partagé ne marche plus, les notifications sont asynchrones).

---

# Itération 2 — cacher les ~340 ms restants

## Résultats mesurés de l'itération 1 (intervalle 30 ms, spacing 15000)
startup ≈ 314-342 ms (≈ 11,3 événements — conforme), meas 0 ms,
fetch 132-180 ms, cycle ≈ 540 ms, distances stables, 0 abort visible.

## Pourquoi le startup ne descend pas sous ~11 événements
Aller-retours LL_CS_REQ → LL_CS_RSP → LL_CS_IND (Channel Sounding Start
procedure, Vol 6 Part B §5.1.26) + marge interne du SDC pour le choix de
l'ancre. Le NOMBRE d'événements est interne au contrôleur (aucun Kconfig) ;
seule leur DURÉE (l'intervalle) et le RECOUVREMENT sont à notre main.

## Changements itération 2
1. **Intervalle 24 → 16 (20 ms)**, spacing 15000 → 10000
   (ACL 3750 + CS 5000 + marge 1250 par lien). Startup attendu ~220-230 ms.
   Replis 0x0c/0x3 : intervalle 20 + spacing 12500, puis retour 24/15000.
2. **Ré-enable AVANT le fetch** (main.c) : wait_done → snapshot des steps
   locaux → enable N+1 → fetch RAS de N. Le startup de N+1 recouvre le fetch
   de N ET ceux des autres beacons. Régime établi 2 beacons :
   cycle ≈ max(startup, 2×fetch) + résidu → prédiction **~230-280 ms**
   (~4 Hz par beacon), au lieu de startup + fetch.
3. **Buffer local 256 → 64 steps** (CS_MAX_STEPS_PER_PROC, cs_ranging.c) :
   les procédures produisent ~30 steps (~300-400 o observés). Même avec le
   buffer snapshot ajouté, bilan RAM ≈ **−11 Ko** vs avant.
   ⚠ Si le channel thinning est réduit (plus de canaux), redimensionner.

## Détails de robustesse
- `sem_rd_ready` n'est PLUS reset dans cs_enable_beacon (le rd_ready de N
  arrive autour de l'enable de N+1).
- Le fetch se fait par compteur EXPLICITE (snap_rc) : un rd_ready plus récent
  ne fait qu'un warning "counter skew", la rétention réflecteur
  (RD_BUFFERS_PER_CONN=10) garde la procédure N.
- `fetch_failed` séparé de `aborted` : l'abort de la procédure en vol N+1 ne
  peut plus invalider le fetch de N.

---

# Itération 2 — verdict banc et v2.1

- **16/10000 : ÉCHEC.** Abort `subevent 0x3` déterministe sur les 2 beacons,
  aucune procédure ne passe. Marge de 1250 µs par créneau insuffisante.
  Les "Peer ranging data overwritten" en rafale ne sont qu'un symptôme
  (le réflecteur produit, l'initiateur avorté ne fetch jamais).
- **v2.1 = code de recouvrement (v2) + timing validé (24/15000).**
  Attendu : cycle ≈ max(startup 340, 2×fetch ≈ 300) + résidu ≈ **340-400 ms**
  pour les 2 beacons, sans abort. L'échec du 20 ms ne disait rien du code.
- Étape optionnelle ensuite : 20/12500 (25 ms, marge 3750 µs — bien plus
  saine que les 1250 du 10000).

---

# Itération 3 — RAS temps réel + exploitation RTT/qualité/dérive

## Firmware : CS_RAS_REALTIME (cs_config.h, défaut 1)
Le squelette `#if CS_RAS_REALTIME` présent dans le code d'origine (jamais
activé) a été complété : init des buffers peer_rt/peer_snap, branche temps
réel de cs_fetch_beacon (la donnée pair est POUSSÉE par le réflecteur à la
fin de procédure → le fetch devient une vérification de compteur, ~0 ms),
symboles on-demand sous garde. Skew de compteur en RT = mesure perdue
(pas de rétention) — cas rare, signalé par LOG_WRN. Prédiction : cycle
2 beacons ≈ startup ≈ 350-400 ms (~3 Hz/beacon). Fallback : CS_RAS_REALTIME=0
(on-demand validé). Seul l'initiateur est à reflasher.

## Python (iq_estimation.py) : trois gisements exploités
1. **RTT mode-1** : layout pair décodé (quality, NADM, RSSI, ToA_ToD i16 LE
   ×0,5 ns, antenna) + ToD_ToA local. d_rtt = médiane((ToD+ToA)/2)×0,5ns×c.
   Grossier mais NON-AMBIGU → arbitre du repli d'alias : validé synthétique
   62 m → 61,0 (au lieu de 11 replié), 104,5 → 105,5.
2. **Pondération γ par tone** (qualité × amplitude, le MATLAB fixait γ=1) :
   robuste à 30 % de tones fadés (bayésien juste au cm).
3. **freq_offset mode-0** (0,01 ppm) : exposé par mesure et en médiane batch
   (modèle d'horloge PARNAV). Ex réel : 4,75 ppm.
⚠ À CALIBRER sur banc : signe/offset du RTT réel (trame 319 : médiane
-35×0,5 ns → offset de groupe à soustraire, constante par carte).
