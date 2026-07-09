# Configurations par cas d'usage × nombre de beacons (1-5)

Deux réglages dans `cs_config.h`, tout le reste se dérive automatiquement :
la **famille** (`CONF_DRONE_FAST_OUTDOOR` / `CONF_BOAT_DOCKING` /
`CONF_DRONE_INDOOR`) et **`CS_NUM_BEACONS`** (1-5). Les garde-fous
BUILD_ASSERT refusent toute combinaison incohérente avec le prj.conf.
Paramètres alignés sur `doc_optimised_parameters.ods` (renvois Vol 6 Part H).

## Build

| Famille | Overlay initiateur | CS_EVENT_LEN réflecteurs |
|---|---|---|
| DRONE_FAST_OUTDOOR | aucun (prj.conf de base : 5000/15000) | 5000 |
| BOAT_DOCKING / DRONE_INDOOR, N≥2 | `-DEXTRA_CONF_FILE=overlay-precision-multi.conf` | 8500 |
| BOAT_DOCKING / DRONE_INDOOR, N=1 | `-DEXTRA_CONF_FILE=overlay-precision-single.conf` | 8500 |

Changer de famille ⇒ reflasher aussi les réflecteurs. Changer seulement N ⇒ initiateur seul.

## Ce qui s'adapte avec N (automatique)

Intervalle de connexion = N × créneau (un créneau ACL+CS par lien, quadrillé
par `CENTRAL_ACL_EVENT_SPACING`) ; startup ≈ 11 × intervalle — la cadence
décroît donc linéairement avec N, c'est structurel (le nombre d'événements du
handshake LL est incompressible). En RAS temps réel le fetch ≈ 0, tous les
beacons ont une procédure en vol en permanence : chaque beacon livre une
distance par « startup ». S'adaptent aussi : les buffers par beacon
(`CS_MAX_STEPS_PER_PROC` réduit à N≥4), le thinning des familles précision
(2 imposé à N≥4, RAM ÷2 — non-ambigu 75 m au lieu de 150), la
`channel_map_repetition` docking (2 seulement à N≤2), et le scheduler
(round ≤ 16 slots, bonus aux beacons de moindre erreur).

## Matrice prédictive (RAS temps réel, pipeline v3.6+)

**DRONE_FAST_OUTDOOR** — thinning 3, RTT sounding 32b, 3C/X, 2M_2BT, repeat 1

| N | Intervalle | Startup | Cadence/beacon | Mesures/s total |
|---|---|---|---|---|
| 1 | 15 ms | ~165 ms | ~6 Hz | 6 |
| 2 | 30 ms | ~330 ms (≈validé 340) | ~3 Hz | 6 |
| 3 | 45 ms | ~495 ms | ~2 Hz | 6 |
| 4 | 60 ms | ~660 ms | ~1,5 Hz | 6 |
| 5 | 75 ms | ~825 ms | ~1,2 Hz | 6 |

**BOAT_DOCKING** (TX 0 dBm, chmap×2 si N≤2) et **DRONE_INDOOR** (TX max) —
thinning 1 (2 si N≥4), PBR pur, 3B, repeat 2

| N | Intervalle | Startup | Cadence/beacon | Note |
|---|---|---|---|---|
| 1 | 20 ms | ~220 ms | ~4,5 Hz | docking : chaque canal sondé 2× |
| 2 | 40 ms | ~440 ms | ~2,3 Hz | idem ; RAM ~42 Ko de buffers |
| 3 | 60 ms | ~660 ms | ~1,5 Hz | chmap×1 |
| 4 | 80 ms | ~880 ms | ~1,1 Hz | thinning 2 imposé (RAM) |
| 5 | 100 ms | ~1,1 s | ~0,9 Hz | idem ; à réserver aux ancres fixes |

Le débit TOTAL de mesures est ~constant (~6/s drone) : ajouter des beacons
n'ajoute pas de mesures, il les répartit — c'est la géométrie (GDOP) qu'on
achète, pas la cadence. 2D : 3 beacons minimum pour une position univoque ;
4-5 = redondance/intégrité (le scheduler écarte l'ancre bruitée).

## Écarts assumés vs doc_optimised_parameters.ods

- `rtt_type` : AA_ONLY (ODS) → **32_BIT_SOUNDING** en drone (arbitre d'alias
  fiabilisé + NADM, cf. RAPPORT_RTT_SOUNDING.md) ; l'ODS notait « pas besoin
  de précision centimétrique », vrai, mais la marge d'AA_ONLY (±25 m requis)
  était insuffisante sur données réelles.
- `.phy` (procédure) : l'ODS indique 2M_2BT « identique à cs_sync_phy » —
  l'enum de procédure n'a PAS de variante 2BT (le 2BT n'existe que pour les
  paquets CS_SYNC) : `BT_LE_CS_PROCEDURE_PHY_2M` conservé.
- Ambiguïté « 150 m » (ODS) : vrai à channel map plein uniquement ;
  c/(2·thinning·1 MHz) en général (50 m en drone).
- `cp_standby` (latency 499) : supprimé — vestige du round-robin actif/veille,
  incompatible avec le pipeline (tous les liens mesurent en continu). À
  reconsidérer sous une autre forme pour N=5 (parquer les ancres hors round).
- `max_procedure_len 400` / `procedure_interval 7` (ODS) : réglages du mode
  continu abandonné ; en one-shot (count=1) les fenêtres courtes par famille
  placent mieux.

## À valider sur banc (non testé au-delà de N=2)

N=3 drone : premier vrai test de trilatération (supports batterie).
Vérifier à chaque N : 0 abort 0x3, startup ≈ 11 × intervalle, RAM au link.
Repli universel : baisser N, ou famille précision → thinning 2 dès N=3.
