# Trace — profils applicatifs CS, scheduler priorisé et commande UART

Base : v2.1 validée sur banc (startup 340 ms recouvert, fetch ~225 ms, cycle 2 beacons ~510 ms).

---

## 1. Profils applicatifs (cs_config.h)

L'ancienne configuration unique `CONF_DRONE_OUTDOOR_MULTI` devient l'un de quatre profils
sélectionnables par `#define` en tête de `cs_config.h`. Toutes les valeurs dépendantes du
profil (thinning, subevent, intervalle de connexion, nombre de beacons, puissance TX,
dimensionnement des buffers, répétitions du scheduler) sont désormais des macros `CS_*`
consommées par `cs_config.c`, `beacon.h` (MAX_BEACONS) et `cs_ranging.c`. Les profils
DOCKING et INDOOR ont un sous-réglage `CS_NUM_REFLECTORS` (1 ou 2).

| | DRONE_MULTI (validé) | DRONE_SINGLE | DOCKING_PRECISION | INDOOR_PRECISION |
|---|---|---|---|---|
| Réflecteurs | 2 | 1 | 1 ou 2 | 1 ou 2 |
| Thinning (canaux) | 3 (~24) | 2 (~36) | 1 (~72) | 1 (~72) |
| Non-ambiguïté c/(2Δf) | 50 m | 75 m | 150 m | 150 m |
| Subevent | 4500 µs | 6000 µs | 8000 µs | 8000 µs |
| Intervalle connexion | 30 ms | **15 ms** | 40 / 20 ms | 40 / 20 ms |
| mode_0_steps | 2 | 2 | 3 | 3 |
| Puissance TX | max | max | **0 dBm** | max |
| Mesures/beacon/round | 1 | 1 | 2 | 2 |
| Startup attendu | 340 ms (mesuré) | ~165 ms | ~440 / ~220 ms | ~440 / ~220 ms |

Logique de conception, en deux axes. **Vitesse (drone)** : peu de canaux → subevent court →
réservations petites → intervalle court → startup court ; en réflecteur unique, le
quadrillage inter-liens disparaît et l'intervalle plonge à 15 ms. **Précision
(docking/intérieur)** : tous les canaux — la résolution en délai et la robustesse au
multipath viennent de la largeur de bande couverte —, mode-0 renforcé (meilleure estimation
d'offset fréquentiel), et moyennage par répétition de mesures via le scheduler, la cible
étant lente. Deux différences entre docking et intérieur : la puissance TX (réduite à 0 dBm
en docking : à quelques mètres, la pleine puissance comprime le récepteur et biaise la
phase ; maximale en intérieur où le NLOS coûte du budget de liaison) ; et l'usage — le
docking est typiquement LOS courte portée, l'intérieur mise tout sur la diversité
fréquentielle. Attention profils précision : buffers dimensionnés à 96 steps (~72 canaux),
soit environ +6 Ko de RAM par rapport au profil drone — à surveiller au link.

## 2. La règle de couplage CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT

Réponse à la question posée : oui, ce symbole est bien dans **`initiator/prj.conf`**
(c'est un Kconfig du SoftDevice Controller, donc figé au build — il ne peut pas vivre dans
`cs_config.c`). Il ne concerne que l'initiateur (option *central only*) ; le réflecteur n'a
que `CS_EVENT_LEN_DEFAULT` à suivre.

Quand on change les paramètres CS d'un profil, la chaîne de dépendances à respecter est :

```
max_subevent_len  (cs_config.c, µs)
      ↓  doit être couvert par
CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT          (réservation d'un événement CS)
      ↓  entre dans
SPACING >= MAX_CONN_EVENT_LEN (3750) + CS_EVENT_LEN + garde (>= 3000 µs)
      ↓  dimensionne
intervalle de connexion >= N_liens_centraux × SPACING
```

En clair : si tu allonges le subevent (plus de canaux, plus de steps), la réservation CS
grossit, donc le créneau par lien (SPACING) grossit, donc l'intervalle de connexion minimal
grossit — et le startup (~11 × intervalle) avec lui. C'est le compromis structurel
précision ↔ cadence. La garde de ~3 ms n'est pas négociable : l'essai à 1250 µs de garde
(20 ms / 10000) a produit des aborts 0x3 structurels. Et en réflecteur **unique**, le
SPACING ne s'applique tout simplement pas (il n'espace que des connexions centrales entre
elles) : seul le plancher `intervalle >= ACL + CS_EVENT_LEN + marge` demeure.

Deux garde-fous rendent l'erreur impossible à rater : des **BUILD_ASSERT** dans
`cs_config.c` refusent de compiler si le prj.conf ne couvre pas le profil (c'est exactement
la classe d'erreurs qui avait produit les 700 ms de startup et les aborts historiques), et
chaque profil a son **overlay** :

| Profil | Commande de build (extrait) | CS_EVENT_LEN | SPACING |
|---|---|---|---|
| DRONE_MULTI | build normal (prj.conf de base) | 5000 | 15000 |
| DRONE_SINGLE | `-DEXTRA_CONF_FILE=overlay-drone-single.conf` | 6500 | défaut (sans effet) |
| DOCKING/INDOOR ×2 | `-DEXTRA_CONF_FILE=overlay-precision-multi.conf` | 8500 | 20000 |
| DOCKING/INDOOR ×1 | `-DEXTRA_CONF_FILE=overlay-precision-single.conf` | 8500 | défaut (sans effet) |

Ne pas oublier de reporter le `CS_EVENT_LEN_DEFAULT` du profil dans le prj.conf des
**réflecteurs** et de les reflasher.

## 3. Scheduler de mesures (scheduler.c/h) et commande UART

Le tour de rôle figé `0,1,0,1…` de la boucle principale est remplacé par des **rounds**
construits par un scheduler, avec deux modes.

**Mode AUTO (défaut).** Chaque beacon prêt reçoit `CS_SCHED_DEFAULT_REPEAT` slot(s) de base
(1 en profil drone, 2 en précision), puis des slots bonus vont aux beacons dont l'erreur
estimée est la plus faible. L'estimateur, entretenu par `scheduler_report()` après chaque
mesure, est une EWMA (α = 0,2) du jitter |dᵢ − dᵢ₋₁| entre distances successives, avec une
pénalité de 1 m injectée à chaque échec de procédure ou de fetch. Rationale : une ancre
stable porte plus d'information par mesure, on la rafraîchit plus souvent ; une ancre
bruitée ou en échec perd ses bonus mais garde toujours son slot de base — elle reste
observée et récupère ses slots dès qu'elle redevient propre. Le round est ensuite
**entrelacé** (round-robin sur les compteurs de slots) : deux procédures ne pouvant pas
s'empiler sur un même lien, un doublon adjacent paierait son startup à découvert, alors
qu'entrelacé il reste recouvert par les fetchs des autres slots.

**Mode MANUAL (UART).** L'`uart21` (celle qui sort déjà les données sérialisées) reçoit
désormais des lignes ASCII terminées par `\n` ou `\r` :

```
ORDER:0,1,1        → mode MANUAL : cet ordre de passage est rejoué à chaque round
                     (répétitions permises : ici le beacon 1 est mesuré 2×/round)
AUTO               → retour au mode automatique
```

Une ligne invalide (index hors [0, MAX_BEACONS-1], ORDER vide, commande inconnue) est
ignorée avec un log d'avertissement, sans toucher au mode courant ; un beacon de la
séquence manuelle qui se déconnecte est sauté, et si la séquence se vide le filet AUTO
reprend. Implémentation RX : ISR UART (interrupt-driven, `CONFIG_UART_INTERRUPT_DRIVEN=y`
ajouté au prj.conf) qui accumule la ligne puis délègue le parsing à la system workqueue —
le handler tourne en contexte thread et peut prendre les mutex. La boucle de `main.c`
consomme `scheduler_build_round()` en conservant intégralement le pipeline v2.1
(wait_done → ré-enable avant fetch → fetch sur snapshot) et nourrit `scheduler_report()`.

## 4. Incident de séance

Une édition scriptée de `main.c` (remplacement par motif trop laxiste) a corrompu le
fichier (19 Mo) ; restauré depuis l'archive v2.1 et réédité par remplacements uniques
vérifiés. Morale : les archives d'étape servent aussi de points de restauration.

## 5. À valider sur banc

Le code compile-t-il et les profils tiennent-ils leurs prédictions : DRONE_SINGLE
(startup ~165 ms), précision multi (~440 ms mais ~72 canaux mesurés — la précision
attendue justifie-t-elle la cadence ?). Vérifier ensuite l'effet du scheduler : en AUTO
avec un beacon volontairement dégradé (masqué), il doit perdre ses bonus sans jamais
disparaître du round ; par UART, `ORDER:0,0,1` doit se voir immédiatement dans le motif
des logs `b0/b0/b1`.
