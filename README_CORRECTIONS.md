# cs_localisation — version corrigée (multi-beacon + Channel Sounding)

Base : architecture multi-beacon de `parnav_cs` (scan/pairing/beacon slots, déjà
fonctionnelle) + Channel Sounding repris du sample mono-beacon validé
(`parnav-nrf-apps/channel_sounding_ras_initiator_1`).

Vérifié contre les sources NCS **v3.3.0** (`subsys/bluetooth/services/ras/rreq/ras_rreq.c`).

## Pourquoi cs_localisation1 ne fonctionnait pas

### Bug 1 — `bt_ras_rreq_cp_subscribe()` manquant (bloquant)
`cs_setup_beacon()` souscrivait à RD-ready, RD-overwritten et On-demand RD,
mais **pas au RAS Control Point**. Or `bt_ras_rreq_cp_get_ranging_data()`
fonctionne en écrivant sur le CP et en attendant les réponses **en
indications sur ce même CP**. Sans souscription, la réponse n'arrive jamais
→ timeout systématique de `sem_rd_complete` → toutes les mesures échouent.
Le sample validé appelle `bt_ras_rreq_cp_subscribe()` ; c'est maintenant fait
dans l'étape 4 du setup.

### Bug 2 — Buffer de steps locaux de 512 octets (bloquant)
`STEP_DATA_BUF_LEN 512` ne contient même pas un seul subevent complet. Une
procédure CS produit plusieurs Ko de step data → `Step data buffer overflow`
→ procédure jetée à chaque fois. Remplacé par `LOCAL_PROCEDURE_MEM`
(dimensionnement du sample validé : `BT_RAS_MAX_STEPS_PER_PROCEDURE × (step
header + step data max)` ≈ 11 Ko).

### Bug 3 — Pas d'échange MTU
Le sample validé fait `bt_gatt_exchange_mtu()` avant la découverte GATT.
Sans ça, l'ATT MTU reste à 23 octets et le transfert RAS (plusieurs Ko) se
fait par notifications de 20 octets → extrêmement lent, timeouts. Ajouté en
étape 0 du setup (le Ranging Profile recommande MTU ≥ 247).

### Bug 4 — Paramètres CS non validés
La config divergeait du jeu prouvé : `MAIN_MODE_2_NO_SUB_MODE`, 2-5 steps,
chmap repetition 1, **PHY de procédure 2M** (qui requiert
`CONFIG_BT_TRANSMIT_POWER_CONTROL=y`, absent → rejet HCI possible),
subevent 16000 µs. Tout est revenu au jeu **identique au sample validé** :
mode 2 + sub-mode 1, 10-20 steps, chmap repetition 5, PHY 1M,
subevent 60000 µs, `max_procedure_len = 100`, intervalle 100,
`max_procedure_count = 0` (exigence SDC), antenne A1_B1.
→ `CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT=60000` **des deux côtés**
(initiateur ET réflecteur), sinon HCI 0x12.

### Bug 5 — Stack du display_thread (512 octets)
`LOG_INF("%.2f", ...)` fait le packaging cbprintf dans le contexte appelant ;
512 octets débordent → hard fault aléatoire. Passé à 2048. Ajout aussi de
`CONFIG_FPU_SHARING=y` (deux threads utilisent la FPU : sans ça, corruption
des registres FPU au changement de contexte) et `CONFIG_CBPRINTF_FP_SUPPORT=y`.

### Bug 6 — Lots IQ > 148 échantillons jetés silencieusement
`ble_protocol_data_send_batch()` faisait `return` si `count > 148`. Avec la
config validée une procédure dépasse facilement 148 échantillons → aucune
sortie UART. Désormais découpé en plusieurs trames `R~ … ~E`.

### Bug 7 — Troncatures uint8_t
`estimate_distance_using_phase_slope()` et `linear_regression()` prenaient
des longueurs en `uint8_t` alors que l'index d'échantillons est `uint16_t`
(jusqu'à 1024). Élargi en `uint16_t`.

## Points vérifiés (pas des bugs)
- `CONFIG_BT_RAS_RREQ_MAX_ACTIVE_CONN` = `BT_MAX_CONN` = 5 par défaut → OK.
- L'instance RREQ est **auto-libérée à la déconnexion** par NCS (callback
  `disconnected` interne de ras_rreq.c) → pas de fuite au reconnect.
- `CONFIG_BT_CTLR_SDC_CS_COUNT=5` déjà présent → un contexte CS par lien.
- Le round-robin enable → 1 procédure → disable reste valable
  (`max_procedure_count=0` = mode continu, d'où le disable explicite).

## Fichiers modifiés
- `initiator/src/cs_ranging.c` (réécrit)
- `initiator/src/main.c` (stack display)
- `initiator/src/distance_estimation.c` (uint16_t)
- `initiator/src/ble_protocol_data.c` (découpage en trames)
- `initiator/prj.conf`, `reflector/prj.conf`

## Cadence attendue
Par beacon : ~60-100 ms de procédure + transfert RAS (~50-150 ms à MTU 247)
→ cycle complet sur 5 beacons ≈ 1 à 1,5 s. Pour accélérer ensuite (une fois
que ça marche) : réduire `channel_map_repetition` (5 → 1) et
`max_procedure_len`, en gardant le reste identique.

## Correctif post-test — `Security setup failed (err -12)` en boucle

Symptôme : le 1er beacon se connecte et se sécurise, tous les suivants
échouent immédiatement avec `-ENOMEM` sur `bt_conn_set_security()`, la purge
`bt_unpair(BT_ADDR_LE_ANY)` ne change rien, boucle infinie connect/disconnect.

Cause (vérifiée dans zephyr/subsys/bluetooth/host/keys.c) : le pool de clés
fait `key_pool[CONFIG_BT_MAX_PAIRED]` et **chaque connexion chiffrée occupe
un slot pendant toute la session, même en non-bonding** (la LTK de session y
est stockée). `CONFIG_BT_MAX_PAIRED` n'était pas défini → défaut Zephyr = 1.
Le slot unique appartient au beacon[0] connecté, il est marqué "in use" et
ne peut pas être purgé → tout pairing supplémentaire retourne -ENOMEM.

Le `parnav_cs` fonctionnel avait `CONFIG_BT_MAX_PAIRED=5` ; cette ligne a été
perdue lors du merge. Réintroduite dans `initiator/prj.conf`.

## v4 — Conflits d'ordonnancement multi-lien (abort 0x3)

Cause : une seule radio. Les ancres ACL de chaque lien sont rigides
(supervision timeout) et prioritaires ; un subevent CS de ~16 ms est un bloc
monolithique. Quand un subevent du lien mesuré chevauche une ancre ACL d'un
autre lien, le SDC avorte le subevent (reason 0x3). Avec des intervalles de
connexion identiques, la phase entre liens est figée à la connexion → la
même victime à chaque procédure. Mesures prises, par couches :

1. **Intervalles de connexion distincts par beacon** (pairing.c,
   `32 + 7×idx` unités = 40 / 48,75 / 57,5... ms) : la phase relative
   précesse, les collisions deviennent transitoires.
2. **Retry dans le slot** (cs_ranging.c, 3 tentatives) : une procédure
   perdue est retentée immédiatement. Le callback subevent attend la fin
   RÉELLE de la procédure (un subevent avorté n'arrête pas la procédure,
   procedure reason 0x0) et le disable est SYNCHRONE (attente du
   `procedure_enable_complete` state=DISABLED) — sans quoi le ré-enable
   tombait sur HCI 0x0C Command Disallowed.
3. **Channel map éclairci** (`CS_CHANNEL_THINNING`, défaut 2 = un canal
   valide sur deux, ~36 canaux) : moitié moins d'airtime CS par procédure
   → moitié moins de collisions, et mesures ~2× plus rapides.
   Mettre 1 pour revenir à tous les canaux (ex. si le pipeline IQ exige la
   pleine densité spectrale). Limite : >= 15 canaux requis par la spec,
   donc THINNING <= 4.

Impact IQ du thinning à 2 : pas de fréquence ~4 MHz au lieu de ~2 MHz →
ambiguïté de distance c/(2·Δf) ≈ 37,5 m (sans conséquence en intérieur),
résolution multipath réduite de moitié. Trames UART ~500-550 octets.

Divers : logs de distance en %.2f (l'ancien formatage manuel %d.%02d
affichait "0.-29" pour les valeurs négatives).
