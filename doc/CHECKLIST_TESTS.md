# Check-list — campagne de tests & améliorations

État au 2026-07-15. Système validé sur banc : précision IQR 6-40 cm (LOS,
statique), fusion RTT anti-repliement démontrée à ~58 m, NLOS détectable par
divergence phase/RTT. Reste : justesse (biais), dynamique, profils radio.

## Tests à faire (demain)

- [ ] **Calibration justesse** — LE test prioritaire.
      Cartes surélevées, LOS dégagée, UN beacon à la fois au ruban :
      1 / 2 / 4 / 8 / 15 m (+ 1 point loin si possible).
      À chaque point : `python uart_console.py --port /dev/ttyACM0 --test 30`
      → noter (d_vraie, mediane_m du JSON).
      * écart constant → `:cal <offset>` puis figer `CAL_OFFSET_M` dans
        estimation.py ;
      * écart ∝ distance → creuser (erreur d'échelle, à signaler) ;
      * écart ≠ par beacon → `CAL_OFFSET_PAR_BEACON`.
- [ ] **Appariement index ↔ carte physique** : déplacer UN beacon de 1 m,
      identifier son index dans la console ; étiqueter les cartes (b0/b1/b2).
      (Rappel : le 14/07, l'ordre supposé contredisait les RSSI.)
- [ ] **Drift statique** : rien ne bouge, `--test 1800` (30 min).
      Médiane stable ? tracer d(t) depuis `mesures` du JSON (thermique quartz).
- [ ] **Test dynamique** : marcher avec un beacon sur un aller-retour métré,
      vitesse ~1 m/s. Vérifier : le suivi, le retard de la médiane glissante
      (~0,6 s), les sauts de tranche RTT en mouvement.
- [ ] **NLOS volontaire** : masquer un beacon (corps, pilier, sac).
      Confirmer la signature : RSSI qui chute + divergence phase/RTT +
      IQR étalé. Noter l'ampleur de la surestimation de distance.
- [ ] **Portée limite** : éloigner un beacon jusqu'à perte de lien (~ -85 dBm
      constaté vers 58 m en intérieur). Noter portée max et taux de mesures.
- [ ] **Profils/modes radio** (si le temps) : thinning 3 → 2 → 1 dans
      cs_config.h (reflash initiateur SEUL), comparer précision/cadence sur
      le même point à 5 m. Changement de FAMILLE (BOAT/INDOOR) = reflasher
      aussi les 3 réflecteurs — prévoir le hub USB.

## Code — fait

- [x] Offset de calibration : `CAL_OFFSET_M` + `CAL_OFFSET_PAR_BEACON`
      (estimation.py), commande `:cal` en live, offsets tracés dans le JSON
      de chaque test (champ `calibration`).
- [x] `:test [s]` / `--test N` : collecte fenêtrée, rapport par beacon
      (médiane, IQR, σ, outliers, RSSI), données brutes + rapport en JSON.
- [x] Fusion RTT (levée d'ambiguïté 50 m), médiane glissante par beacon.

## Code — à améliorer (par priorité)

- [ ] **Pondération de `find_amp` par l'amplitude** des tonalités (γ par
      tonalité ∝ |z|) : réduire les échappées de lobe (23 % sur b2 au bureau,
      quasi nulles en dégagé). Fichier : estimation.py, ~10 lignes.
- [ ] **Fenêtre de médiane configurable** (`:med N` ?) : 7 = confort statique,
      3 = réactivité vol. À décider après le test dynamique.
- [ ] **Gate NLOS explicite** : exposer |d_rtt − d_phase| et le RSSI comme
      indicateur de confiance dans la sortie (utile au filtre de fusion du
      drone) — faire évoluer estimate() vers un retour riche (dict) tout en
      gardant la console lisible.
- [ ] **uart21 + Raspberry Pi Pico** (dès réception) : `IF_UART_ON_VCOM 0`
      dans if.c + reflash → flux sans pertes, config de vol.
- [ ] **Identité des beacons sur l'UART data** : ligne `MSG,<idx>,<addr>` à
      la connexion (firmware) + commandes REPORTON/REPORTOFF pour l'état
      système côté compagnon drone.
- [ ] **CRC8 en fin de ligne IQ** (firmware + décodeur) si des pertes
      réapparaissent sur le lien final.
- [ ] Cadence : tenter `procedure_interval` 4 → 3 (180 → 135 ms) une fois le
      reste validé — surveiller les aborts 0x3 dans les logs ACM1.
