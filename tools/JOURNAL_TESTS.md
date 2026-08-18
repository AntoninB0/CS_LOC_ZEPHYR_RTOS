# Journal de tests — ranging BLE Channel Sounding

## 2026-07-29 — Tests intérieur, profil drone fast

Tests menés en intérieur (atrium / bureau), profil `DRONE_FAST_OUTDOOR`
(thinning 3, repliement de phase à 50 m), avec 3 réflecteurs.

### Constat principal
En intérieur, le **multipath important dégrade fortement les mesures** en mode
fast : beaucoup d'erreurs sur les distances, et donc sur la position trilatérée.

Symptômes observés :
- Distances des beacons lointains / faible RSSI (−75 à −90 dBm)
  **surestimées** de plusieurs mètres (trajet direct atténué, l'estimateur
  accroche des trajets réfléchis plus longs).
- **Échappées de lobe** fréquentes à faible SNR : la distance saute entre le
  bon lobe et un lobe parasite (ex. un beacon oscillant entre ~8 m et ~23 m).
- Position 3D incohérente : avec 3 balises seulement, le système est
  juste-déterminé (résidu toujours nul), donc une distance biaisée déplace la
  position hors de la zone réelle sans qu'aucun résidu ne l'alerte.
- Géométrie des balises quasi coplanaire → coordonnée verticale (z) très mal
  contrainte, valeurs de hauteur aberrantes.

### Ce qui a fonctionné
- Beacons **proches et en ligne de vue** (bon RSSI) : distances stables et
  justes à quelques dizaines de cm.
- Levée d'ambiguïté au-delà de 50 m par la fusion RTT : validée sur un beacon
  à ~58 m (phase repliée à 8,6 m → position correcte après fusion).
- Cadence : ~5 Hz par beacon en régime établi (procédures free-running).

### Conclusion
Le mode fast n'est pas adapté à l'intérieur à multipath dense : sa faible
diversité de canaux (thinning 3) ne résout pas assez les trajets multiples.

## 2026-07-30 — Plan de tests (demain)

1. **Mode drone fast en extérieur** — valider le profil `DRONE_FAST_OUTDOOR`
   dans son environnement cible (ligne de vue dégagée, peu de multipath).
   Attendu : distances propres, position fiable, portée utile.

2. **Mode docking en intérieur** — profil `BOAT_DOCKING` (précision, tous les
   canaux) : vérifier si la diversité fréquentielle accrue réduit les erreurs
   de multipath constatées aujourd'hui en fast.

3. **Mode docking en extérieur** — même profil en conditions dégagées, comme
   référence de précision courte portée.

Rappel : changer de famille (fast → docking) impose de reflasher initiateur
ET réflecteurs (`./build.sh --boat-n`), et d'adapter `estimation.py` au
repliement du profil (thinning docking → 150 m au lieu de 50 m).
