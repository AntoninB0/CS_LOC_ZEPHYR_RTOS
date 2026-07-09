# Dump IQ horodaté — firmware ↔ cs_console.py

## Activation
Commande UART `IQON` (désactivation `IQOFF`). Le dump est émis par le thread de
mesure APRÈS chaque fetch réussi, sur l'UART data (uart21) — il n'interfère pas
avec le pipeline CS, mais ajoute le temps d'écriture UART au cycle : ~30-60 ms
par procédure à 115200 bauds. **Passer uart21 à 1 Mbaud (devicetree,
`current-speed`) pour un usage continu** (~4-7 ms), et sélectionner le même
débit dans cs_console.py.

## Format des lignes
```
IQL,<beacon>,<ranging_counter>,<t_ms>,<HEX>     (une ligne PAR SUBEVENT local)
IQP,<beacon>,<ranging_counter>,<t_ms>,<HEX>     (ranging data RAS du pair)
```
- `t_ms` (IQL) : k_uptime de la carte à l'ARRIVÉE du subevent (événement HCI).
  Tous les steps d'un même subevent partagent ce timestamp ; l'instant fin par
  step se reconstruit hors-ligne : ancre du subevent + somme des durées de
  steps précédents (durées déterministes fixées par la config — T_PM, T_IP,
  T_FCS… Core Spec Vol 6 Part B §4.5.18). C'est la réponse pragmatique à
  « timestamp par IQ » : le contrôleur n'expose pas d'horodatage par step.
- `t_ms` (IQP) : t_done de la procédure. Les steps du pair suivent le MÊME
  ordre de canaux que les steps locaux (même procédure) : alignement par index.
- HEX local : flux de steps HCI `{mode(1), canal(1), len(1), data(len)}`.
  Step mode-2 : `antenna_permutation(1)` puis N × `[PCT(3) + tone_quality(1)]`,
  PCT little-endian, I = 12 bits bas, Q = 12 bits hauts (signés).
- HEX pair : ranging data au format RAS, conservé brut dans le JSON
  (`raw_hex`) — le décodage RAS pair pourra être ajouté côté Python.

## JSON produit (cs_console.py, bouton « Capture IQ »)
```json
{ "records": [
  { "type": "local_subevent", "beacon": 0, "ranging_counter": 42,
    "t_board_ms": 123456, "t_host": 1751812345.1, "raw_hex": "…",
    "steps": [ { "idx": 0, "mode": 0, "channel": 2, "raw": "…" },
               { "idx": 2, "mode": 2, "channel": 34,
                 "antenna_permutation": 0,
                 "paths": [ { "i": -512, "q": 1023, "tone_quality": 0 } ] } ] },
  { "type": "peer_procedure", "beacon": 0, "ranging_counter": 42,
    "t_board_ms": 123900, "raw_hex": "…" } ] }
```

## cs_console.py (tools/)
Fenêtre unique : à gauche la console UART (choix du port/baud, envoi de
commandes scheduler avec boutons rapides AUTO/ORDER/IQON/IQOFF, capture IQ →
JSON avec compteur et masquage des lignes IQ) ; à droite les terminaux RTT,
un onglet par carte (sélection de la sonde J-Link par numéro de série, cible
`NRF54L15_M33` ajustable). Dépendances : `pip install pyserial pylink-square`
(pylink optionnel, requis seulement pour le RTT ; driver SEGGER nécessaire).
