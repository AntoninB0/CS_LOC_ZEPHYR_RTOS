# Dump IQ horodaté — firmware ↔ tools/ (uart_console + cs_decoder)

## Activation
**ON par défaut** (l'initiateur ne calcule plus de distance : le dump IQ EST sa
sortie ; désactivation `IQOFF`, réactivation `IQON`). Le dump est émis par le
thread de mesure APRÈS chaque fetch réussi, sur l'UART data (uart21) — il n'interfère pas
avec le pipeline CS, mais ajoute le temps d'écriture UART au cycle. L'UART data
est réglé à **921600** (`current-speed` dans l'overlay board — repli depuis
1 Mbaud, le J-Link OB du DK tronquait des lignes) → ~5-8 ms par procédure au
lieu de ~30-60 ms à 115200. Sélectionner le même débit côté hôte
(uart_console.py / cs_decoder.py : 921600 par défaut).

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

## Chaîne hôte (tools/)
- **cs_decoder.py** : décode/valide les lignes (steps IQL, RAS IQP, RTT mode 1),
  apparie local/réflecteur par ranging counter et livre des objets
  `Measurement` complets (`measurements_from_serial` / `measurements_from_lines`).
  Décodage RAS pair inclus — le `raw_hex` brut n'est plus la seule vue.
- **estimation.py** : distance par mesure (IFFT -> vraisemblance von Mises ->
  anti-lobe naboer -> fusion RTT anti-repliement 50 m -> médiane glissante).
- **uart_console.py** : console interactive (commandes IQON/IQOFF/AUTO/ORDER,
  affichage d'une ligne par mesure avec distance) ; mode `--file` pour rejouer
  une capture (ex. tools/capture_ref_921600.txt). Dépendances : install.txt.
