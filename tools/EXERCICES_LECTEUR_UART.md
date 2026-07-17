# Construire son lecteur UART — plan en exercices

Objectif final : un programme Python qui lit **en continu** l'UART de
l'initiateur, reconnaît les lignes IQ horodatées `IQL`/`IQP`, les décode et les
ressort proprement (NDJSON) pour un second programme d'estimation de distance.

Chaque exercice est **autonome et testable**. Corrigé de référence :
[`iq_reader/iq_reader.py`](iq_reader/iq_reader.py) — n'y va qu'après avoir tenté.

## Rappel du format firmware (ce que la carte envoie)

Sur l'UART data, une ligne ASCII par événement (voir [`../IQ_DUMP.md`](../IQ_DUMP.md)) :

```
IQL,<beacon>,<ranging_counter>,<t_ms>,<HEX>     ← un par SUBEVENT local (IQ locaux)
IQP,<beacon>,<ranging_counter>,<t_ms>,<HEX>     ← ranging data RAS du pair (brut)
```

- `beacon` : index du réflecteur (0..N-1)
- `ranging_counter` : numéro de la procédure CS (sert à apparier IQL et IQP)
- `t_ms` : horodatage carte `k_uptime` en ms (le plus fin dispo : par subevent)
- `HEX` : les données en hexadécimal majuscule

Ligne de test (colle-la dans tes scripts sans matériel) — 1 step mode-2,
I = -512, Q = 1023, canal 34 :

```
IQL,0,42,123456,0222050000FE3F00
```

---

## Exercice 0 — Environnement et ouverture du port
**But :** parler à l'UART.

1. Crée un venv, installe `pyserial` (`pip install pyserial`).
2. Liste les ports série disponibles (`serial.tools.list_ports.comports()`).
3. Ouvre le port de la carte (`/dev/ttyACM0` typiquement) à **1000000 bauds**,
   `timeout=1`.
4. Lis des octets bruts (`ser.read(256)`) et affiche-les tels quels.

**Réussite :** tu vois défiler des octets quand la carte tourne (IQON actif par
défaut). Sinon vérifie port/baud (`iq_reader.py --list`).

**Indice :** `import serial` ; `with serial.Serial(port, 1000000, timeout=1) as ser:`

---

## Exercice 1 — Découper le flux en lignes
**But :** l'UART arrive par paquets d'octets, pas par lignes. Il faut bufferiser.

1. Accumule les octets lus dans un buffer `bytes`.
2. Tant que le buffer contient `\n`, découpe la première ligne, traite-la,
   garde le reste.
3. Décode chaque ligne en UTF-8 (`errors="replace"`) et enlève `\r\n`.
4. Affiche une ligne par... ligne.

**Réussite :** l'affichage est net, une mesure par ligne, jamais de ligne
coupée en deux.

**Piège :** ne fais PAS `ser.readline()` naïvement partout ; comprends d'abord
le bufferisation manuelle (tu en auras besoin pour la robustesse plus tard).

---

## Exercice 2 — Ne garder que les lignes IQ
**But :** le firmware envoie aussi des logs sur le même canal éventuellement.

1. Écris `parse_line(line)` qui renvoie `None` si la ligne n'est pas une ligne IQ.
2. Détecte le préfixe `IQL,` ou `IQP,`. **Robustesse :** un log peut préfixer la
   ligne (ex. `[00:00:01] IQL,...`) → cale-toi sur la **dernière** occurrence
   de `IQL,`/`IQP,` (`line.rfind`).
3. Affiche seulement les lignes reconnues.

**Test :** `parse_line("LOG: hello")` → `None` ; `parse_line("[t] IQL,0,42,1,00")`
→ reconnu.

---

## Exercice 3 — Extraire les champs d'en-tête
**But :** transformer une ligne en champs exploitables.

1. Découpe la ligne en 5 morceaux : `tag, beacon, counter, t_ms, hex` — attention,
   le HEX ne contient pas de virgule mais fais quand même un `split(",", 4)`
   (limite à 4 coupures) par sécurité.
2. Convertis `beacon`, `counter`, `t_ms` en `int` (protège avec `try/except`).
3. Construis un dict :
   ```python
   {"type": "local_subevent" if tag == "IQL" else "peer_procedure",
    "beacon": ..., "ranging_counter": ..., "t_board_ms": ...,
    "t_host": time.time(), "raw_hex": ...}
   ```

**Réussite :** sur la ligne de test → `beacon=0, ranging_counter=42,
t_board_ms=123456`.

---

## Exercice 4 — Décoder les IQ locaux (lignes IQL)
**But :** le cœur du sujet — sortir les échantillons I/Q.

Le HEX d'une ligne IQL est un flux de **steps** HCI concaténés. Chaque step :

```
mode(1 octet)  channel(1 octet)  len(1 octet)  data(len octets)
```

Seuls les steps **mode 2** portent des IQ. Leur `data` :

```
antenna_permutation(1)  puis  N × [ PCT(3 octets) + tone_quality(1) ]
```

Le PCT (3 octets, little-endian) encode deux entiers 12 bits **signés** :
`I` = bits 0-11, `Q` = bits 12-23.

1. Écris `parse_local_steps(hexstr)` : boucle sur les steps (avance de
   `3 + len`), et pour chaque step mode 2 extrais la liste des `paths`
   `{"i", "q", "tone_quality"}`.
2. Le nombre de paths se déduit de `len` : `n_paths = (len - 1) // 4`.
3. Conversion 12 bits signé : `v - 4096 if v & 0x800 else v`.
4. Ajoute la liste `steps` au dict des lignes IQL.

**Test (ligne de référence)** `0222050000FE3F00` doit donner :
- 1 step, `mode=2`, `channel=34`, `antenna_permutation=0`
- `paths=[{"i": -512, "q": 1023, "tone_quality": 0}]`

**Vérif du décodage :** PCT = `00 FE 3F` → `0x3FFE00` → bas 12 bits `0xE00`=3584
→ signé -512 ; hauts 12 bits `0x3FF`=1023. ✔

*(Les lignes IQP restent en `raw_hex` : leur décodage RAS est fait par le
programme d'estimation, cf. `parse_peer_ras` dans `iq_estimation.py`.)*

---

## Exercice 5 — Sortie exploitable (NDJSON)
**But :** produire un flux qu'un autre programme peut consommer.

1. Pour chaque record, `print(json.dumps(rec, separators=(",",":")), flush=True)`
   → **une ligne JSON par record** (NDJSON) sur stdout.
2. En parallèle, affiche un résumé lisible sur **stderr** (nb de steps, t_ms) —
   comme ça stdout reste « propre » pour un pipe.
3. Option `--out fichier` pour aussi écrire dans un fichier (mode append).

**Réussite :** `python ton_lecteur.py --port ... > mesures.ndjson` remplit un
fichier d'objets JSON, un par ligne, pendant que le résumé s'affiche à l'écran.

**Pourquoi NDJSON :** ça se pipe (`ton_lecteur.py | ton_estimateur.py`), se
relit ligne par ligne sans tout charger, et reste compatible avec le schéma de
`cs_console.py` / `IQEstimator.feed()`.

---

## Exercice 6 — Robustesse (indispensable en continu)
**But :** tourner des heures sans planter.

1. `try/except` autour de tout parsing : une ligne corrompue (bruit UART) ne
   doit jamais tuer le programme, juste être ignorée.
2. **Reconnexion auto** : si le port disparaît (reflash de la carte,
   `serial.SerialException`), attends 2 s et rouvre — boucle infinie.
3. `Ctrl-C` (`KeyboardInterrupt`) doit fermer proprement (fichier flush/close).
4. Argparse : `--port`, `--baud` (défaut 1000000), `--out`, `--quiet`, `--list`.

**Test reconnexion :** débranche/rebranche la carte pendant que ça tourne → le
lecteur doit repartir tout seul.

---

## Exercice 7 (bonus) — Apparier IQL et IQP en une mesure complète
**But :** une procédure CS = plusieurs IQL (subevents) + 1 IQP (pair), à recoller.

1. Maintiens un dict `pending[(beacon, counter)]`.
2. Accumule les `steps` des IQL et le `peer` de l'IQP pour la même clé.
3. Quand les deux moitiés sont là → émets la mesure complète, retire la clé.
4. Purge les demi-mesures orphelines (une moitié jamais complétée) au-delà d'un
   seuil (ex. 32 clés en attente).

**Réf :** classe `IQEstimator` dans `iq_estimation.py` fait exactement ça
(`feed()`).

---

## Exercice 8 (bonus) — Brancher l'estimation de distance
**But :** boucler la chaîne (mais dans un programme SÉPARÉ, comme prévu).

1. Nouveau script qui lit le NDJSON de ton lecteur (stdin, ligne par ligne).
2. `from iq_estimation import IQEstimator` ; pour chaque record `est.feed(rec)`.
3. Quand `feed` renvoie un dict → affiche `beacon`, `t_board_ms`, distance
   (`bayesian`).
4. Chaîne complète :
   ```bash
   python ton_lecteur.py --port /dev/ttyACM0 --quiet | python ton_estimateur.py
   ```

**Réussite :** des distances s'affichent en temps réel, calculées hors carte, à
partir des IQ horodatés — l'objectif initial du nettoyage.

---

## Ordre de difficulté / dépendances
```
0 → 1 → 2 → 3 → 4 → 5 → 6        (lecteur complet, = iq_reader.py)
                     └→ 7 → 8    (appariement + estimation, bonus)
```
Fais 0→6 d'abord : à la fin tu as un lecteur robuste et utile. 7-8 ne sont utiles
que si tu veux aussi faire l'estimation (sinon c'est le rôle du programme séparé).
