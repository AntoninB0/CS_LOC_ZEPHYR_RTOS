# Méthode de l'étude — consistance des IQ (Channel Sounding)

Texte de méthode (FR), destiné au rapport. Décrit le principe de l'étude, la
métrique, la vérification par les quantiles, et la procédure d'exécution des
tests.

---

## 1. Objet et principe

L'étude porte sur l'**acquisition** : évaluer la qualité avec laquelle une
configuration de Channel Sounding **échantillonne le canal**, indépendamment de
tout estimateur de distance.

Le choix de ne **pas** conclure sur la distance est délibéré. La distance
estimée est une grandeur *en aval* dont l'erreur est dominée par des facteurs
extérieurs à l'acquisition (sélection de lobe de l'estimateur, multitrajet,
calibration du group delay). Elle est donc peu fiable comme indicateur de la
qualité d'échantillonnage, et elle interdit les tests en mouvement (l'exploitation
des IQ suppose un canal **statique**).

On mesure à la place la **consistance des IQ** : sur un montage statique, le
canal ne varie pas, donc la phase mesurée à chaque fréquence doit être
**stable dans le temps**. La consistance mesure cette stabilité, au niveau
physique, avec une métrique standard.

---

## 2. La métrique : cohérence `R`

Pour chaque canal *k*, sur une capture statique de *T* procédures :

> **R_k = | (1/T) · Σ_t exp( j · arg( Y_I(t) · Y_R(t) ) ) |**   ∈ [0, 1]

- `Y_I` et `Y_R` sont les échantillons IQ complexes des deux sens (initiateur et
  réflecteur). Le **produit `Y_I · Y_R`** est la mesure aller-retour : il annule
  le décalage d'oscillateur local commun.
- `R_k` est la **longueur du vecteur résultant moyen** (statistique circulaire),
  c'est-à-dire la **cohérence** du canal : `R = 1` → phase parfaitement stable,
  `R = 0` → phase aléatoire. En pratique, **R ≈ la qualité/SNR du tone**.

**Score global** d'un test : la **médiane** des `R_k` ; le **10ᵉ percentile**
signale les canaux les plus faibles (fadés).

**Propriété clé — pas de calibration.** `R` est **invariante à tout offset de
phase constant** : en décomposant `φ_k(t) = φ₀ + δ_k(t)` (partie constante +
variation temporelle), le terme constant `e^{jφ₀}` a module 1 et disparaît du
module de la moyenne. Le **group delay** et la **distance absolue** sont deux
offsets constants → ils s'annulent. Aucune calibration n'est donc requise ; la
distance n'intervient que comme **condition d'essai** (elle fixe le bilan de
liaison → le SNR → le fading → `R`).

---

## 3. Vérification par les quantiles (« top X % »)

La métrique globale ne dit pas *combien* de canaux sont exploitables. Pour cela,
on trie les canaux par cohérence décroissante et on trace la **courbe de
fiabilité vs fraction gardée** (quantile de cohérence) :

- Abscisse : **% de canaux gardés**, triés du meilleur au pire.
- Ordonnée : le **pire `R` parmi ce top-x %**.

Lecture directe : le point à *x = 20 %* signifie *« si l'on garde les 20 % de
canaux les plus cohérents, ils ont tous `R ≥ y` »*. Les percentiles usuels
(10 %, 20 %, 50 %, 80 %) sont marqués sur la courbe.

**Ce que la courbe vérifie :**
- **Combien de canaux sont fiables** : une courbe qui reste haute longtemps →
  beaucoup de canaux exploitables ; une courbe qui chute vite → peu.
- **Un classement des configurations** : l'**aire sous la courbe** résume la
  fiabilité globale ; superposées, les courbes de plusieurs configs se classent
  sans ambiguïté.

**Nuance à écrire :** garder moins de canaux augmente `R` moyen **mais détruit la
diversité fréquentielle** (l'étendue spectrale → la résolution et la période
d'alias). La courbe n'invite donc pas à « garder le top 10 % » ; elle quantifie
un **compromis qualité ↔ diversité** et indique quels canaux sont *à retirer*
(les creux de fading), pas à réduire l'ensemble à quelques tones.

*(Rapprochement : `R` est la **cohérence interférométrique** standard en imagerie
radar/InSAR et un indicateur de verrouillage de phase en GNSS ; sélectionner les
meilleurs canaux est analogue à la détection du premier trajet en UWB ou à la
sélection de sous-porteuses fiables en OFDM.)*

---

## 4. Comment lancer les tests

### Préparation
1. Choisir le **profil** dans `initiator/src/cs_config.h` (une seule famille
   `CONF_*`) et le nombre de réflecteurs `CS_NUM_BEACONS`.
2. Compiler et flasher :
   ```bash
   ./build.sh --boat-n --initiator-only     # boat/indoor, N>=2 ; --boat-1 pour N=1 ; drone = aucun flag
   ```
   ⚠ Changement de **famille** (drone ↔ boat/indoor) → reflasher aussi les
   réflecteurs (`--clean`, sans `--initiator-only`).
3. Montage **statique** et **calme** (pas de passage, 2,4 GHz peu chargé),
   distances/hauteur/orientation d'antenne **notées** (métadonnées, pas de
   calibration).

### Acquisition
```bash
cd tools && source .venv/bin/activate
python uart_console.py --list                 # repérer le port data (celui qui sort des IQL/IQP)
python uart_console.py --port /dev/ttyACM0 --test 30 --note "boat_thin1_5m_N1"
```
- `--test 30` collecte 30 s (60 s conseillé à N ≥ 2, couverture par canal
  partielle).
- `--note` étiquette le test (profil + condition) → sert de label aux
  comparaisons.
- Le test produit automatiquement, dans **`tests/AAAAMMJJ/test_<stamp>/`** :
  le JSON (données + IQ + durée + note), `config.txt` (snapshot config firmware),
  le tracé temporel, `estimateur/` (figures IFFT/bayésien) et
  **`consistency_bX.png`** (le rapport de consistance par beacon).

### Analyse
```bash
# rapport d'un test (4 panneaux)
python iq_consistency.py --json ../tests/<jour>/<test>/<test>.json --report

# comparaison de configs (courbes top-X% superposées, étiquetées par --note)
python iq_consistency.py --json ../tests/<jour>/<A>/<A>.json ../tests/<jour>/<B>/<B>.json --report --out compare.png
```
Lire la médiane `R`, la courbe top-X% et le panneau R-vs-amplitude.

### Scénarios (axe principal = les profils livrés)
1. **Comparaison des profils** à géométrie propre identique (drone / boat /
   indoor) — le résultat central.
2. **Chaque profil dans son environnement cible** (drone→extérieur,
   boat→accostage proche, indoor→multitrajet).
3. **Robustesse croisée** profil × environnement.
4. **Single vs multi-beacon** (N = 1 / 2 / 3) — couverture par procédure +
   interférence inter-liens.
5. Réglages ciblés (boat thinning 1 vs 2, TX en portée), antenne, reproductibilité.

Volume : **~25–30 tests** suffisent (chaque test est déjà moyenné sur ~150
procédures × ~72 canaux) ; le coût est le **reflash** par changement de config,
pas le nombre de tests — donc grouper les configs à une même géométrie.
