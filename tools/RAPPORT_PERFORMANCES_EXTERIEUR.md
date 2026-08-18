# Rapport de performances — tests extérieur (2026-07-31)

Tests menés le 31/07 en extérieur, initiateur fixe, **carte tournée à 90°**
(rot90 : rotation de l'antenne) pour évaluer la sensibilité à l'orientation.
Session écourtée (pluie) → peu de tests en mode drone.

⚠️ **Vérité terrain = GPS**, donc imprécise (±quelques mètres). Ce rapport
juge donc surtout la **précision/répétabilité** (dispersion des mesures), pas
la justesse absolue — la calibration au laser reste à faire. Un biais
systématique (~+2 m constaté en intérieur) n'est pas encore corrigé.

## Synthèse : l'extérieur est concluant

Contraste net avec l'intérieur. En extérieur, les mesures sont **stables au
centimètre** ; en intérieur (Elektro hall), le multipath les rend erratiques.

| Contexte | σ typique | σ minimal | verdict |
|---|---|---|---|
| Intérieur hall (drone fast) | 2 à 5 m | 0,06 m | multipath → inexploitable |
| **Extérieur drone fast** | 0,2 à 2 m | 0,16 m | **concluant** (peu de tests) |
| **Extérieur boat/docking** | 0,05 à 0,15 m | **0,03 m** | **très bon** en bon lien |

## Mode DOCKING (boat) extérieur — 12 tests

Géométrie fixe, distances mesurées consensus par beacon :

| Beacon | Distance | σ typique | σ min | RSSI |
|---|---|---|---|---|
| b2 | 5,03 m | 0,08 m | **0,03 m** | −59 dBm |
| b1 | 24,98 m | 0,13 m | 0,07 m | −80 dBm |
| b0 | 16,19 m | 0,07 m (bon lien) | 0,07 m | −82 dBm |

**Points forts :**
- **b2 (~5 m, LOS) est irréprochable** sur les 12 tests : σ de 3 à 20 cm,
  aucune valeur aberrante. Répétabilité centimétrique confirmée.
- **b1 à ~25 m tenu à ±7-15 cm** malgré un RSSI faible (−80 dBm) : la
  diversité fréquentielle du mode docking (tous les canaux) paie en extérieur.

**Limite identifiée — sensibilité à l'orientation :**
- b0 est **bimodal** selon la rotation : ~16 m (σ ~0,1 m, bon) sur une partie
  des tests, mais bascule à ~37 m (σ jusqu'à 10 m) sur d'autres orientations,
  avec chute du RSSI (−75 → −84 dBm). b1 se dégrade aussi (jusqu'à 47 m) sur
  les 3 derniers tests.
- Interprétation : à certaines orientations rot90, le **diagramme d'antenne
  atténue le lien** vers b0/b1 (les plus lointains/faibles), et l'estimateur
  accroche alors un trajet parasite. L'orientation de l'antenne compte pour
  les liens déjà limites.

## Mode DRONE fast extérieur — 3 tests (écourté, pluie)

Géométrie fixe, 3 tests consécutifs :

| Beacon | Distance | σ | RSSI | cadence |
|---|---|---|---|---|
| b0 | 5,04 m | 0,2-0,7 m | −55 dBm | 5,2 Hz |
| b1 | 16,18 m | 0,6-2,4 m | −71 dBm | 5,3 Hz |
| b2 | 24,92 m | 0,16-0,5 m | −77 dBm | 5,2 Hz |

**Points forts :**
- **Excellent échantillonnage : ~5,2-5,3 Hz** par beacon, nettement au-dessus
  du mode docking (~3,5-4 Hz, subevents plus longs).
- **Répétabilité remarquable** : sur les 3 tests, les médianes concordent au
  centimètre (b0 5,03/5,04/5,05 ; b2 24,92/24,97/24,87). Le classement RSSI
  suit exactement le classement des distances (proche = fort), signe de
  mesures saines.
- **Robuste à la rotation d'antenne** : contrairement au mode docking, la
  rot90 ne change quasiment pas les distances mesurées en drone fast.

**Réserve :** échantillon très réduit (pluie), à confirmer sur une vraie
campagne. b1 (~16 m) montre un σ plus élevé (jusqu'à 2,4 m) sur 2 des 3 tests.

## Conclusions

1. **L'extérieur valide l'approche** : hors multipath, précision centimétrique
   en docking et cadence 5 Hz + bonne répétabilité en drone fast.
2. **Le multipath intérieur est le facteur limitant n°1** (déjà documenté) —
   confirmé par le contraste des σ (× ~30 entre intérieur et extérieur).
3. **L'orientation d'antenne (rot90)** impacte surtout le mode docking sur les
   liens faibles ; le mode drone fast y est robuste.
4. **À faire** : calibration au laser (biais absolu non mesuré ici, GPS trop
   imprécis) ; campagne drone fast extérieur complète (météo) ; caractériser
   proprement l'effet d'orientation d'antenne (balayage angulaire contrôlé).
