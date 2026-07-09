# Mise à jour RTT sounding sequence — rapport

**Projet :** localisation BLE Channel Sounding (PARNAV), nRF54L15, NCS v3.3.0
**Version :** v3.7 (sur base v3.6 : RAS temps réel + arbitre d'alias RTT)

## Contexte et problème

Depuis la v3.6, la solution PBR fine (phase des tons) est repliée modulo sa portée
non-ambiguë — 50 m en profil drone (thinning 3) — et c'est la mesure RTT des steps
mode-1 qui désigne le bon repli. Cette décision n'est correcte que si l'erreur RTT reste
très en dessous de ±25 m (la demi-période). Or le RTT était configuré en `AA_ONLY` :
l'horodatage des paquets CS_SYNC se fait sur le seul access address, un motif conçu pour
identifier le paquet, pas pour le dater. Les données réelles le montrent : sur la trame
319, les ToA_ToD s'étalent de −117 à +49 demi-nanosecondes d'un step à l'autre, soit une
dispersion de l'ordre de ±25 m par step que seule la médiane sur ~8 steps ramène à
quelques mètres — sans marge de sécurité. Une erreur d'alias vaut 50 m d'un coup : c'est
la pire erreur possible pour le futur filtre de position.

## Modification

Les profils drone (`CONF_DRONE_OUTDOOR_MULTI` et `_SINGLE`) passent en
`CS_RTT_TYPE = BT_CONN_LE_CS_RTT_TYPE_32_BIT_SOUNDING` : les paquets CS_SYNC embarquent
une séquence de 32 bits dédiée à la datation (autocorrélation à pic raide). Les profils
précision (docking, indoor) restent en `AA_ONLY` : en PBR pur, aucun step mode-1 n'est
émis, on ne négocie pas une capacité inutilisée. Le paramètre est centralisé dans
`cs_config.h` (macro `CS_RTT_TYPE` par profil, appliquée dans `cs_config.c`).

Côté Python, `iq_estimation.py` expose désormais le **NADM** (Normalized Attack Detector
Metric) des steps mode-1 pair : ce champ, à 0xFF (indisponible) en AA_ONLY, devient
significatif avec la sounding sequence — le contrôleur compare le motif reçu au motif
attendu. Échelle 0 (altération très improbable) à 6 (très probable), médiane par mesure
dans le champ `nadm` du résultat.

## Effets attendus

Précision RTT par step divisée par un facteur 3 à 5 : l'arbitre d'alias devient fiable
même en NLOS ou à longue portée, et le nombre de steps mode-1 pourrait à terme être
réduit (airtime rendu au PBR). Le NADM fournit un indicateur d'intégrité par mesure —
multipath sévère, relais ou spoofing produisent un motif déformé — argument directement
pertinent pour un système de navigation en environnement GNSS-denied. Coût : quelques
dizaines de µs d'airtime par step mode-1 ; le subevent de 4 500 µs absorbe (au pire le
contrôleur répartit sur un subevent de plus, sans effet sur le startup).

## Plan de test et replis

Reflasher initiateur ET réflecteurs (la capacité est négociée par les deux extrémités ;
le nRF54L15 la supporte). Vérifications dans l'ordre : (1) pas de 0x11/0x12 au Create
Config / Procedure Enable — sinon repli une ligne : `CS_RTT_TYPE` → `AA_ONLY` ;
(2) distances stables, timings inchangés ; (3) capture IQON : la dispersion des
ToA_ToD par step doit visiblement se resserrer vs la trame 319, et le champ `nadm`
n'être plus None ; (4) recalibrer l'offset RTT à distance connue (le group delay change
avec le type de datation — l'ancienne médiane de −35 × 0,5 ns n'est plus valable).
Étapes ultérieures possibles : `96_BIT_SOUNDING` si l'airtime le permet, et
`CS_SYNC_PHY = 2M_2BT` (les deux se multiplient : la sounding donne le motif à dater,
le filtre BT=2 en raidit les fronts) — à tester une variable à la fois.
