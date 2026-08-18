#ifndef CS_CONFIG_H_
#define CS_CONFIG_H_

/*io pin trigger par une interruption pour chaque cs ranging et envoyer a la sentiboard pour avoir un timestamp */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * 1) FAMILLE APPLICATIVE — décommenter EXACTEMENT UNE.
 * 2) CS_NUM_BEACONS — nombre de réflecteurs (1 à 5). TOUT le timing et le
 *    dimensionnement en découlent automatiquement (intervalle = N × créneau,
 *    buffers, scheduler) ; garde-fous BUILD_ASSERT dans cs_config.c.
 *
 * Overlays prj.conf par famille (réservations SDC, indépendantes de N) :
 *   DRONE_FAST_OUTDOOR : aucun (prj.conf de base : CS_EVENT 5000/SPACING 15000)
 *   BOAT_DOCKING / DRONE_INDOOR, N>=2 : overlay-precision-multi.conf
 *   BOAT_DOCKING / DRONE_INDOOR, N==1 : overlay-precision-single.conf
 * Rappel : reflasher aussi les réflecteurs si CS_EVENT_LEN change de famille.
 * Référence des paramètres : doc_optimised_parameters.ods (renvois Vol 6 H).
 * ═══════════════════════════════════════════════════════════════════════════ */
//#define CONF_DRONE_FAST_OUTDOOR   /*drone extérieur rapide */
#define CONF_BOAT_DOCKING /*   accostage bateau, précision courte portée */
//- #define CONF_DRONE_INDOOR    /* drone lent intérieur, multipath dense */

#define CS_NUM_BEACONS 2          /* 1..5 */

/* Mode de récupération des ranging data pair : 1 = RAS temps réel (fetch ~0,
 * pas de rétention), 0 = on-demand (fallback validé). Voir itération 3. */
#define CS_RAS_REALTIME 1

/* ─────────────────────────────────────────────────────────────────────────
 * FAMILLE : DRONE EXTÉRIEUR RAPIDE
 * Priorité : cadence + robustesse au mouvement. Peu de canaux (airtime et
 * data minimaux), créneau de 15 ms/lien, RTT sounding (levée d'ambiguïté
 * fiable au-delà de 50 m). NOTE capacités SDC : 3C/X, 2M_2BT, sounding et
 * NADM documentés dans l'ODS ne sont PAS supportés par le SoftDevice
 * Controller (Create Config 0x11 constaté) — voir le bloc CS_RTT_TYPE.
 * ───────────────────────────────────────────────────────────────────────── */
#if defined(CONF_DRONE_FAST_OUTDOOR)
#define CS_CHANNEL_THINNING     3     /* ~24 canaux, non-ambigu PBR 50 m */
#define CS_SUBEVENT_LEN_US      4500
#define CS_PROC_LEN_UNITS       40    /* 25 ms : fenêtre courte, placement facile */
#define CS_MODE0_STEPS          2
#define CS_MAIN_STEPS_MAX       6
#define CS_MODE                 BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1 /*tester sans submode
/* ⚠ CAPACITÉS SDC (table nrfxlib, vérifiée sur cible — Create Config 0x11
 * sinon) : sounding sequence, CSA #3C, 2M_2BT et NADM NON SUPPORTÉS par le
 * SoftDevice Controller. Le levier de datation RTT DISPONIBLE est la
 * random payload (32/64/96/128 bits, supportée) : corrélation affinée vs
 * AA-only, même rôle que la sounding d'après la spec (§2, Vol 6 Part H).
 * Fallback validé banc : BT_CONN_LE_CS_RTT_TYPE_AA_ONLY. */
#define CS_RTT_TYPE             BT_CONN_LE_CS_RTT_TYPE_32_BIT_RANDOM
#define CS_SYNC_PHY             BT_CONN_LE_CS_SYNC_2M_PHY      /* 2M_2BT non supporté SDC */
#define CS_CHSEL_TYPE           BT_CONN_LE_CS_CHSEL_TYPE_3B    /* #3C non supporté SDC */
#define CS_CH3C_SHAPE           BT_CONN_LE_CS_CH3C_SHAPE_HAT   /* sans effet en 3B */
#define CS_CHMAP_REPETITION     1
#define CS_MAX_TX_POWER         BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER
#define CS_SPACING_UNITS        12    /* 15 ms/lien = ACL 3750 + CS 5000 + marge */
#define CS_SINGLE_FLOOR_UNITS   12
/* RAM : à 4-5 beacons, buffers réduits (24 canaux → ~30 steps réels). */
#define CS_MAX_STEPS_PER_PROC   (CS_NUM_BEACONS >= 4 ? 48 : 64)
#define CS_SCHED_DEFAULT_REPEAT 1

/* ─────────────────────────────────────────────────────────────────────────
 * FAMILLE : ACCOSTAGE BATEAU (précision, < 30 m, dynamique lente)
 * Tous les canaux = résolution max ; TX 0 dBm (anti-saturation proche) ;
 * cible lente et proche → channel_map_repetition 2 quand la RAM le permet
 * (N<=2, cf. note ODS), sinon moyennage par le scheduler (2 mesures/round).
 * PBR pur (non-ambigu 150 m >> enveloppe) → pas de RTT, chsel 3B (statique).
 * ⚠ N>=4 : thinning 2 imposé (75 m non-ambigu, RAM ~÷2 par beacon).
 * ───────────────────────────────────────────────────────────────────────── */
#elif defined(CONF_BOAT_DOCKING)
#define CS_CHANNEL_THINNING     (CS_NUM_BEACONS >= 3 ? 2 : 1)
#define CS_SUBEVENT_LEN_US      8000
#define CS_PROC_LEN_UNITS       100   /* 62,5 ms : ~72 canaux à écouler */
#define CS_MODE0_STEPS          3
#define CS_MAIN_STEPS_MAX       10
#define CS_MODE                 BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE
#define CS_RTT_TYPE             BT_CONN_LE_CS_RTT_TYPE_AA_ONLY /* sans objet en PBR pur */
#define CS_SYNC_PHY             BT_CONN_LE_CS_SYNC_2M_PHY
#define CS_CHSEL_TYPE           BT_CONN_LE_CS_CHSEL_TYPE_3B
#define CS_CH3C_SHAPE           BT_CONN_LE_CS_CH3C_SHAPE_HAT   /* inutilisé en 3B */
#define CS_CHMAP_REPETITION     (CS_NUM_BEACONS <= 2 ? 2 : 1)
/* Puissance TX max (EIRP) de TOUTES les transmissions CS. PLAFOND : le
 * contrôleur prend le niveau matériel supporté le plus proche <= cette valeur.
 * Plage HCI -127..+20 dBm ; nRF54L15 : max réel +8 dBm (au-delà = plafonné).
 * BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER (=20) = "prends le maximum matériel".
 * ⚠ Baisser la TX ne réduit PAS le multipath (direct et réflexions atténués du
 *   même rapport) : le 0 dBm ci-dessous est de l'anti-saturation courte portée
 *   (< qq m, un signal trop fort déforme la phase), pas un filtre multipath.
 *   À distance / en extérieur, MONTER la TX pour le SNR.
 * Paliers indicatifs (dBm) :
 *      0    accostage serré < ~3 m (anti-saturation)      <-- valeur actuelle
 *     +4    intérieur / quelques mètres
 *     +8    extérieur, portée max nRF54L15
 *   BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER   laisse le contrôleur prendre son max */
#define CS_MAX_TX_POWER         8    /* dBm */
#define CS_SPACING_UNITS        16    /* 20 ms/lien = ACL 3750 + CS 8500 + marge */
#define CS_SINGLE_FLOOR_UNITS   16
#define CS_MAX_STEPS_PER_PROC   (CS_NUM_BEACONS <= 2 ? 176 : \
                                 (CS_NUM_BEACONS == 3 ? 96 : 64))
#define CS_SCHED_DEFAULT_REPEAT 2

/* ─────────────────────────────────────────────────────────────────────────
 * FAMILLE : DRONE LENT INTÉRIEUR (multipath dense)
 * La diversité fréquentielle est LE remède au multipath : tous les canaux,
 * mode-0 renforcé, TX max (budget NLOS). Cible lente → 3B suffit ; le
 * lissage vient du scheduler (2 mesures/round) + pondération γ côté Python.
 * ⚠ N>=4 : thinning 2 imposé (RAM), à éviter si le multipath est sévère.
 * ───────────────────────────────────────────────────────────────────────── */
#elif defined(CONF_DRONE_INDOOR)
#define CS_CHANNEL_THINNING     (CS_NUM_BEACONS >= 4 ? 2 : 1)
#define CS_SUBEVENT_LEN_US      8000
#define CS_PROC_LEN_UNITS       100
#define CS_MODE0_STEPS          3
#define CS_MAIN_STEPS_MAX       10
#define CS_MODE                 BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE
#define CS_RTT_TYPE             BT_CONN_LE_CS_RTT_TYPE_AA_ONLY
#define CS_SYNC_PHY             BT_CONN_LE_CS_SYNC_2M_PHY
#define CS_CHSEL_TYPE           BT_CONN_LE_CS_CHSEL_TYPE_3B
#define CS_CH3C_SHAPE           BT_CONN_LE_CS_CH3C_SHAPE_HAT
#define CS_CHMAP_REPETITION     1
#define CS_MAX_TX_POWER         BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER
#define CS_SPACING_UNITS        16
#define CS_SINGLE_FLOOR_UNITS   16
#define CS_MAX_STEPS_PER_PROC   (CS_NUM_BEACONS >= 4 ? 64 : 96)
#define CS_SCHED_DEFAULT_REPEAT 2

#else
#error "cs_config.h : sélectionner exactement une famille CONF_*"
#endif

/* ── Dérivés communs (ne pas éditer) ────────────────────────────────────────
 * Intervalle de connexion : N==1 → plancher single (le quadrillage SPACING ne
 * s'applique qu'entre connexions centrales) ; N>=2 → N × créneau, le SDC
 * espace les ancres ACL de SPACING (déterministe, sans collision).
 * Startup ≈ 11 × intervalle (LL_CS_REQ/RSP/IND + marge SDC, incompressible
 * en nombre), payé UNE FOIS à l'armement en free-running
 *#define CS_CHANNEL_THINNING  (CS_NUM_BEACONS >= 3 ? 2 : 1) (max_procedure_count = 0). La cadence en régime établi vaut ensuite
 * procedure_interval × intervalle de connexion par beacon (∝ N). */
#define CS_MAX_BEACONS          CS_NUM_BEACONS
#define CS_CONN_INTERVAL_UNITS  (CS_NUM_BEACONS == 1 ? CS_SINGLE_FLOOR_UNITS \
                                 : CS_NUM_BEACONS * CS_SPACING_UNITS)
#define CS_ACL_DESYNC_STEP      0  /* précession inutile avec le quadrillage ;
                                    * fallback 6 si aborts 0x3 figés */

extern struct bt_le_cs_set_default_settings_param     cs_settings;
extern struct bt_le_cs_create_config_params           cs_create_config;
extern struct bt_le_cs_set_procedure_parameters_param cs_proc_params;
extern struct bt_le_conn_param                        cs_cp_active;
/* cs_cp_standby SUPPRIMÉ : vestige round-robin actif/veille. Perspective
 * N>=4 (note ODS sur .id/config par réflecteur) : parquer en latency les
 * beacons hors du round courant — non implémenté. */

#endif /* CS_CONFIG_H_ */
