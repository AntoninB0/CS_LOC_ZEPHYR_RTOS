#include "cs_config.h"

/* ── GARDE-FOUS DE COHÉRENCE prj.conf ↔ profil (voir cs_config.h) ──────────
 * Refusent la compilation si les réservations de timeline du SDC ne couvrent
 * pas le profil sélectionné — c'est exactement la classe d'erreurs qui a
 * produit les 700 ms de startup et les aborts 0x3 historiques. */
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT >= CS_SUBEVENT_LEN_US,
	     "CS_EVENT_LEN_DEFAULT < max_subevent_len : Procedure Params rejetés (0x12). "
	     "Adapter l'overlay prj.conf au profil.");
BUILD_ASSERT(CS_NUM_BEACONS >= 1 && CS_NUM_BEACONS <= 5,
	     "CS_NUM_BEACONS doit etre entre 1 et 5.");
BUILD_ASSERT(CONFIG_BT_MAX_CONN >= CS_NUM_BEACONS,
	     "CONFIG_BT_MAX_CONN < CS_NUM_BEACONS (prj.conf).");
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CS_COUNT >= CS_NUM_BEACONS,
	     "CONFIG_BT_CTLR_SDC_CS_COUNT < CS_NUM_BEACONS : procedures CS "
	     "concurrentes insuffisantes pour le pipeline (prj.conf).");
#if CS_MAX_BEACONS > 1
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT >=
	     CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT +
	     CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT + 3000,
	     "SPACING trop petit pour ACL + evenement CS + garde 3 ms : aborts 0x3 "
	     "structurels garantis (echec mesure du 20 ms/10000).");
BUILD_ASSERT(CS_CONN_INTERVAL_UNITS * 1250 >=
	     CS_MAX_BEACONS * CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT,
	     "Intervalle de connexion < N_liens x SPACING : le quadrillage ne rentre pas "
	     "(Procedure Enable 0x0C ou startup differe).");
#endif

struct bt_le_cs_set_default_settings_param cs_settings = {
    .enable_initiator_role     = true,
    .enable_reflector_role     = false,
    .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_ONE,
    .max_tx_power              = CS_MAX_TX_POWER, /* profil : max en portée,
                                                   * réduit en docking (anti-
                                                   * saturation < qq mètres) */
};

struct bt_le_cs_create_config_params cs_create_config = {
    .id                     = 0,
    .mode                   = CS_MODE, /* profil : PBR + RTT (drone, levée d'ambiguïté au-delà de la portée non-ambiguë) ou PBR pur (précision courte portée) */
    .min_main_mode_steps    = 2,
    .max_main_mode_steps    = CS_MAIN_STEPS_MAX, /* + de steps/subevent = canaux mieux tassés (profils précision) vs subevents courts (profils drone) */
    .main_mode_repetition   = 1,
    .mode_0_steps           = CS_MODE0_STEPS,    /* 3 en précision : meilleure estim. d'offset fréquentiel */
    .role                   = BT_CONN_LE_CS_ROLE_INITIATOR,
    .rtt_type               = CS_RTT_TYPE, /* sounding 32b (drone) : datation fine + NADM ; AA_ONLY (PBR pur) */
    .cs_sync_phy            = CS_SYNC_PHY, /* 2M_2BT n'affine que les CS_SYNC (RTT), pas la phase PBR */
    .channel_map_repetition = CS_CHMAP_REPETITION, /* 2 en docking N<=2 : chaque canal sondé 2x (cible lente et proche, note ODS) */
    .channel_selection_type = CS_CHSEL_TYPE, /* 3C+X en drone rapide : décorrélation temps-fréquence (note ODS) */
    .ch3c_shape             = CS_CH3C_SHAPE,
    .ch3c_jump              = 2,
    .cs_enhancements_1      = 0,
};

struct bt_le_cs_set_procedure_parameters_param cs_proc_params = {
    .config_id              = 0, /* mis à jour au runtime dans do_set_procedure_params() */
    .max_procedure_len      = CS_PROC_LEN_UNITS, /* ×0,625 ms. Fenêtre courte = placement facile (drone) ; large pour ~72 canaux (précision) */
    .min_procedure_interval = 4,
    .max_procedure_interval = 4,
    .max_procedure_count    = 1,   /* one-shot : s'arrête seule (pas de disable → pas de 0x0c) */
    .min_subevent_len       = CS_SUBEVENT_LEN_US,
    .max_subevent_len       = CS_SUBEVENT_LEN_US, /* ⚠ couplé à CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT (BUILD_ASSERT ci-dessus) */
    .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
    .phy                    = BT_LE_CS_PROCEDURE_PHY_2M,
    .tx_power_delta         = 0x80,
    .preferred_peer_antenna = 1,
    .snr_control_initiator  = BT_LE_CS_SNR_CONTROL_NOT_USED,
    .snr_control_reflector  = BT_LE_CS_SNR_CONTROL_NOT_USED,
};

struct bt_le_conn_param cs_cp_active = {
    /* Intervalle FIXE (min == max). Startup CS ≈ ~11 événements × intervalle
     * (LL_CS_REQ/RSP/IND + marge SDC : le NOMBRE d'événements n'est pas
     * configurable, seule leur DURÉE l'est). En multi : intervalle =
     * CS_MAX_BEACONS × SPACING (quadrillage déterministe, cf. BUILD_ASSERT).
     * En single : plancher = ACL + événement CS + marge, le SPACING ne
     * s'applique pas (une seule connexion centrale).
     * VALIDÉ multi-drone 24/15000 : startup 340 ms, 0 abort.
     * ÉCHEC 16/10000 : garde 1250 µs trop juste → abort 0x3 structurel. */
    .interval_min = CS_CONN_INTERVAL_UNITS,
    .interval_max = CS_CONN_INTERVAL_UNITS,
    .latency      = 0,
    .timeout      = 3200,
};
