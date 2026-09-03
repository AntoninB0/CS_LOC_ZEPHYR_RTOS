#include "cs_config.h"

/* ── CONSISTENCY GUARDS prj.conf <-> profile (see cs_config.h) ─────────────
 * Refuse compilation if the SDC timeline reservations do not cover the selected
 * profile — this is exactly the class of errors that produced the historical
 * 700 ms startup and 0x3 aborts. */
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT >= CS_SUBEVENT_LEN_US,
	     "CS_EVENT_LEN_DEFAULT < max_subevent_len: Procedure Params rejected (0x12). "
	     "Adapt the prj.conf overlay to the profile.");
BUILD_ASSERT(CS_NUM_BEACONS >= 1 && CS_NUM_BEACONS <= 5,
	     "CS_NUM_BEACONS must be between 1 and 5.");
BUILD_ASSERT(CONFIG_BT_MAX_CONN >= CS_NUM_BEACONS,
	     "CONFIG_BT_MAX_CONN < CS_NUM_BEACONS (prj.conf).");
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CS_COUNT >= CS_NUM_BEACONS,
	     "CONFIG_BT_CTLR_SDC_CS_COUNT < CS_NUM_BEACONS: not enough concurrent "
	     "CS procedures for the pipeline (prj.conf).");
#if CS_MAX_BEACONS > 1
BUILD_ASSERT(CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT >=
	     CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT +
	     CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT + 3000,
	     "SPACING too small for ACL + CS event + 3 ms guard: structural 0x3 "
	     "aborts guaranteed (20 ms/10000 measurement failure).");
BUILD_ASSERT(CS_CONN_INTERVAL_UNITS * 1250 >=
	     CS_MAX_BEACONS * CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT,
	     "Connection interval < N_links x SPACING: the grid does not fit "
	     "(Procedure Enable 0x0C or deferred startup).");
#endif

struct bt_le_cs_set_default_settings_param cs_settings = {
    .enable_initiator_role     = true,
    .enable_reflector_role     = false,
    .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_ONE,
    .max_tx_power              = CS_MAX_TX_POWER, /* profile: max for range,
                                                   * reduced in docking (anti-
                                                   * saturation < a few meters) */
};

struct bt_le_cs_create_config_params cs_create_config = {
    .id                     = 0,
    .mode                   = CS_MODE, /* profile: PBR + RTT (drone, ambiguity resolution beyond the non-ambiguous range) or pure PBR (short-range precision) */
    .min_main_mode_steps    = 2,
    .max_main_mode_steps    = CS_MAIN_STEPS_MAX, /* more steps/subevent = channels packed tighter (precision profiles) vs short subevents (drone profiles) */
    .main_mode_repetition   = 1,
    .mode_0_steps           = CS_MODE0_STEPS,    /* 3 in precision: better frequency-offset estimation */
    .role                   = BT_CONN_LE_CS_ROLE_INITIATOR,
    .rtt_type               = CS_RTT_TYPE, /* 32-bit sounding (drone): fine timestamping + NADM; AA_ONLY (pure PBR) */
    .cs_sync_phy            = CS_SYNC_PHY, /* 2M_2BT only refines the CS_SYNC (RTT), not the PBR phase */
    .channel_map_repetition = CS_CHMAP_REPETITION, /* 2 in docking N<=2: each channel sounded 2x (slow and near target, ODS note) */
    .channel_selection_type = CS_CHSEL_TYPE, /* 3C+X in fast drone: time-frequency decorrelation (ODS note) */
    .ch3c_shape             = CS_CH3C_SHAPE,
    .ch3c_jump              = 2,
    .cs_enhancements_1      = 0,
};

struct bt_le_cs_set_procedure_parameters_param cs_proc_params = {
    .config_id              = 0, /* updated at runtime in do_set_procedure_params() */
    .max_procedure_len      = CS_PROC_LEN_UNITS, /* x0.625 ms. Short window = easy placement (drone); wide for ~72 channels (precision) */
    /* Per-link sampling period = interval x connection interval (unit:
     * connection events). Drone N=3: 4 x 45 ms = 180 ms/beacon. */
    .min_procedure_interval = 4,
    .max_procedure_interval = 4,
    /* 0 = FREE-RUNNING: the controller repeats the procedures every
     * procedure_interval until disable. The LL handshake (~11 events x
     * interval ≈ 495 ms) is paid ONLY ONCE at arming, never again per
     * measurement — it was what capped the rate in one-shot
     * (max_procedure_count=1: 3-beacon cycle ≈ 1485 ms observed). */
    .max_procedure_count    = 0,
    .min_subevent_len       = CS_SUBEVENT_LEN_US,
    .max_subevent_len       = CS_SUBEVENT_LEN_US, /* ⚠ coupled to CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT (BUILD_ASSERT above) */
    .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
    .phy                    = BT_LE_CS_PROCEDURE_PHY_2M,
    .tx_power_delta         = 0x80,
    .preferred_peer_antenna = 1,
    .snr_control_initiator  = BT_LE_CS_SNR_CONTROL_NOT_USED,
    .snr_control_reflector  = BT_LE_CS_SNR_CONTROL_NOT_USED,
};

struct bt_le_conn_param cs_cp_active = {
    /* FIXED interval (min == max). CS startup ≈ ~11 events x interval
     * (LL_CS_REQ/RSP/IND + SDC margin: the NUMBER of events is not
     * configurable, only their DURATION is). In multi: interval =
     * CS_MAX_BEACONS x SPACING (deterministic grid, cf. BUILD_ASSERT).
     * In single: floor = ACL + CS event + margin, SPACING does not apply
     * (a single central connection).
     * VALIDATED multi-drone 24/15000: startup 340 ms, 0 abort.
     * FAILURE 16/10000: 1250 us guard too tight → structural 0x3 abort. */
    .interval_min = CS_CONN_INTERVAL_UNITS,
    .interval_max = CS_CONN_INTERVAL_UNITS,
    .latency      = 0,
    .timeout      = 3200,
};
