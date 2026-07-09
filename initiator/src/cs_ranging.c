#include "cs_ranging.h"
#include "distance_estimation.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include "if.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>

#include <zephyr/logging/log.h>

#include "cs_config.h"

LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#define CS_CONFIG_ID	   0
#define NUM_MODE_0_STEPS   3

/* Éclaircissement du channel map : ne garder qu'un canal CS valide sur N.
 * Réduit l'occupation radio par procédure (~50 ms / N), donc la probabilité
 * de collision avec les ancres ACL des autres liens (abort 0x3), et accélère
 * chaque mesure. Contrepartie pour le pipeline IQ : pas de fréquence élargi
 * (N=2 → ~36 canaux espacés de ~4 MHz ; ambiguïté de distance c/(2·Δf)
 * ≈ 37,5 m, sans conséquence en intérieur). La spec impose >= 15 canaux :
 * N <= 4. Mettre 1 pour désactiver (tous les canaux). */
/* CS_CHANNEL_THINNING et CS_MAX_STEPS_PER_PROC viennent du profil
 * (cs_config.h). Trop petit => "Step data buffer overflow" => abort. */

#define LOCAL_PROCEDURE_MEM                                                                        \
	((CS_MAX_STEPS_PER_PROC * sizeof(struct bt_le_cs_subevent_step)) +                        \
	 (CS_MAX_STEPS_PER_PROC * BT_RAS_MAX_STEP_DATA_LEN))

/* Buffer peer (RAS) partagé : le fetch RAS est fait séquentiellement en phase
 * de collecte, un beacon à la fois. */
#if !CS_RAS_REALTIME
/* Buffer pair PARTAGÉ du mode on-demand (fetch séquentiel). En temps réel il
 * disparaît : chaque beacon a ses peer_rt/peer_snap (~1 Ko), net gain RAM. */
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
#endif

/* ── PIPELINE : état de mesure PAR BEACON ────────────────────────────────────
 * On lance les procédures CS des N beacons puis on collecte, pour recouvrir les
 * ~700 ms de startup au lieu de les additionner. Chaque beacon a son buffer
 * local + ses sémaphores : les callbacks (routés par conn) ne se marchent pas
 * dessus. */
struct cs_meas {
	struct k_sem          sem_done;      /* procédure terminée (COMPLETE/ABORTED) */
	struct k_sem          sem_rd_ready;  /* données RAS du pair prêtes */
	struct net_buf_simple local;         /* steps locaux accumulés (procédure EN VOL) */
	uint8_t               local_buf[LOCAL_PROCEDURE_MEM];
	uint16_t              local_rc;
	uint16_t              peer_rc;
	bool                  aborted;       /* procédure en vol avortée */
	bool                  fetch_failed;  /* échec du Get Ranging Data en cours */
	bool                  active;        /* procédure en vol pour ce beacon */
	/* Instrumentation (k_uptime, ms) : décomposition du temps de mesure */
	int64_t               t_enable;      /* bt_le_cs_procedure_enable accepté */
	int64_t               t_first_se;    /* 1er subevent reçu (0 si aucun) */
	int64_t               t_done;        /* procédure COMPLETE/ABORTED */
	/* Découpage par subevent du buffer local : offset/longueur de chaque
	 * tranche + horodatage k_uptime de son arrivée (callback HCI). Sert au
	 * dump IQ : les steps d'un subevent partagent son timestamp, l'instant
	 * fin par step se reconstruit ensuite par les durées de steps
	 * (Vol 6 Part B §4.5.18). */
#define CS_SE_META_MAX 8
/* Buffer de réception des ranging data du pair (mode temps réel) :
 * dimensionné sur le profil (headers RAS + ~11 o/step mode-2). */
#define PEER_PROCEDURE_MEM (32 + CS_MAX_STEPS_PER_PROC * 12)
	struct { uint16_t off; uint16_t len; int64_t t_ms; } se_meta[CS_SE_META_MAX];
	uint8_t               se_count;
	/* SNAPSHOT de la dernière procédure TERMINÉE : permet de ré-armer la
	 * procédure N+1 (qui écrit dans `local`) AVANT le fetch RAS de N.
	 * Le startup de N+1 est ainsi recouvert par le fetch de N lui-même,
	 * en plus de ceux des autres beacons. */
	struct net_buf_simple snap;
	uint8_t               snap_buf[LOCAL_PROCEDURE_MEM];
	uint16_t              snap_rc;
	int64_t               snap_startup_ms;
	int64_t               snap_meas_ms;
	int64_t               snap_t_first;   /* k_uptime du 1er subevent */
	int64_t               snap_t_done;    /* k_uptime de la fin de procédure */
#if CS_RAS_REALTIME
	/* Temps réel : peer_rt est le buffer SOUSCRIT (auto-reset par la lib
	 * après chaque callback → copie obligatoire dans peer_snap, consommé
	 * par cs_fetch_beacon). */
	struct net_buf_simple peer_rt;
	uint8_t               peer_rt_buf[PEER_PROCEDURE_MEM];
	struct net_buf_simple peer_snap;
	uint8_t               peer_snap_buf[PEER_PROCEDURE_MEM];
	uint16_t              peer_snap_rc;
#endif
	struct { uint16_t off; uint16_t len; int64_t t_ms; } snap_se_meta[CS_SE_META_MAX];
	uint8_t               snap_se_count;
};

/* Dump IQ sur l'UART data (commandes UART "IQON"/"IQOFF", cf. scheduler). */
bool cs_iq_dump = false;
static struct cs_meas meas[MAX_BEACONS];

/* Connexion concernée par les callbacks en cours (setup OU mesure) */
static struct bt_conn *volatile cs_active_conn;

static K_SEM_DEFINE(sem_mtu_exchange,       0, 1);
static K_SEM_DEFINE(sem_discovery,          0, 1);
static K_SEM_DEFINE(sem_caps,               0, 1);
static K_SEM_DEFINE(sem_config_created,     0, 1);
static K_SEM_DEFINE(sem_cs_security,        0, 1);
static K_SEM_DEFINE(sem_procedure_disabled, 0, 1);
#if !CS_RAS_REALTIME
static K_SEM_DEFINE(sem_rd_complete,        0, 1);
#endif

static volatile bool discovery_ok;
static uint8_t  latest_n_ap;
static uint8_t  created_config_id;

/* Index du beacon associé à conn (lockless : les conn sont stables pendant une
 * mesure). -1 si non trouvé. Utilisé pour router les callbacks CS/RAS. */
static int meas_idx(struct bt_conn *conn)
{
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].conn == conn) {
			return i;
		}
	}
	return -1;
}

/* ── Échange MTU ────────────────────────────────────────────────────────────
 * Sans bt_gatt_exchange_mtu, l'ATT MTU reste à 23 octets : le transfert RAS
 * (plusieurs Ko) devient très lent voire dépasse les timeouts. */

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t att_err,
			    struct bt_gatt_exchange_params *params)
{
	if (att_err) {
		LOG_WRN("MTU exchange failed (ATT err %u)", att_err);
	} else {
		LOG_INF("MTU exchanged: %u", bt_gatt_get_mtu(conn));
	}
	k_sem_give(&sem_mtu_exchange);
}

/* ── Découverte GATT du Ranging Service ─────────────────────────────────── */

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
	int err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);

	discovery_ok = (err == 0);
	if (err) {
		LOG_ERR("RAS RREQ alloc failed (err %d)", err);
	}
	bt_gatt_dm_data_release(dm);
	k_sem_give(&sem_discovery);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_ERR("Ranging Service not found on reflector");
	discovery_ok = false;
	k_sem_give(&sem_discovery);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("GATT discovery error (err %d)", err);
	discovery_ok = false;
	k_sem_give(&sem_discovery);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found       = discovery_error_found_cb,
};

/* ── Callbacks Channel Sounding (bt_conn) ───────────────────────────────── */

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	if (conn != cs_active_conn) {
		return;
	}
	k_sem_give(&sem_caps);
}

static void config_created_cb(struct bt_conn *conn, uint8_t status,
			      struct bt_conn_le_cs_config *config)
{
	if (conn != cs_active_conn) {
		return;
	}
	if (status || !config) {
		LOG_ERR("CS config creation failed (status %u)", status);
		return;
	}
	created_config_id = config->id;
	LOG_INF("CS config %u created (mode %u)", config->id, config->mode);
	k_sem_give(&sem_config_created);
}

static void cs_security_enabled_cb(struct bt_conn *conn, uint8_t status)
{
	if (conn != cs_active_conn) {
		return;
	}
	k_sem_give(&sem_cs_security);
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (conn != cs_active_conn) {
		return;
	}
	if (params->state == BT_CONN_LE_CS_PROCEDURES_DISABLED) {
		k_sem_give(&sem_procedure_disabled);
	}
}

static void subevent_result_cb(struct bt_conn *conn,
			       struct bt_conn_le_cs_subevent_result *result)
{
	int i = meas_idx(conn);
	if (i < 0 || !meas[i].active) {
		return;
	}
	struct cs_meas *m = &meas[i];

	if (m->t_first_se == 0) {
		m->t_first_se = k_uptime_get();
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* Subevent avorté (typ. 0x3 : conflit avec l'ACL d'un autre lien). */
		LOG_WRN("CS subevent aborted b%d (subevent 0x%X, procedure 0x%X)", i,
			result->header.subevent_abort_reason,
			result->header.procedure_abort_reason);
		m->aborted = true;
		net_buf_simple_reset(&m->local);
	} else if (!m->aborted && result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&m->local)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			if (m->se_count < CS_SE_META_MAX) {
				m->se_meta[m->se_count].off  = m->local.len;
				m->se_meta[m->se_count].len  = len;
				m->se_meta[m->se_count].t_ms = k_uptime_get();
				m->se_count++;
			}
			net_buf_simple_add_mem(&m->local, step_data, len);
		} else {
			LOG_ERR("Step data buffer overflow b%d", i);
			m->aborted = true;
		}
	}

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		m->t_done   = k_uptime_get();
		m->local_rc = bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
		k_sem_give(&m->sem_done);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		m->t_done   = k_uptime_get();
		m->aborted  = true;
		k_sem_give(&m->sem_done);
	}
}

BT_CONN_CB_DEFINE(cs_conn_cb) = {
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete                   = config_created_cb,
	.le_cs_security_enable_complete          = cs_security_enabled_cb,
	.le_cs_procedure_enable_complete         = procedure_enable_cb,
	.le_cs_subevent_data_available           = subevent_result_cb,
};

/* ── Callbacks RAS (Ranging Service client) ─────────────────────────────── */

#if CS_RAS_REALTIME
/* Notification temps réel complète (thread BT RX). Le buffer souscrit
 * (peer_rt) est reset par la lib au retour : on copie dans peer_snap. */
static void rt_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	int i = meas_idx(conn);

	if (i < 0) {
		return;
	}
	if (err) {
		LOG_WRN("RT ranging data b%d failed (err %d)", i, err);
		return;
	}
	if (meas[i].peer_rt.len > PEER_PROCEDURE_MEM) {
		LOG_ERR("RT data b%d trop grande (%u)", i, meas[i].peer_rt.len);
		return;
	}
	net_buf_simple_reset(&meas[i].peer_snap);
	net_buf_simple_add_mem(&meas[i].peer_snap, meas[i].peer_rt.data,
			       meas[i].peer_rt.len);
	meas[i].peer_snap_rc = ranging_counter;
	meas[i].peer_rc      = ranging_counter;
	k_sem_give(&meas[i].sem_rd_ready);
}
#endif /* CS_RAS_REALTIME */

#if !CS_RAS_REALTIME
static void rd_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	int i = meas_idx(conn);
	if (i < 0) {
		return;
	}
	meas[i].peer_rc = ranging_counter;
	k_sem_give(&meas[i].sem_rd_ready);
}

static void rd_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_WRN("Peer ranging data overwritten (counter %u)", ranging_counter);
}

static void rd_get_complete_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	if (conn != cs_active_conn) {
		return;
	}
	if (err) {
		LOG_ERR("Ranging data get failed (err %d)", err);
		int i = meas_idx(conn);
		if (i >= 0) {
			/* Flag DÉDIÉ : `aborted` appartient à la procédure EN
			 * VOL (N+1), qui peut être déjà repartie pendant ce
			 * fetch de N — ne pas la polluer. */
			meas[i].fetch_failed = true;
		}
	}
	k_sem_give(&sem_rd_complete);
}
#endif /* !CS_RAS_REALTIME */

/* ── Channel map ────────────────────────────────────────────────────────── */

static void cs_build_channel_map(uint8_t channel_map[10])
{
	bt_le_cs_set_valid_chmap_bits(channel_map);

#if CS_CHANNEL_THINNING > 1
	/* Ne garder qu'un canal valide sur CS_CHANNEL_THINNING, en parcourant
	 * les canaux dans l'ordre des fréquences. */
	int kept = 0;
	int count = 0;

	for (int ch = 0; ch < 79; ch++) {
		if (!(channel_map[ch / 8] & BIT(ch % 8))) {
			continue;
		}
		if (count % CS_CHANNEL_THINNING != 0) {
			channel_map[ch / 8] &= ~BIT(ch % 8);
		} else {
			kept++;
		}
		count++;
	}
	LOG_INF("CS channel map thinned: %d/%d channels kept", kept, count);
#endif
}

/* ── Setup par beacon ───────────────────────────────────────────────────── */

int cs_setup_beacon(struct beacon_state *beacon)
{
	int err;
	struct bt_conn *conn = beacon->conn;

	cs_active_conn = conn;
	discovery_ok = false;
	k_sem_reset(&sem_mtu_exchange);
	k_sem_reset(&sem_discovery);
	k_sem_reset(&sem_caps);
	k_sem_reset(&sem_config_created);
	k_sem_reset(&sem_cs_security);

	/* 0. Échange MTU. Struct static : doit survivre jusqu'au callback.
	 * OK car le setup est strictement séquentiel. */
	static struct bt_gatt_exchange_params mtu_params;

	mtu_params.func = mtu_exchange_cb;
	err = bt_gatt_exchange_mtu(conn, &mtu_params);
	if (err) {
		LOG_ERR("MTU exchange start failed (err %d)", err);
		cs_active_conn = NULL;
		return err;
	}
	if (k_sem_take(&sem_mtu_exchange, K_SECONDS(10))) {
		cs_active_conn = NULL;
		return -ETIMEDOUT;
	}

	/* 1. Découverte GATT du Ranging Service + allocation client RREQ */
	err = bt_gatt_dm_start(conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("GATT discovery start failed (err %d)", err);
		cs_active_conn = NULL;
		return err;
	}
	if (k_sem_take(&sem_discovery, K_SECONDS(10)) || !discovery_ok) {
		cs_active_conn = NULL;
		return -ETIMEDOUT;
	}

	/* 2. Capacités CS distantes */
	err = bt_le_cs_read_remote_supported_capabilities(conn);
	if (err) {
		LOG_ERR("Read remote CS capabilities failed (err %d)", err);
		goto cleanup;
	}
	if (k_sem_take(&sem_caps, K_SECONDS(10))) {
		err = -ETIMEDOUT;
		goto cleanup;
	}

	/* 3. Default settings : rôle initiateur */
	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role     = cs_settings.enable_initiator_role,
		.enable_reflector_role     = cs_settings.enable_reflector_role,
		.cs_sync_antenna_selection = cs_settings.cs_sync_antenna_selection,
		.max_tx_power              = cs_settings.max_tx_power,
	};

	err = bt_le_cs_set_default_settings(conn, &default_settings);
	if (err) {
		LOG_ERR("Set default CS settings failed (err %d)", err);
		goto cleanup;
	}

	/* 4. Souscriptions RAS — deux modes EXCLUSIFS (le serveur rejette la
	 * double souscription) : temps réel (notifications poussées à la fin
	 * de chaque procédure, dans le buffer peer_rt du beacon) ou on-demand
	 * (rd_ready + Get Ranging Data sur le RAS Control Point, dont la
	 * souscription CP est alors OBLIGATOIRE — réponses en indications). */
#if CS_RAS_REALTIME
	{
		int idx = meas_idx(conn);

		if (idx < 0) {
			LOG_ERR("RT subscribe: beacon inconnu");
			goto cleanup;
		}
		err = bt_ras_rreq_realtime_rd_subscribe(conn, &meas[idx].peer_rt,
							rt_data_cb);
		if (err) {
			LOG_ERR("Realtime RD subscribe failed (err %d)", err);
			goto cleanup;
		}
	}
#else
	err = bt_ras_rreq_rd_overwritten_subscribe(conn, rd_overwritten_cb);
	if (err) {
		LOG_ERR("RD overwritten subscribe failed (err %d)", err);
		goto cleanup;
	}
	err = bt_ras_rreq_rd_ready_subscribe(conn, rd_ready_cb);
	if (err) {
		LOG_ERR("RD ready subscribe failed (err %d)", err);
		goto cleanup;
	}
	err = bt_ras_rreq_on_demand_rd_subscribe(conn);
	if (err) {
		LOG_ERR("On-demand RD subscribe failed (err %d)", err);
		goto cleanup;
	}
	err = bt_ras_rreq_cp_subscribe(conn);
	if (err) {
		LOG_ERR("RAS-CP subscribe failed (err %d)", err);
		goto cleanup;
	}
#endif

	/* 5. Config CS : mode 2 (PBR) + sub-mode 1, base du sample validé,
	 * chmap repetition 1 et channel map éventuellement éclairci pour
	 * réduire l'airtime par procédure (voir CS_CHANNEL_THINNING). */
	struct bt_le_cs_create_config_params config_params = {
		.id                     = CS_CONFIG_ID,
		.mode                   = cs_create_config.mode,
		.min_main_mode_steps    = cs_create_config.min_main_mode_steps,
		.max_main_mode_steps    = cs_create_config.max_main_mode_steps,
		.main_mode_repetition   = cs_create_config.main_mode_repetition,
		.mode_0_steps           = cs_create_config.mode_0_steps,
		.role                   = cs_create_config.role,
		.rtt_type               = cs_create_config.rtt_type,
		.cs_sync_phy            = cs_create_config.cs_sync_phy,
		.channel_map_repetition = cs_create_config.channel_map_repetition,
		.channel_selection_type = cs_create_config.channel_selection_type,
		.ch3c_shape             = cs_create_config.ch3c_shape,
		.ch3c_jump              = cs_create_config.ch3c_jump,
	};
	cs_build_channel_map(config_params.channel_map);

	err = bt_le_cs_create_config(conn, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Create CS config failed (err %d)", err);
		goto cleanup;
	}
	if (k_sem_take(&sem_config_created, K_SECONDS(10))) {
		err = -ETIMEDOUT;
		goto cleanup;
	}
	beacon->cs_config_id = created_config_id;

	/* 6. Sécurité CS (clés de ranging) */
	err = bt_le_cs_security_enable(conn);
	if (err) {
		LOG_ERR("CS security enable failed (err %d)", err);
		goto cleanup;
	}
	if (k_sem_take(&sem_cs_security, K_SECONDS(10))) {
		err = -ETIMEDOUT;
		goto cleanup;
	}

	/* 7. Paramètres de procédure. max_procedure_count DOIT valoir 0
	 * (le SDC rejette toute autre valeur, HCI 0x12) : 0 = procédures
	 * répétées jusqu'à disable explicite, d'où le cs_procedure_disable
	 * du round-robin. max_subevent_len doit rester <=
	 * CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT des deux côtés. */
	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id                     = beacon->cs_config_id,
		.max_procedure_len             = cs_proc_params.max_procedure_len,
		.min_procedure_interval        = cs_proc_params.min_procedure_interval,
		.max_procedure_interval        = cs_proc_params.max_procedure_interval,
		.max_procedure_count           = cs_proc_params.max_procedure_count,
		.min_subevent_len              = cs_proc_params.min_subevent_len,
		.max_subevent_len              = cs_proc_params.max_subevent_len,
		.tone_antenna_config_selection = cs_proc_params.tone_antenna_config_selection,
		.phy                           = cs_proc_params.phy,
		.tx_power_delta                = cs_proc_params.tx_power_delta,
		.preferred_peer_antenna        = cs_proc_params.preferred_peer_antenna,
		.snr_control_initiator         = cs_proc_params.snr_control_initiator,
		.snr_control_reflector         = cs_proc_params.snr_control_reflector,
	};

	err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
	if (err) {
		LOG_ERR("Set procedure parameters failed (err %d)", err);
		goto cleanup;
	}

	beacon->cs_ready = true;
	cs_active_conn = NULL;
	LOG_INF("CS setup complete for beacon");
	return 0;

cleanup:
	cs_active_conn = NULL;
	bt_ras_rreq_free(conn);
	return err;
}

/* ── Mesure one-shot (round-robin) ──────────────────────────────────────── */

static void cs_procedure_disable(struct bt_conn *conn, uint8_t config_id)
{
	struct bt_le_cs_procedure_enable_param disable_params = {
		.config_id = config_id,
		.enable    = 0,
	};

	k_sem_reset(&sem_procedure_disabled);

	int err = bt_le_cs_procedure_enable(conn, &disable_params);

	if (err) {
		if (err != -EALREADY) {
			LOG_WRN("Procedure disable failed (err %d)", err);
		}
		return;
	}

	/* Le disable est ASYNCHRONE : attendre la confirmation
	 * (state = DISABLED) avant tout ré-enable, sinon le contrôleur
	 * refuse avec HCI 0x0C Command Disallowed. */
	if (k_sem_take(&sem_procedure_disabled, K_SECONDS(2))) {
		LOG_WRN("Timeout waiting for CS procedure disable");
	}
}

/* Init des buffers/sémaphores par beacon. À appeler une fois avant la boucle. */
/* ── Rétrogradation automatique des options CS OPTIONNELLES ─────────────────
 * Create Config renvoie 0x11 (Unsupported Feature or Parameter Value) si le
 * contrôleur ne supporte pas une capacité optionnelle demandée : CS_SYNC
 * 2M_2BT, CSA #3C, RTT sounding. On lit les caps LOCALES au boot, on logue,
 * et on replie chaque option non supportée sur sa valeur de base — le log
 * WRN désigne le coupable. Les deux extrémités étant des nRF54L15/SDC
 * identiques, la vérification locale couvre le banc (en hétérogène, lire
 * aussi les caps DISTANTES avant le setup). */
static void cs_apply_local_caps(void)
{
	struct bt_conn_le_cs_capabilities caps;

	if (bt_le_cs_read_local_supported_capabilities(&caps)) {
		LOG_WRN("Caps CS locales illisibles, options conservées telles quelles");
		return;
	}
	LOG_INF("CS caps: 2M_2BT=%d CSA3C=%d RTTsound_n=%u RTTaa_n=%u NADMsound=%d",
		caps.cs_sync_2m_2bt_phy_supported, caps.chsel_alg_3c_supported,
		caps.rtt_sounding_n, caps.rtt_aa_only_n,
		caps.phase_based_nadm_sounding_supported);

	if (cs_create_config.cs_sync_phy == BT_CONN_LE_CS_SYNC_2M_2BT_PHY &&
	    !caps.cs_sync_2m_2bt_phy_supported) {
		LOG_WRN("caps: CS_SYNC 2M_2BT non supporte -> repli 2M");
		cs_create_config.cs_sync_phy = BT_CONN_LE_CS_SYNC_2M_PHY;
	}
	if (cs_create_config.channel_selection_type == BT_CONN_LE_CS_CHSEL_TYPE_3C &&
	    !caps.chsel_alg_3c_supported) {
		LOG_WRN("caps: CSA #3C non supporte -> repli #3B");
		cs_create_config.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	}
	if (cs_create_config.rtt_type == BT_CONN_LE_CS_RTT_TYPE_32_BIT_SOUNDING &&
	    caps.rtt_sounding_n == 0) {
		LOG_WRN("caps: RTT sounding non supporte -> repli AA_ONLY");
		cs_create_config.rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	}
}

void cs_ranging_init(void)
{
	cs_apply_local_caps();

	for (int i = 0; i < MAX_BEACONS; i++) {
		k_sem_init(&meas[i].sem_done, 0, 1);
		k_sem_init(&meas[i].sem_rd_ready, 0, 1);
		net_buf_simple_init_with_data(&meas[i].local, meas[i].local_buf,
					      LOCAL_PROCEDURE_MEM);
		net_buf_simple_reset(&meas[i].local);
		net_buf_simple_init_with_data(&meas[i].snap, meas[i].snap_buf,
					      LOCAL_PROCEDURE_MEM);
		net_buf_simple_reset(&meas[i].snap);
#if CS_RAS_REALTIME
		net_buf_simple_init_with_data(&meas[i].peer_rt,
					      meas[i].peer_rt_buf,
					      PEER_PROCEDURE_MEM);
		net_buf_simple_reset(&meas[i].peer_rt);
		net_buf_simple_init_with_data(&meas[i].peer_snap,
					      meas[i].peer_snap_buf,
					      PEER_PROCEDURE_MEM);
		net_buf_simple_reset(&meas[i].peer_snap);
#endif
	}
}

/* ── PHASE 1 : lance la procédure CS du beacon i SANS attendre. ──────────────
 * Retourne 0 si l'enable est accepté (le SDC autorise le CS concurrent). */
int cs_enable_beacon(int i)
{
	struct bt_conn *conn = beacons[i].conn;

	if (!beacons[i].cs_ready || conn == NULL) {
		return -1;
	}

	meas[i].aborted    = false;
	meas[i].active     = true;
	meas[i].t_first_se = 0;
	meas[i].t_done     = 0;
	meas[i].se_count   = 0;
	net_buf_simple_reset(&meas[i].local);
	k_sem_reset(&meas[i].sem_done);
	/* PAS de reset de sem_rd_ready ici : le rd_ready de la procédure N
	 * (pas encore fetchée) peut arriver juste avant/après cet enable de
	 * N+1 — le reset le perdrait et cs_fetch_beacon timeouterait. */

	struct bt_le_cs_procedure_enable_param p = {
		.config_id = beacons[i].cs_config_id,
		.enable    = 1,
	};
	int err = bt_le_cs_procedure_enable(conn, &p);
	if (err) {
		LOG_ERR("Procedure enable b%d failed (err %d)", i, err);
		meas[i].active = false;
	} else {
		meas[i].t_enable = k_uptime_get();
	}
	return err;
}

/* ── Dump IQ vers l'UART data (lignes ASCII, format doc IQ_DUMP.md) ─────────
 * IQL,<beacon>,<counter>,<t_ms_subevent>,<hex steps du subevent>  (× subevents)
 * IQP,<beacon>,<counter>,<t_ms_done>,<hex ranging data RAS du pair>
 * Émis depuis le thread de mesure APRÈS le fetch : n'ajoute que le temps
 * d'écriture UART au cycle (à 115200 bauds ~30-60 ms/procédure → passer
 * l'uart21 à 1 Mbaud en devicetree si le dump est utilisé en continu). */
static void iq_emit_hex(const uint8_t *data, uint16_t len)
{
	static const char hexc[] = "0123456789ABCDEF";
	char chunk[64];
	int  ci = 0;

	for (uint16_t k = 0; k < len; k++) {
		chunk[ci++] = hexc[data[k] >> 4];
		chunk[ci++] = hexc[data[k] & 0x0F];
		if (ci == sizeof(chunk)) {
			if_send((uint8_t *)chunk, ci);
			ci = 0;
		}
	}
	if (ci) {
		if_send((uint8_t *)chunk, ci);
	}
}

static void cs_iq_emit(int i, struct net_buf_simple *peer)
{
	char hdr[48];
	int  n;

	if (meas[i].snap_se_count > 0) {
		for (uint8_t se = 0; se < meas[i].snap_se_count; se++) {
			n = snprintf(hdr, sizeof(hdr), "IQL,%d,%u,%lld,", i,
				     meas[i].snap_rc, meas[i].snap_se_meta[se].t_ms);
			if_send((uint8_t *)hdr, n);
			iq_emit_hex(meas[i].snap.data + meas[i].snap_se_meta[se].off,
				    meas[i].snap_se_meta[se].len);
			if_send((uint8_t *)"\n", 1);
		}
	} else {
		/* Filet : pas de découpage par subevent disponible — émettre
		 * la procédure entière, horodatée au 1er subevent. */
		n = snprintf(hdr, sizeof(hdr), "IQL,%d,%u,%lld,", i,
			     meas[i].snap_rc, meas[i].snap_t_first);
		if_send((uint8_t *)hdr, n);
		iq_emit_hex(meas[i].snap.data, meas[i].snap.len);
		if_send((uint8_t *)"\n", 1);
	}
	/* snap_t_done : pas meas[i].t_done, remis à zéro par l'enable de N+1
	 * qui précède ce fetch dans le pipeline. */
	n = snprintf(hdr, sizeof(hdr), "IQP,%d,%u,%lld,", i, meas[i].snap_rc,
		     meas[i].snap_t_done);
	if_send((uint8_t *)hdr, n);
	iq_emit_hex(peer->data, peer->len);
	if_send((uint8_t *)"\n", 1);
}

/* ── PHASE 2a : attend la fin de la procédure du beacon i et SNAPSHOTTE ses
 * steps locaux + compteur + timings. Après retour 0, l'appelant peut ré-armer
 * immédiatement (cs_enable_beacon) : la procédure N+1 écrira dans `local`
 * sans toucher au snapshot de N. */
int cs_wait_done_beacon(int i)
{
	struct bt_conn *conn = beacons[i].conn;

	if (!meas[i].active || conn == NULL) {
		return -EINVAL;
	}

	int err = k_sem_take(&meas[i].sem_done, K_SECONDS(5));

	if (cs_proc_params.max_procedure_count == 0) {
		/* cs_active_conn requis par procedure_enable_cb (disable) */
		cs_active_conn = conn;
		cs_procedure_disable(conn, beacons[i].cs_config_id);
		cs_active_conn = NULL;
	}
	meas[i].active = false;

	if (err || meas[i].aborted || meas[i].local.len == 0) {
		LOG_WRN("Beacon[%d] procedure failed", i);
		return err ? err : -EIO;
	}

	net_buf_simple_reset(&meas[i].snap);
	net_buf_simple_add_mem(&meas[i].snap, meas[i].local.data, meas[i].local.len);
	memcpy(meas[i].snap_se_meta, meas[i].se_meta, sizeof(meas[i].se_meta));
	meas[i].snap_se_count   = meas[i].se_count;
	meas[i].snap_rc         = meas[i].local_rc;
	meas[i].snap_t_first    = meas[i].t_first_se;
	meas[i].snap_t_done     = meas[i].t_done;
	if (meas[i].se_count == 0) {
		/* Diagnostic : données locales présentes mais aucun subevent
		 * horodaté — divergence de code ou chemin inattendu. */
		LOG_WRN("b%d: local %u octets mais se_count=0", i, meas[i].local.len);
	}
	meas[i].snap_startup_ms = meas[i].t_first_se ?
				  (meas[i].t_first_se - meas[i].t_enable) : -1;
	meas[i].snap_meas_ms    = meas[i].t_first_se ?
				  (meas[i].t_done - meas[i].t_first_se) : -1;
	return 0;
}

/* ── PHASE 2b : récupère les données RAS du pair pour le SNAPSHOT du beacon i
 * (fetch séquentiel : buffer peer partagé) et calcule la distance. La
 * procédure N+1 peut déjà être en vol : elle n'interfère pas (buffers et
 * flags découplés). */
float cs_fetch_beacon(int i)
{
	struct bt_conn *conn = beacons[i].conn;

	if (conn == NULL || meas[i].snap.len == 0) {
		return -1.0f;
	}

	if (k_sem_take(&meas[i].sem_rd_ready, K_SECONDS(5))) {
		LOG_WRN("Beacon[%d] rd_ready timeout", i);
		return -1.0f;
	}

	int64_t t_fetch_start = k_uptime_get();

#if CS_RAS_REALTIME
	/* Temps réel : la donnée pair a été POUSSÉE par le réflecteur dès la
	 * fin de procédure (rt_data_cb → peer_snap). Zéro aller-retour GATT :
	 * le "fetch" se réduit à une vérification de compteur. */
	if (meas[i].peer_snap_rc != meas[i].snap_rc) {
		/* Contrairement à l'on-demand, PAS de rétention : si N+1 a
		 * déjà écrasé peer_snap, la moitié pair de N est perdue —
		 * mesure sacrifiée plutôt que mélanger deux procédures. */
		LOG_WRN("Beacon[%d] RT counter skew (snap %u peer %u), mesure perdue",
			i, meas[i].snap_rc, meas[i].peer_snap_rc);
		return -1.0f;
	}
	struct net_buf_simple *peer = &meas[i].peer_snap;
#else
	if (meas[i].peer_rc != meas[i].snap_rc) {
		/* Le pair a déjà annoncé un compteur plus récent (N+1 très
		 * rapide) : non bloquant, on fetch explicitement snap_rc —
		 * la rétention réflecteur (RD_BUFFERS_PER_CONN=10) le garde. */
		LOG_WRN("Beacon[%d] counter skew (snap %u peer %u)",
			i, meas[i].snap_rc, meas[i].peer_rc);
	}

	/* cs_active_conn route rd_get_complete_cb (fetch séquentiel). */
	cs_active_conn = conn;
	meas[i].fetch_failed = false;

	net_buf_simple_reset(&latest_peer_steps);
	k_sem_reset(&sem_rd_complete);
	int err = bt_ras_rreq_cp_get_ranging_data(conn, &latest_peer_steps,
						  meas[i].snap_rc, rd_get_complete_cb);
	if (err) {
		LOG_ERR("Beacon[%d] get ranging data failed (err %d)", i, err);
		cs_active_conn = NULL;
		return -1.0f;
	}
	if (k_sem_take(&sem_rd_complete, K_SECONDS(5)) || meas[i].fetch_failed) {
		LOG_WRN("Beacon[%d] ranging data timeout", i);
		cs_active_conn = NULL;
		return -1.0f;
	}
	cs_active_conn = NULL;
	struct net_buf_simple *peer = &latest_peer_steps;
#endif /* CS_RAS_REALTIME */

	if (cs_iq_dump) {
		cs_iq_emit(i, peer);
	}

	/* Décomposition : startup = enable → 1er subevent (latence de
	 * scheduling SDC, ∝ intervalle de connexion), meas = 1er subevent →
	 * done, fetch = attente donnée pair (GATT on-demand, ~0 en RT). */
	LOG_INF("b%d timing: startup=%lld ms meas=%lld ms fetch=%lld ms",
		i, meas[i].snap_startup_ms, meas[i].snap_meas_ms,
		k_uptime_get() - t_fetch_start);

	return estimate_distance(&meas[i].snap, peer,
				 latest_n_ap, BT_CONN_LE_CS_ROLE_INITIATOR, i);
}

/* ── Compat : wait_done + fetch enchaînés (sans recouvrement N+1/fetch). */
float cs_collect_beacon(int i)
{
	if (cs_wait_done_beacon(i) != 0) {
		return -1.0f;
	}
	return cs_fetch_beacon(i);
}
