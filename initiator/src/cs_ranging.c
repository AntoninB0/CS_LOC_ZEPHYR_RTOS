#include "cs_ranging.h"

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

/* ── FREE-RUNNING : état de mesure PAR BEACON ────────────────────────────────
 * Les procédures CS sont armées UNE FOIS par beacon (max_procedure_count = 0) :
 * le contrôleur les répète tous les procedure_interval, le handshake LL de
 * ~11 × intervalle (~495 ms) n'est payé qu'à l'armement. En mode RAS temps
 * réel, le snapshot de chaque procédure terminée est pris DANS le callback
 * (thread BT RX) et apparié à la notification pair par ranging counter ; le
 * thread principal ne fait que collecter (dump IQ). Chaque beacon a son
 * buffer local + ses sémaphores : les callbacks (routés par conn) ne se
 * marchent pas dessus. */
struct cs_meas {
	struct k_mutex        lock;          /* snap/peer_snap : callback BT RX vs collecte */
	struct k_sem          sem_done;      /* on-demand : procédure terminée (COMPLETE/ABORTED) */
	struct k_sem          sem_rd_ready;  /* RT : paire locale+pair prête ; on-demand : donnée pair annoncée */
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
	bool                  snap_valid;     /* RT : snapshot cohérent disponible */
	bool                  snap_busy;      /* RT : dump en cours (main) — ne pas écraser */
	int64_t               snap_startup_ms;
	int64_t               snap_meas_ms;
	int64_t               snap_t_first;   /* k_uptime du 1er subevent */
	int64_t               snap_t_done;    /* k_uptime de la fin de procédure */
	int64_t               snap_prev_done; /* t_done du snapshot précédent (période) */
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

/* Dump IQ sur l'UART data (commandes UART "IQON"/"IQOFF", cf. scheduler).
 * ON par défaut : le dump IQL/IQP horodaté EST la sortie de l'initiateur —
 * l'estimation de distance est faite hors carte (Python). */
bool cs_iq_dump = true;
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

/* Copie local → snap + métadonnées/timings. Appelé sous m->lock en RT
 * (callback BT RX), sans lock en on-demand (thread principal exclusif). */
static void cs_snapshot_from_local(struct cs_meas *m)
{
	net_buf_simple_reset(&m->snap);
	net_buf_simple_add_mem(&m->snap, m->local.data, m->local.len);
	memcpy(m->snap_se_meta, m->se_meta, sizeof(m->se_meta));
	m->snap_se_count   = m->se_count;
	m->snap_rc         = m->local_rc;
	m->snap_t_first    = m->t_first_se;
	m->snap_prev_done  = m->snap_t_done;
	m->snap_t_done     = m->t_done;
	m->snap_startup_ms = m->t_first_se ? (m->t_first_se - m->t_enable) : -1;
	m->snap_meas_ms    = m->t_first_se ? (m->t_done - m->t_first_se) : -1;
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
#if CS_RAS_REALTIME
		/* FREE-RUNNING : la procédure suivante démarre toute seule dans
		 * procedure_interval — snapshot immédiat ICI (le thread principal
		 * n'a pas le temps de le faire avant l'arrivée des subevents de
		 * N+1 dans `local`). Le sémaphore d'appariement est donné par
		 * rt_data_cb quand la moitié pair du MÊME compteur arrive. */
		k_mutex_lock(&m->lock, K_FOREVER);
		if (!m->snap_busy && !m->aborted && m->local.len > 0) {
			cs_snapshot_from_local(m);
			m->snap_valid = true;
		} else if (m->snap_busy) {
			LOG_WRN("b%d: dump en cours, mesure %u sautee", i, m->local_rc);
		}
		k_mutex_unlock(&m->lock);
		/* Préparer l'accumulation de la procédure suivante. */
		net_buf_simple_reset(&m->local);
		m->se_count   = 0;
		m->t_first_se = 0;
		m->aborted    = false;
#else
		k_sem_give(&m->sem_done);
#endif
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		m->t_done   = k_uptime_get();
#if CS_RAS_REALTIME
		/* Mesure sacrifiée, la suivante arrive dans procedure_interval. */
		net_buf_simple_reset(&m->local);
		m->se_count   = 0;
		m->t_first_se = 0;
		m->aborted    = false;
#else
		m->aborted  = true;
		k_sem_give(&m->sem_done);
#endif
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
 * (peer_rt) est reset par la lib au retour : on copie dans peer_snap.
 * APPARIEMENT : la paire (snap local, peer_snap) n'est signalée au thread de
 * collecte (sem_rd_ready) que si le compteur pair == compteur du snapshot
 * local — les résultats HCI locaux d'une procédure précèdent toujours la
 * notification GATT du réflecteur (~1 événement de connexion plus tard),
 * donc en nominal chaque procédure s'apparie ; un compteur dépareillé =
 * mesure sacrifiée (collecte plus lente que la cadence, ou abort local). */
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

	struct cs_meas *m = &meas[i];

	k_mutex_lock(&m->lock, K_FOREVER);
	if (m->snap_valid && !m->snap_busy && ranging_counter == m->snap_rc) {
		net_buf_simple_reset(&m->peer_snap);
		net_buf_simple_add_mem(&m->peer_snap, m->peer_rt.data,
				       m->peer_rt.len);
		m->peer_snap_rc = ranging_counter;
		m->peer_rc      = ranging_counter;
		k_mutex_unlock(&m->lock);
		k_sem_give(&m->sem_rd_ready);
		return;
	}
	k_mutex_unlock(&m->lock);
	LOG_WRN("b%d: paire perdue (peer %u vs snap %u%s)", i, ranging_counter,
		m->snap_rc, m->snap_busy ? ", dump en cours" : "");
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

	/* 7. Paramètres de procédure. max_procedure_count = 0 : procédures
	 * répétées par le contrôleur tous les procedure_interval jusqu'à
	 * disable explicite (FREE-RUNNING, cf. cs_config.c — en RT on ne
	 * disable jamais, en on-demand cs_wait_done_beacon disable après
	 * chaque mesure). max_subevent_len doit rester <=
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

/* ── Mesure one-shot (fallback on-demand uniquement) ────────────────────── */

#if !CS_RAS_REALTIME
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
#endif /* !CS_RAS_REALTIME */

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
		k_mutex_init(&meas[i].lock);
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

/* ── ARMEMENT : lance les procédures CS du beacon i SANS attendre. ───────────
 * FREE-RUNNING (max_procedure_count = 0) : à appeler UNE FOIS par beacon (et
 * après reconnexion) — le contrôleur répète ensuite les procédures tous les
 * procedure_interval sans nouveau handshake LL. Retourne 0 si l'enable est
 * accepté (le SDC autorise le CS concurrent). */
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
#if CS_RAS_REALTIME
	/* Armement (unique) : repartir d'un état d'appariement vierge. */
	k_mutex_lock(&meas[i].lock, K_FOREVER);
	meas[i].snap_valid  = false;
	meas[i].snap_busy   = false;
	meas[i].snap_t_done = 0;
	k_mutex_unlock(&meas[i].lock);
	k_sem_reset(&meas[i].sem_rd_ready);
#endif
	/* On-demand : PAS de reset de sem_rd_ready ici — le rd_ready de la
	 * procédure N (pas encore fetchée) peut arriver juste avant/après cet
	 * enable de N+1, le reset le perdrait et cs_fetch_beacon timeouterait. */

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

#if !CS_RAS_REALTIME
/* ── PHASE 2a (on-demand) : attend la fin de la procédure du beacon i et
 * SNAPSHOTTE ses steps locaux + compteur + timings. Après retour 0, l'appelant
 * peut ré-armer immédiatement (cs_enable_beacon) : la procédure N+1 écrira
 * dans `local` sans toucher au snapshot de N. */
static int cs_wait_done_beacon(int i)
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

	cs_snapshot_from_local(&meas[i]);
	if (meas[i].se_count == 0) {
		/* Diagnostic : données locales présentes mais aucun subevent
		 * horodaté — divergence de code ou chemin inattendu. */
		LOG_WRN("b%d: local %u octets mais se_count=0", i, meas[i].local.len);
	}
	return 0;
}

/* ── PHASE 2b (on-demand) : récupère les données RAS du pair pour le SNAPSHOT
 * du beacon i (fetch séquentiel : buffer peer partagé) et émet le dump IQ
 * horodaté (IQL/IQP). La procédure N+1 peut déjà être en vol : elle
 * n'interfère pas (buffers et flags découplés). Retourne 0 si la donnée pair
 * a été récupérée et émise, < 0 sinon. AUCUN calcul de distance. */
static int cs_fetch_beacon(int i)
{
	struct bt_conn *conn = beacons[i].conn;

	if (conn == NULL || meas[i].snap.len == 0) {
		return -1;
	}

	if (k_sem_take(&meas[i].sem_rd_ready, K_SECONDS(5))) {
		LOG_WRN("Beacon[%d] rd_ready timeout", i);
		return -1;
	}

	int64_t t_fetch_start = k_uptime_get();

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
		return -1;
	}
	if (k_sem_take(&sem_rd_complete, K_SECONDS(5)) || meas[i].fetch_failed) {
		LOG_WRN("Beacon[%d] ranging data timeout", i);
		cs_active_conn = NULL;
		return -1;
	}
	cs_active_conn = NULL;
	struct net_buf_simple *peer = &latest_peer_steps;

	if (cs_iq_dump) {
		cs_iq_emit(i, peer);
	}

	/* Décomposition : startup = enable → 1er subevent (latence de
	 * scheduling SDC, ∝ intervalle de connexion), meas = 1er subevent →
	 * done, fetch = attente donnée pair (GATT on-demand). */
	LOG_INF("b%d timing: startup=%lld ms meas=%lld ms fetch=%lld ms",
		i, meas[i].snap_startup_ms, meas[i].snap_meas_ms,
		k_uptime_get() - t_fetch_start);

	return 0;
}
#endif /* !CS_RAS_REALTIME */

/* ── COLLECTE d'une mesure appariée du beacon i ──────────────────────────────
 * RT free-running : attend la prochaine paire (snapshot local + notification
 * pair du même compteur, signalée par rt_data_cb), fige le snapshot le temps
 * du dump IQ (snap_busy : les callbacks sautent la mesure suivante plutôt que
 * d'écraser), émet IQL/IQP. Ne consomme AUCUN airtime : les procédures
 * continuent en fond sur tous les liens pendant le dump.
 * On-demand : reproduit l'ancien pipeline wait → ré-enable → fetch ;
 * *rearmed vaut false si le ré-enable a échoué (l'appelant doit ré-armer).
 * Retourne 0 si une mesure a été émise, < 0 sinon. */
int cs_collect_beacon(int i, bool *rearmed)
{
#if CS_RAS_REALTIME
	struct cs_meas *m = &meas[i];
	int64_t t_wait = k_uptime_get();

	*rearmed = true;	/* free-running : rien à ré-armer */

	/* 2 s >> période de mesure (procedure_interval × intervalle de
	 * connexion) : un timeout signifie lien mort ou aborts en rafale. */
	if (k_sem_take(&m->sem_rd_ready, K_SECONDS(2))) {
		LOG_WRN("Beacon[%d] aucune mesure appariee (timeout)", i);
		return -ETIMEDOUT;
	}

	k_mutex_lock(&m->lock, K_FOREVER);
	if (!m->snap_valid || m->peer_snap_rc != m->snap_rc) {
		/* Le snapshot a été écrasé par une procédure plus récente pas
		 * encore appariée : la prochaine paire re-signalera. */
		k_mutex_unlock(&m->lock);
		return -EIO;
	}
	m->snap_busy = true;
	k_mutex_unlock(&m->lock);

	if (cs_iq_dump) {
		cs_iq_emit(i, &m->peer_snap);
	}

	/* period = espacement entre fins de procédures consécutives (cadence
	 * d'échantillonnage réelle), meas = 1er subevent → done, wait = temps
	 * passé par la collecte à attendre cette paire. */
	LOG_INF("b%d timing: period=%lld ms meas=%lld ms wait=%lld ms",
		i, m->snap_prev_done ? (m->snap_t_done - m->snap_prev_done) : -1,
		m->snap_meas_ms, k_uptime_get() - t_wait);

	k_mutex_lock(&m->lock, K_FOREVER);
	m->snap_busy = false;
	k_mutex_unlock(&m->lock);
	return 0;
#else
	int werr = cs_wait_done_beacon(i);

	/* Ré-armer AVANT le fetch : le startup de N+1 recouvre le fetch de N. */
	*rearmed = (cs_enable_beacon(i) == 0);

	return (werr == 0) ? cs_fetch_beacon(i) : werr;
#endif /* CS_RAS_REALTIME */
}
