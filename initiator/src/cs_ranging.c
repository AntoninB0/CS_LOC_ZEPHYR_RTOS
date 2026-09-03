#include "cs_ranging.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include "if.h"
#include "manager.h"
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

/* Channel-map thinning: keep only one valid CS channel out of N. Reduces the
 * radio occupancy per procedure (~50 ms / N), hence the probability of
 * collision with the other links' ACL anchors (0x3 abort), and speeds up each
 * measurement. Trade-off for the IQ pipeline: less frequency spread (N=2 →
 * ~36 channels spaced by ~4 MHz; distance ambiguity c/(2*df) ≈ 37.5 m, of no
 * consequence indoors). The spec requires >= 15 channels: N <= 4. Set to 1 to
 * disable (all channels). */
/* CS_CHANNEL_THINNING and CS_MAX_STEPS_PER_PROC come from the profile
 * (cs_config.h). Too small => "Step data buffer overflow" => abort. */

#define LOCAL_PROCEDURE_MEM                                                                        \
	((CS_MAX_STEPS_PER_PROC * sizeof(struct bt_le_cs_subevent_step)) +                        \
	 (CS_MAX_STEPS_PER_PROC * BT_RAS_MAX_STEP_DATA_LEN))

/* Shared peer (RAS) buffer: the RAS fetch is done sequentially in the
 * collection phase, one beacon at a time. */
#if !CS_RAS_REALTIME
/* SHARED peer buffer of the on-demand mode (sequential fetch). In real-time it
 * disappears: each beacon has its own peer_rt/peer_snap (~1 KB), net RAM gain. */
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
#endif

/* ── FREE-RUNNING: PER-BEACON measurement state ──────────────────────────────
 * CS procedures are armed ONCE per beacon (max_procedure_count = 0): the
 * controller repeats them every procedure_interval, the LL handshake of
 * ~11 x interval (~495 ms) is paid only at arming. In real-time RAS mode the
 * snapshot of each completed procedure is taken INSIDE the callback (BT RX
 * thread) and paired to the peer notification by ranging counter; the main
 * thread only collects (IQ dump). Each beacon has its own local buffer +
 * semaphores: the callbacks (routed by conn) do not step on each other. */
struct cs_meas {
	struct k_mutex        lock;          /* snap/peer_snap: BT RX callback vs collection */
	struct k_sem          sem_done;      /* on-demand: procedure finished (COMPLETE/ABORTED) */
	struct k_sem          sem_rd_ready;  /* RT: local+peer pair ready; on-demand: peer data announced */
	struct net_buf_simple local;         /* local steps accumulated (IN-FLIGHT procedure) */
	uint8_t               local_buf[LOCAL_PROCEDURE_MEM];
	uint16_t              local_rc;
	uint16_t              peer_rc;
	bool                  aborted;       /* in-flight procedure aborted */
	bool                  fetch_failed;  /* failure of the in-progress Get Ranging Data */
	bool                  active;        /* in-flight procedure for this beacon */
	/* Instrumentation (k_uptime, ms): breakdown of the measurement time */
	int64_t               t_enable;      /* bt_le_cs_procedure_enable accepted */
	int64_t               t_first_se;    /* first subevent received (0 if none) */
	int64_t               t_done;        /* procedure COMPLETE/ABORTED */
	/* Per-subevent slicing of the local buffer: offset/length of each slice
	 * + k_uptime timestamp of its arrival (HCI callback). Used by the IQ
	 * dump: the steps of a subevent share its timestamp, the per-step end
	 * instant is then reconstructed from the step durations
	 * (Vol 6 Part B section 4.5.18). */
#define CS_SE_META_MAX 8
/* Reception buffer for the peer ranging data (real-time mode): sized on the
 * profile (RAS headers + ~11 bytes/step in mode-2). */
#define PEER_PROCEDURE_MEM (32 + CS_MAX_STEPS_PER_PROC * 12)
	struct { uint16_t off; uint16_t len; int64_t t_ms; } se_meta[CS_SE_META_MAX];
	uint8_t               se_count;
	/* SNAPSHOT of the last COMPLETED procedure: allows re-arming procedure
	 * N+1 (which writes into `local`) BEFORE the RAS fetch of N. The startup
	 * of N+1 is thus overlapped by the fetch of N itself, on top of those of
	 * the other beacons. */
	struct net_buf_simple snap;
	uint8_t               snap_buf[LOCAL_PROCEDURE_MEM];
	uint16_t              snap_rc;
	bool                  snap_valid;     /* RT: consistent snapshot available */
	bool                  snap_busy;      /* RT: dump in progress (main) — do not overwrite */
	int64_t               snap_startup_ms;
	int64_t               snap_meas_ms;
	int64_t               snap_t_first;   /* k_uptime of the first subevent */
	int64_t               snap_t_done;    /* k_uptime of the procedure end */
	int64_t               snap_prev_done; /* t_done of the previous snapshot (period) */
#if CS_RAS_REALTIME
	/* Real-time: peer_rt is the SUBSCRIBED buffer (auto-reset by the lib
	 * after each callback → mandatory copy into peer_snap, consumed by
	 * cs_fetch_beacon). */
	struct net_buf_simple peer_rt;
	uint8_t               peer_rt_buf[PEER_PROCEDURE_MEM];
	struct net_buf_simple peer_snap;
	uint8_t               peer_snap_buf[PEER_PROCEDURE_MEM];
	uint16_t              peer_snap_rc;
#endif
	struct { uint16_t off; uint16_t len; int64_t t_ms; } snap_se_meta[CS_SE_META_MAX];
	uint8_t               snap_se_count;
};

/* IQ dump on the data UART (UART commands "IQON"/"IQOFF", cf. scheduler).
 * ON by default: the timestamped IQL/IQP dump IS the initiator output — the
 * distance estimation is done off-board (Python). */
bool cs_iq_dump = true;
static struct cs_meas meas[MAX_BEACONS];

/* Connection concerned by the callbacks in progress (setup OR measurement) */
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

/* Index of the beacon associated with conn (lockless: conns are stable during a
 * measurement). -1 if not found. Used to route the CS/RAS callbacks. */
static int meas_idx(struct bt_conn *conn)
{
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].conn == conn) {
			return i;
		}
	}
	return -1;
}

/* ── MTU exchange ────────────────────────────────────────────────────────────
 * Without bt_gatt_exchange_mtu, the ATT MTU stays at 23 bytes: the RAS transfer
 * (several KB) becomes very slow or even exceeds the timeouts. */

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

/* ── GATT discovery of the Ranging Service ──────────────────────────────── */

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

/* ── Channel Sounding callbacks (bt_conn) ───────────────────────────────── */

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

/* Copy local → snap + metadata/timings. Called under m->lock in RT
 * (BT RX callback), without lock in on-demand (main thread exclusive). */
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
		/* Aborted subevent (typ. 0x3: conflict with another link's ACL). */
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
		/* FREE-RUNNING: the next procedure starts on its own within
		 * procedure_interval — immediate snapshot HERE (the main thread
		 * has no time to do it before the subevents of N+1 arrive in
		 * `local`). The pairing semaphore is given by rt_data_cb when the
		 * peer half of the SAME counter arrives. */
		k_mutex_lock(&m->lock, K_FOREVER);
		if (!m->snap_busy && !m->aborted && m->local.len > 0) {
			cs_snapshot_from_local(m);
			m->snap_valid = true;
		} else if (m->snap_busy) {
			LOG_WRN("b%d: dump in progress, measurement %u skipped", i, m->local_rc);
		}
		k_mutex_unlock(&m->lock);
		/* Prepare the accumulation of the next procedure. */
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
		/* Sacrificed measurement, the next one arrives within procedure_interval. */
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

/* ── RAS callbacks (Ranging Service client) ─────────────────────────────── */

#if CS_RAS_REALTIME
/* Complete real-time notification (BT RX thread). The subscribed buffer
 * (peer_rt) is reset by the lib on return: we copy it into peer_snap.
 * PAIRING: the pair (local snap, peer_snap) is signalled to the collection
 * thread (sem_rd_ready) only if the peer counter == the local snapshot counter
 * — the local HCI results of a procedure always precede the reflector's GATT
 * notification (~1 connection event later), so nominally every procedure pairs;
 * a mismatched counter = sacrificed measurement (collection slower than the
 * rate, or a local abort). */
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
		LOG_ERR("RT data b%d too large (%u)", i, meas[i].peer_rt.len);
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
	LOG_WRN("b%d: pair lost (peer %u vs snap %u%s)", i, ranging_counter,
		m->snap_rc, m->snap_busy ? ", dump in progress" : "");
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
			/* DEDICATED flag: `aborted` belongs to the IN-FLIGHT
			 * procedure (N+1), which may already have restarted
			 * during this fetch of N — do not pollute it. */
			meas[i].fetch_failed = true;
		}
	}
	k_sem_give(&sem_rd_complete);
}
#endif /* !CS_RAS_REALTIME */

/* ── Channel map ────────────────────────────────────────────────────────── */

static void cs_build_channel_map(uint8_t channel_map[10])
{
	/* Has the manager imposed a channel map (CHMAP:* over UART)? If so it
	 * takes precedence over the profile's compile-time map. Takes effect at
	 * THIS config creation; a live hot-swap requires re-creating the config
	 * during the connection (procedure restart). */
	if (manager_get_chmap(channel_map)) {
		LOG_INF("CS channel map: provided by the manager");
		return;
	}

	bt_le_cs_set_valid_chmap_bits(channel_map);

#if CS_CHANNEL_THINNING > 1
	/* Keep only one valid channel out of CS_CHANNEL_THINNING, walking the
	 * channels in frequency order. */
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

/* ── Per-beacon setup ───────────────────────────────────────────────────── */

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

	/* 0. MTU exchange. Static struct: must survive until the callback.
	 * OK because the setup is strictly sequential. */
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

	/* 1. GATT discovery of the Ranging Service + RREQ client allocation */
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

	/* 2. Remote CS capabilities */
	err = bt_le_cs_read_remote_supported_capabilities(conn);
	if (err) {
		LOG_ERR("Read remote CS capabilities failed (err %d)", err);
		goto cleanup;
	}
	if (k_sem_take(&sem_caps, K_SECONDS(10))) {
		err = -ETIMEDOUT;
		goto cleanup;
	}

	/* 3. Default settings: initiator role */
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

	/* 4. RAS subscriptions — two EXCLUSIVE modes (the server rejects the
	 * double subscription): real-time (notifications pushed at the end of
	 * each procedure, into the beacon's peer_rt buffer) or on-demand
	 * (rd_ready + Get Ranging Data on the RAS Control Point, whose CP
	 * subscription is then MANDATORY — responses as indications). */
#if CS_RAS_REALTIME
	{
		int idx = meas_idx(conn);

		if (idx < 0) {
			LOG_ERR("RT subscribe: unknown beacon");
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

	/* 5. CS config: mode 2 (PBR) + sub-mode 1, validated sample base,
	 * chmap repetition 1 and channel map optionally thinned to reduce the
	 * airtime per procedure (see CS_CHANNEL_THINNING). */
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

	/* 6. CS security (ranging keys) */
	err = bt_le_cs_security_enable(conn);
	if (err) {
		LOG_ERR("CS security enable failed (err %d)", err);
		goto cleanup;
	}
	if (k_sem_take(&sem_cs_security, K_SECONDS(10))) {
		err = -ETIMEDOUT;
		goto cleanup;
	}

	/* 7. Procedure parameters. max_procedure_count = 0: procedures repeated
	 * by the controller every procedure_interval until an explicit disable
	 * (FREE-RUNNING, cf. cs_config.c — in RT we never disable, in on-demand
	 * cs_wait_done_beacon disables after each measurement). max_subevent_len
	 * must stay <= CONFIG_BT_CTLR_SDC_CS_EVENT_LEN_DEFAULT on both sides. */
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

/* ── One-shot measurement (on-demand fallback only) ─────────────────────── */

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

	/* The disable is ASYNCHRONOUS: wait for the confirmation
	 * (state = DISABLED) before any re-enable, otherwise the controller
	 * refuses with HCI 0x0C Command Disallowed. */
	if (k_sem_take(&sem_procedure_disabled, K_SECONDS(2))) {
		LOG_WRN("Timeout waiting for CS procedure disable");
	}
}
#endif /* !CS_RAS_REALTIME */

/* Init of the per-beacon buffers/semaphores. Call once before the loop. */
/* ── Automatic downgrade of the OPTIONAL CS options ─────────────────────────
 * Create Config returns 0x11 (Unsupported Feature or Parameter Value) if the
 * controller does not support a requested optional capability: CS_SYNC 2M_2BT,
 * CSA #3C, RTT sounding. We read the LOCAL caps at boot, log them, and fall
 * back each unsupported option to its base value — the WRN log names the
 * culprit. Both ends being identical nRF54L15/SDC, the local check covers the
 * bench (in a heterogeneous setup, also read the REMOTE caps before setup). */
static void cs_apply_local_caps(void)
{
	struct bt_conn_le_cs_capabilities caps;

	if (bt_le_cs_read_local_supported_capabilities(&caps)) {
		LOG_WRN("Local CS caps unreadable, options kept as-is");
		return;
	}
	LOG_INF("CS caps: 2M_2BT=%d CSA3C=%d RTTsound_n=%u RTTaa_n=%u NADMsound=%d",
		caps.cs_sync_2m_2bt_phy_supported, caps.chsel_alg_3c_supported,
		caps.rtt_sounding_n, caps.rtt_aa_only_n,
		caps.phase_based_nadm_sounding_supported);

	if (cs_create_config.cs_sync_phy == BT_CONN_LE_CS_SYNC_2M_2BT_PHY &&
	    !caps.cs_sync_2m_2bt_phy_supported) {
		LOG_WRN("caps: CS_SYNC 2M_2BT unsupported -> fallback 2M");
		cs_create_config.cs_sync_phy = BT_CONN_LE_CS_SYNC_2M_PHY;
	}
	if (cs_create_config.channel_selection_type == BT_CONN_LE_CS_CHSEL_TYPE_3C &&
	    !caps.chsel_alg_3c_supported) {
		LOG_WRN("caps: CSA #3C unsupported -> fallback #3B");
		cs_create_config.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	}
	if (cs_create_config.rtt_type == BT_CONN_LE_CS_RTT_TYPE_32_BIT_SOUNDING &&
	    caps.rtt_sounding_n == 0) {
		LOG_WRN("caps: RTT sounding unsupported -> fallback AA_ONLY");
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

/* ── ARM: start beacon i's CS procedures WITHOUT waiting. ────────────────────
 * FREE-RUNNING (max_procedure_count = 0): call ONCE per beacon (and after a
 * reconnection) — the controller then repeats the procedures every
 * procedure_interval without a new LL handshake. Returns 0 if the enable is
 * accepted (the SDC allows concurrent CS). */
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
	/* Arming (one-time): start from a clean pairing state. */
	k_mutex_lock(&meas[i].lock, K_FOREVER);
	meas[i].snap_valid  = false;
	meas[i].snap_busy   = false;
	meas[i].snap_t_done = 0;
	k_mutex_unlock(&meas[i].lock);
	k_sem_reset(&meas[i].sem_rd_ready);
#endif
	/* On-demand: do NOT reset sem_rd_ready here — the rd_ready of procedure
	 * N (not yet fetched) can arrive just before/after this enable of N+1,
	 * a reset would lose it and cs_fetch_beacon would time out. */

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

/* ── IQ dump to the data UART (ASCII lines, format in doc GUIDE.md) ────────
 * IQL,<beacon>,<counter>,<t_ms_subevent>,<hex subevent steps>  (x subevents)
 * IQP,<beacon>,<counter>,<t_ms_done>,<hex peer RAS ranging data>
 * Emitted from the measurement thread AFTER the fetch: adds only the UART write
 * time to the cycle (at 115200 baud ~30-60 ms/procedure → move uart21 to
 * 1 Mbaud in devicetree if the dump is used continuously). */
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
		/* Safety net: no per-subevent slicing available — emit the whole
		 * procedure, timestamped at the first subevent. */
		n = snprintf(hdr, sizeof(hdr), "IQL,%d,%u,%lld,", i,
			     meas[i].snap_rc, meas[i].snap_t_first);
		if_send((uint8_t *)hdr, n);
		iq_emit_hex(meas[i].snap.data, meas[i].snap.len);
		if_send((uint8_t *)"\n", 1);
	}
	/* snap_t_done: not meas[i].t_done, which is reset by the enable of N+1
	 * that precedes this fetch in the pipeline. */
	n = snprintf(hdr, sizeof(hdr), "IQP,%d,%u,%lld,", i, meas[i].snap_rc,
		     meas[i].snap_t_done);
	if_send((uint8_t *)hdr, n);
	iq_emit_hex(peer->data, peer->len);
	if_send((uint8_t *)"\n", 1);
}

#if !CS_RAS_REALTIME
/* ── PHASE 2a (on-demand): wait for the end of beacon i's procedure and
 * SNAPSHOT its local steps + counter + timings. After returning 0, the caller
 * can re-arm immediately (cs_enable_beacon): procedure N+1 will write into
 * `local` without touching N's snapshot. */
static int cs_wait_done_beacon(int i)
{
	struct bt_conn *conn = beacons[i].conn;

	if (!meas[i].active || conn == NULL) {
		return -EINVAL;
	}

	int err = k_sem_take(&meas[i].sem_done, K_SECONDS(5));

	if (cs_proc_params.max_procedure_count == 0) {
		/* cs_active_conn required by procedure_enable_cb (disable) */
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
		/* Diagnostic: local data present but no timestamped subevent —
		 * code divergence or unexpected path. */
		LOG_WRN("b%d: local %u bytes but se_count=0", i, meas[i].local.len);
	}
	return 0;
}

/* ── PHASE 2b (on-demand): fetch the peer RAS data for beacon i's SNAPSHOT
 * (sequential fetch: shared peer buffer) and emit the timestamped IQ dump
 * (IQL/IQP). Procedure N+1 may already be in flight: it does not interfere
 * (buffers and flags decoupled). Returns 0 if the peer data was fetched and
 * emitted, < 0 otherwise. NO distance computation. */
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
		/* The peer already announced a newer counter (very fast N+1):
		 * non-blocking, we explicitly fetch snap_rc — the reflector
		 * retention (RD_BUFFERS_PER_CONN=10) keeps it. */
		LOG_WRN("Beacon[%d] counter skew (snap %u peer %u)",
			i, meas[i].snap_rc, meas[i].peer_rc);
	}

	/* cs_active_conn routes rd_get_complete_cb (sequential fetch). */
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

	/* Breakdown: startup = enable → first subevent (SDC scheduling latency,
	 * proportional to the connection interval), meas = first subevent →
	 * done, fetch = wait for the peer data (GATT on-demand). */
	LOG_INF("b%d timing: startup=%lld ms meas=%lld ms fetch=%lld ms",
		i, meas[i].snap_startup_ms, meas[i].snap_meas_ms,
		k_uptime_get() - t_fetch_start);

	return 0;
}
#endif /* !CS_RAS_REALTIME */

/* ── COLLECT a paired measurement from beacon i ──────────────────────────────
 * RT free-running: wait for the next pair (local snapshot + peer notification
 * of the same counter, signalled by rt_data_cb), freeze the snapshot during the
 * IQ dump (snap_busy: the callbacks skip the next measurement rather than
 * overwrite), emit IQL/IQP. Consumes NO airtime: the procedures keep running in
 * the background on all links during the dump.
 * On-demand: reproduces the old wait → re-enable → fetch pipeline; *rearmed is
 * false if the re-enable failed (the caller must re-arm).
 * Returns 0 if a measurement was emitted, < 0 otherwise. */
int cs_collect_beacon(int i, bool *rearmed)
{
#if CS_RAS_REALTIME
	struct cs_meas *m = &meas[i];
	int64_t t_wait = k_uptime_get();

	*rearmed = true;	/* free-running: nothing to re-arm */

	/* 2 s >> measurement period (procedure_interval x connection interval):
	 * a timeout means a dead link or a burst of aborts. */
	if (k_sem_take(&m->sem_rd_ready, K_SECONDS(2))) {
		LOG_WRN("Beacon[%d] no paired measurement (timeout)", i);
		return -ETIMEDOUT;
	}

	k_mutex_lock(&m->lock, K_FOREVER);
	if (!m->snap_valid || m->peer_snap_rc != m->snap_rc) {
		/* The snapshot was overwritten by a newer, not-yet-paired
		 * procedure: the next pair will re-signal. */
		k_mutex_unlock(&m->lock);
		return -EIO;
	}
	m->snap_busy = true;
	k_mutex_unlock(&m->lock);

	if (cs_iq_dump) {
		cs_iq_emit(i, &m->peer_snap);
	}

	/* period = spacing between consecutive procedure ends (real sampling
	 * rate), meas = first subevent → done, wait = time the collector spent
	 * waiting for this pair. */
	LOG_INF("b%d timing: period=%lld ms meas=%lld ms wait=%lld ms",
		i, m->snap_prev_done ? (m->snap_t_done - m->snap_prev_done) : -1,
		m->snap_meas_ms, k_uptime_get() - t_wait);

	k_mutex_lock(&m->lock, K_FOREVER);
	m->snap_busy = false;
	k_mutex_unlock(&m->lock);
	return 0;
#else
	int werr = cs_wait_done_beacon(i);

	/* Re-arm BEFORE the fetch: the startup of N+1 overlaps the fetch of N. */
	*rearmed = (cs_enable_beacon(i) == 0);

	return (werr == 0) ? cs_fetch_beacon(i) : werr;
#endif /* CS_RAS_REALTIME */
}
