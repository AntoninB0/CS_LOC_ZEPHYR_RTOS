/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include "beacon.h"
#include "pairing.h"
#include "cs_ranging.h"
#include "if.h"
#include "scheduler.h"
#include "cs_config.h"

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

/* Ranging Service UUID 0x185B — advertised by the reflector firmware */
static struct bt_uuid_16 ranging_svc_uuid = BT_UUID_INIT_16(0x185B);
#define BT_UUID_RANGING_SERVICE ((struct bt_uuid *)&ranging_svc_uuid)

#define CON_STATUS_LED DK_LED1

/* Declared in pairing.c */
extern void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err);

/* ── Connection callbacks ────────────────────────────────────────────────── */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s (err 0x%02X)", addr, err);

	if (err) {
		/* Pas d'unref ici : l'application n'a pas encore pris de référence,
		 * le module scan gère la sienne. */
		return;
	}

	/* Reject duplicate connection to already-connected beacon */
	const bt_addr_le_t *addr_le = bt_conn_get_dst(conn);

	k_mutex_lock(&beacons_mutex, K_FOREVER);
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].active && beacons[i].conn != NULL &&
		    bt_addr_le_cmp(bt_conn_get_dst(beacons[i].conn), addr_le) == 0) {
			k_mutex_unlock(&beacons_mutex);
			LOG_WRN("Duplicate connection rejected");
			bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			return;
		}
	}
	k_mutex_unlock(&beacons_mutex);

	struct bt_conn *ref = bt_conn_ref(conn);

	if (k_msgq_put(&new_conn_queue, &ref, K_NO_WAIT) != 0) {
		LOG_ERR("Queue full, disconnecting");
		bt_conn_unref(ref);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason 0x%02X)", addr, reason);

	/* Release-once : l'unref n'est fait que si CET appel a libéré le slot.
	 * Évite le double-unref (underflow de refcount) en course avec le
	 * chemin d'erreur du thread de pairing. */
	if (free_beacon(conn)) {
		bt_conn_unref(conn);
	}

	dk_set_led_off(CON_STATUS_LED);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	return false;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected      = connected_cb,
	.disconnected   = disconnected_cb,
	.le_param_req   = le_param_req,
	.security_changed = security_changed_cb,
};

/* ── Scan ────────────────────────────────────────────────────────────────── */

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("Found beacon: %s", addr);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connection attempt failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	LOG_INF("Connecting...");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	/* Paramètres de connexion initiaux = cs_cp_active (source unique dans
	 * cs_config.h/.c). Pas d'init statique possible depuis une variable :
	 * on pointe directement le scan sur la struct globale. */
	struct bt_scan_init_param param = {
		.scan_param = NULL,
		.conn_param = &cs_cp_active,
		.connect_if_match = 1,
	};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		LOG_ERR("Filter add failed (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filter enable failed (err %d)", err);
		return err;
	}

	return 0;
}

/* ── Affichage 1 Hz ──────────────────────────────────────────────────────── */

/* 2048 : le packaging cbprintf des %f par LOG_INF se fait dans le contexte
 * appelant — 512 octets débordaient la stack (hard fault aléatoire). */
#define DISPLAY_STACK_SIZE 2048
#define DISPLAY_PRIORITY   7
K_THREAD_STACK_DEFINE(display_stack, DISPLAY_STACK_SIZE);
static struct k_thread display_thread_data;

static void display_thread(void *p1, void *p2, void *p3)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	while (true) {
		k_sleep(K_MSEC(1000));

		k_mutex_lock(&beacons_mutex, K_FOREVER);
		int n = 0;

		for (int i = 0; i < MAX_BEACONS; i++) {
			if (!beacons[i].active) {
				continue;
			}
			n++;
			bt_addr_le_to_str(&beacons[i].addr, addr_str, sizeof(addr_str));
			if (beacons[i].last_distance >= 0.0f) {
				LOG_INF("[%d] %s  %.2f m", i, addr_str,
					(double)beacons[i].last_distance);
			} else {
				LOG_INF("[%d] %s  -- m", i, addr_str);
			}
		}

		if (n == 0) {
			LOG_INF("No beacons connected");
		}

		k_mutex_unlock(&beacons_mutex);
	}
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
	int err;

	LOG_INF("Starting multi-beacon initiator");

	dk_leds_init();

	if (if_initialisation()) {
		LOG_ERR("UART init failed");
	}

	/* Scheduler de mesures : ordre de passage AUTO (erreur min privilégiée)
	 * ou imposé par UART ("ORDER:0,1,1" / "AUTO", voir scheduler.h). */
	scheduler_init();
	if_set_line_cb(scheduler_uart_line);

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	err = scan_init();
	if (err) {
		LOG_ERR("Scan init failed (err %d)", err);
		return 0;
	}

	k_thread_create(&pairing_thread_data, pairing_stack, PAIRING_STACK_SIZE,
			(k_thread_entry_t)pairing_n_bluetooth, NULL, NULL, NULL,
			PAIRING_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&display_thread_data, display_stack, DISPLAY_STACK_SIZE,
			display_thread, NULL, NULL, NULL,
			DISPLAY_PRIORITY, 0, K_NO_WAIT);

	/* ── Init bloquant : on ne continue pas tant que les MAX_BEACONS
	 *    réflecteurs ne sont pas connectés et sécurisés ─────────────── */
	LOG_INF("Waiting for %d reflectors...", MAX_BEACONS);
	err = pairing_wait_all_connected(K_FOREVER);
	if (err) {
		LOG_ERR("Wait for reflectors failed (err %d)", err);
		return 0;
	}
	LOG_INF("All %d reflectors connected — starting application", MAX_BEACONS);

	/* ── PIPELINE CONTINU : ré-enable IMMÉDIAT après chaque collecte ─────────
	 * v1 (enable-tous puis collect-tous par cycle) ne recouvrait les startups
	 * qu'À L'INTÉRIEUR d'un cycle : entre deux cycles, le startup repartait de
	 * zéro à découvert. Ici chaque beacon est ré-armé dès sa collecte finie :
	 * le startup de sa procédure N+1 court PENDANT le fetch RAS des autres
	 * beacons. En régime établi, le temps de cycle tend vers
	 * max(startup, Σ fetch) au lieu de startup + Σ fetch.
	 * (Le startup n'occupe pas le radio : c'est de l'attente de scheduling,
	 * donc parfaitement recouvrable par le GATT.)
	 * NOTE : pas de bt_conn_ref pendant la mesure (déconnexion en plein cycle
	 * non protégée) — OK sur banc, à durcir pour la prod. */
	cs_ranging_init();

	bool in_flight[MAX_BEACONS] = { false };
	uint8_t seq[SCHED_ROUND_MAX];

	while (true) {
		bool enable_failed = false;
		int64_t t_cycle = k_uptime_get();

		/* Le scheduler fournit l'ordre de passage du round : indices de
		 * beacons prêts, répétitions permises (un beacon "pertinent"
		 * peut être mesuré plusieurs fois par round). AUTO : slots
		 * bonus aux beacons de moindre erreur ; MANUAL : ordre UART. */
		int len = scheduler_build_round(seq);

		if (len == 0) {
			for (int i = 0; i < MAX_BEACONS; i++) {
				in_flight[i] = false;
			}
			k_sleep(K_MSEC(50));
			continue;
		}

		for (int k = 0; k < len; k++) {
			int  i = seq[k];
			bool ready;

			k_mutex_lock(&beacons_mutex, K_FOREVER);
			ready = beacons[i].active && beacons[i].cs_ready &&
				beacons[i].conn != NULL;
			k_mutex_unlock(&beacons_mutex);

			if (!ready) {
				/* Déconnecté depuis la construction du round */
				in_flight[i] = false;
				continue;
			}

			/* Rien en vol (démarrage, ou échec au tour précédent) :
			 * armer et passer au suivant — la collecte attendra le
			 * prochain passage, le startup court pendant ce temps. */
			if (!in_flight[i]) {
				in_flight[i] = (cs_enable_beacon(i) == 0);
				enable_failed |= !in_flight[i];
				continue;
			}

			/* 1. Attendre la fin de la procédure N (snapshot). */
			int werr = cs_wait_done_beacon(i);

			/* 2. Ré-armer AVANT le fetch : le startup de N+1 (pure
			 * attente de scheduling, radio libre) recouvre le fetch
			 * GATT de N ci-dessous ET ceux des autres beacons. */
			in_flight[i] = (cs_enable_beacon(i) == 0);
			enable_failed |= !in_flight[i];

			/* 3. Fetch RAS + distance sur le snapshot de N. */
			float d = (werr == 0) ? cs_fetch_beacon(i) : -1.0f;

			scheduler_report(i, d, werr == 0 && d >= 0.0f);

			k_mutex_lock(&beacons_mutex, K_FOREVER);
			beacons[i].last_distance = d;
			k_mutex_unlock(&beacons_mutex);

			if (d < 0.0f) {
				LOG_WRN("Beacon[%d] ranging failed", i);
			}
		}

		LOG_INF("cycle: %lld ms", k_uptime_get() - t_cycle);

		if (enable_failed) {
			/* Backoff uniquement en dégradé (enable refusé) : en
			 * nominal la boucle est cadencée par les sémaphores de
			 * collecte, pas par un sleep. */
			k_sleep(K_MSEC(50));
		}
	}

	return 0;
}