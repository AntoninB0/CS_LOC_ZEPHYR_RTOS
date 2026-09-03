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
#include "manager.h"
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
		/* No unref here: the application has not taken a reference yet,
		 * the scan module manages its own. */
		return;
	}

	/* Reject duplicate connection to already-connected beacon */
	const bt_addr_le_t *addr_le = bt_conn_get_dst(conn);

	/* Whitelist manager: rejects any address off the list when the
	 * whitelist is active (WL:ON). Always OK when disabled. */
	if (!manager_addr_allowed(addr_le)) {
		LOG_WRN("Connection rejected (not whitelisted): %s", addr);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

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

	/* Release-once: the unref is done only if THIS call freed the slot.
	 * Avoids the double-unref (refcount underflow) racing with the pairing
	 * thread's error path. */
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
	/* Initial connection parameters = cs_cp_active (single source in
	 * cs_config.h/.c). No static init possible from a variable: we point
	 * the scan directly at the global struct. */
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

/* ── 1 Hz display ────────────────────────────────────────────────────────── */

/* 2048: the cbprintf packaging of the %f in LOG_INF happens in the caller
 * context — 512 bytes overflowed the stack (random hard fault). */
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
			LOG_INF("[%d] %s  %s", i, addr_str,
				beacons[i].cs_ready ? "ranging" : "setup");
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

	/* Application manager: UART commands (IQON/IQOFF, whitelist, chmap),
	 * connection filtering by whitelist, channel-map hot-swap staging. */
	manager_init();
	if_set_line_cb(manager_uart_line);

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

	/* ── Blocking init: do not continue until the MAX_BEACONS reflectors
	 *    are connected and secured ──────────────────────────────────── */
	LOG_INF("Waiting for %d reflectors...", MAX_BEACONS);
	err = pairing_wait_all_connected(K_FOREVER);
	if (err) {
		LOG_ERR("Wait for reflectors failed (err %d)", err);
		return 0;
	}
	LOG_INF("All %d reflectors connected — starting application", MAX_BEACONS);

	/* ── CONTINUOUS MEASUREMENT (free-running) ───────────────────────────────
	 * Each beacon's CS procedures are armed ONCE (max_procedure_count = 0):
	 * the controller then repeats them every procedure_interval without a new
	 * LL handshake — the ~495 ms startup (≈ 11 x connection interval) is paid
	 * only at arming, never again per measurement. The old one-shot pipeline
	 * paid this startup on EVERY measurement (3-beacon cycle ≈ 1485 ms); here
	 * the loop only collects the pairs (local steps + peer RAS) as they come:
	 * cycle ≈ procedure period (~180 ms in drone N=3).
	 * NOTE: no bt_conn_ref during the measurement (a disconnect mid-cycle is
	 * unprotected) — OK on the bench, to harden for production. */
	cs_ranging_init();

	bool armed[MAX_BEACONS] = { false };

	while (true) {
		bool enable_failed = false;
		bool any_collected = false;
		int64_t t_cycle = k_uptime_get();

		/* Simple round-robin over all beacons (the scheduler was removed:
		 * in free-running each link measures continuously, the order has
		 * no effect on the rate). */
		for (int i = 0; i < MAX_BEACONS; i++) {
			bool ready;

			k_mutex_lock(&beacons_mutex, K_FOREVER);
			ready = beacons[i].active && beacons[i].cs_ready &&
				beacons[i].conn != NULL;
			k_mutex_unlock(&beacons_mutex);

			if (!ready) {
				/* Disconnected: the controller has stopped its
				 * procedures, we will re-arm on return. */
				armed[i] = false;
				continue;
			}

			/* Not armed yet (startup, reconnection, or failure on
			 * the previous round): arm and move on — the first
			 * measurement arrives after the LL startup, while the
			 * loop collects the other beacons. */
			if (!armed[i]) {
				armed[i] = (cs_enable_beacon(i) == 0);
				enable_failed |= !armed[i];
				continue;
			}

			/* Collect the next paired measurement + timestamped IQ
			 * dump (IQL/IQP). No distance: computed off-board
			 * (Python) from the IQ lines. */
			bool rearmed = true;
			int rc = cs_collect_beacon(i, &rearmed);

			any_collected = true;
			if (!rearmed) {
				armed[i] = false;
				enable_failed = true;
			}
			if (rc != 0) {
				LOG_WRN("Beacon[%d] ranging failed", i);
			}
		}

		LOG_INF("cycle: %lld ms", k_uptime_get() - t_cycle);

		/* Backoff if nothing was collected this round (all disconnected
		 * or just armed): avoids a busy spin. In nominal operation the
		 * loop is paced by the collection semaphores (cs_collect_beacon). */
		if (enable_failed || !any_collected) {
			k_sleep(K_MSEC(50));
		}
	}

	return 0;
}
