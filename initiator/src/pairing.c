#include "pairing.h"
#include "beacon.h"
#include "cs_ranging.h"
#include "cs_config.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(pairing_stack, PAIRING_STACK_SIZE);
struct k_thread pairing_thread_data;

static K_SEM_DEFINE(sem_secured, 0, 1);
/* Given when MAX_BEACONS reflectors are connected + secured */
static K_SEM_DEFINE(sem_all_connected, 0, 1);

static volatile bool security_failed;

void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed (err %d)", err);

		if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING) {
			/* Stale bond: the reflector restarted and lost its keys
			 * (RAM only, no CONFIG_BT_SETTINGS), but we keep its old
			 * LTK and try to encrypt with it instead of re-pairing.
			 * We purge the bond: the next attempt will do a full
			 * pairing. */
			int uerr = bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));

			if (uerr) {
				LOG_WRN("Unpair failed (err %d)", uerr);
			} else {
				LOG_INF("Stale bond removed, will re-pair");
			}
		}

		/* Fast-fail: wake the pairing thread immediately instead of
		 * consuming the 10 s timeout */
		security_failed = true;
		k_sem_give(&sem_secured);
		return;
	}
	LOG_INF("Security level %d", level);
	k_sem_give(&sem_secured);
}

static int count_active_beacons(void)
{
	int count = 0;

	k_mutex_lock(&beacons_mutex, K_FOREVER);
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].active) {
			count++;
		}
	}
	k_mutex_unlock(&beacons_mutex);

	return count;
}

int pairing_wait_all_connected(k_timeout_t timeout)
{
	int err;

	/* k_sem_reset() on the pairing side wakes the waiters with -EAGAIN:
	 * we re-arm the wait in that case instead of returning. */
	do {
		err = k_sem_take(&sem_all_connected, timeout);
	} while (err == -EAGAIN);

	return err;
}

void pairing_n_bluetooth(void)
{
	int err;
	struct bt_conn *conn;
	bool was_full = false;

	while (true) {
		int count = count_active_beacons();

		if (count >= MAX_BEACONS) {
			/* Quota reached: stop the scan and signal main() */
			if (!was_full) {
				bt_scan_stop();
				k_sem_give(&sem_all_connected);
				was_full = true;
			}
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Quota not reached. The reset is done ONLY on the full →
		 * not-full transition (loss of a beacon): a reset on every
		 * iteration woke main() with -EAGAIN right at boot. */
		if (was_full) {
			was_full = false;
			k_sem_reset(&sem_all_connected);
		}

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		if (err && err != -EALREADY) {
			LOG_ERR("Scan start failed (err %d)", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_INF("Scanning... (%d/%d)", count, MAX_BEACONS);

		/* Blocking wait for a new connection */
		int retries = 0;

		while (k_msgq_get(&new_conn_queue, &conn, K_SECONDS(3)) != 0) {
			bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
			if (++retries % 5 == 0) {
				LOG_INF("Still scanning, no beacon found (%ds)",
					retries * 3);
			}
		}

		struct beacon_state *beacon = alloc_beacon(conn);

		if (!beacon) {
			LOG_ERR("No beacon slot available");
			bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(conn);
			continue;
		}

		/* Purge any residual give from a previous pairing */
		k_sem_reset(&sem_secured);
		security_failed = false;

		err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			LOG_ERR("Security setup failed (err %d)", err);
			if (err == -ENOMEM) {
				/* Key pool (CONFIG_BT_MAX_PAIRED) saturated by
				 * accumulated stale bonds: global purge. The
				 * already-encrypted links are not cut. */
				bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
				LOG_INF("Key pool purged");
			}
			goto fail;
		}

		if (k_sem_take(&sem_secured, K_SECONDS(10))) {
			LOG_ERR("Timeout waiting for security");
			goto fail;
		}
		if (security_failed) {
			goto fail;
		}

		/* Channel Sounding setup: RAS discovery, capabilities, config,
		 * CS security. Blocking, sequential (one beacon at a time). */
		err = cs_setup_beacon(beacon);
		if (err) {
			LOG_ERR("CS setup failed (err %d)", err);
			goto fail;
		}

		/* Inter-link anti-collision: two regimes, see CS_ACL_DESYNC_STEP
		 * in cs_config.h.
		 *  - step 0: same interval everywhere, the SDC spaces the ACL
		 *    anchors by CENTRAL_ACL_EVENT_SPACING (15 ms, prj.conf) →
		 *    deterministic grid without collision.
		 *  - step > 0: distinct intervals → phase precession, transient
		 *    collisions instead of stuck ones (old regime, fallback if
		 *    0x3 aborts persist). */
		if (CS_ACL_DESYNC_STEP > 0) {
			int idx = beacon - beacons;
			struct bt_le_conn_param cp = {
				.interval_min = cs_cp_active.interval_min + CS_ACL_DESYNC_STEP * idx,
				.interval_max = cs_cp_active.interval_max + CS_ACL_DESYNC_STEP * idx,
				.latency      = cs_cp_active.latency,
				.timeout      = cs_cp_active.timeout,
			};
			err = bt_conn_le_param_update(conn, &cp);
			if (err) {
				LOG_WRN("Conn param update failed (err %d)", err);
				/* Non-fatal: continue with the initial interval */
			}
		}
		/* step 0: nothing to do, the initial connection interval
		 * (scan_init → cs_cp_active) is already the right one. */

		count = count_active_beacons();
		LOG_INF("Beacon connected and secured (%d/%d)", count, MAX_BEACONS);

		if (count >= MAX_BEACONS) {
			bt_scan_stop();
			k_sem_give(&sem_all_connected);
		}
		continue;

fail:
		/* Release-once: if disconnected_cb already freed the slot (link
		 * dropped during security/CS setup), free_beacon returns false
		 * here and we do NOT do a second unref — this was the cause of
		 * "Found valid connection in disconnected state". */
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (free_beacon(conn)) {
			bt_conn_unref(conn);
		}
		/* Backoff: let the disconnection finish before rescanning,
		 * otherwise the scan immediately finds the same beacon again and
		 * bt_conn_le_create fails on the object still in
		 * "disconnecting state". */
		k_sleep(K_MSEC(200));
	}
}