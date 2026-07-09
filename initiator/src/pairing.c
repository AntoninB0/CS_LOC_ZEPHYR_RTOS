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
/* Donné quand MAX_BEACONS réflecteurs sont connectés + sécurisés */
static K_SEM_DEFINE(sem_all_connected, 0, 1);

static volatile bool security_failed;

void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed (err %d)", err);

		if (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING) {
			/* Bond périmé : le réflecteur a redémarré et perdu ses
			 * clés (RAM only, pas de CONFIG_BT_SETTINGS), mais nous
			 * gardons sa vieille LTK et tentons de chiffrer avec au
			 * lieu de re-pairer. On purge le bond : la prochaine
			 * tentative refera un pairing complet. */
			int uerr = bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));

			if (uerr) {
				LOG_WRN("Unpair failed (err %d)", uerr);
			} else {
				LOG_INF("Stale bond removed, will re-pair");
			}
		}

		/* Fast-fail : réveiller le thread de pairing immédiatement au
		 * lieu de consommer les 10 s du timeout */
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

	/* k_sem_reset() côté pairing réveille les waiters avec -EAGAIN :
	 * on ré-arme l'attente dans ce cas au lieu de retourner. */
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
			/* Quota atteint : on arrête le scan et on signale main() */
			if (!was_full) {
				bt_scan_stop();
				k_sem_give(&sem_all_connected);
				was_full = true;
			}
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Quota non atteint. Le reset n'est fait QUE sur la transition
		 * plein → non-plein (perte d'un beacon) : un reset à chaque
		 * itération réveillait main() avec -EAGAIN dès le boot. */
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

		/* Attente bloquante d'une nouvelle connexion */
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

		/* Purge un éventuel give résiduel d'un pairing précédent */
		k_sem_reset(&sem_secured);
		security_failed = false;

		err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			LOG_ERR("Security setup failed (err %d)", err);
			if (err == -ENOMEM) {
				/* Pool de clés (CONFIG_BT_MAX_PAIRED) saturé par
				 * des bonds périmés accumulés : purge globale.
				 * Les liens déjà chiffrés ne sont pas coupés. */
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

		/* Setup Channel Sounding : découverte RAS, capacités, config,
		 * sécurité CS. Bloquant, séquentiel (un beacon à la fois). */
		err = cs_setup_beacon(beacon);
		if (err) {
			LOG_ERR("CS setup failed (err %d)", err);
			goto fail;
		}

		/* Anti-collision inter-liens : deux régimes, voir
		 * CS_ACL_DESYNC_STEP dans cs_config.h.
		 *  - step 0 : même intervalle partout, le SDC espace les ancres
		 *    ACL de CENTRAL_ACL_EVENT_SPACING (15 ms, prj.conf) →
		 *    quadrillage déterministe sans collision.
		 *  - step > 0 : intervalles distincts → précession des phases,
		 *    collisions transitoires au lieu de figées (ancien régime,
		 *    fallback si des aborts 0x3 persistent). */
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
				/* Non fatal : on continue avec l'intervalle initial */
			}
		}
		/* step 0 : rien à faire, l'intervalle de connexion initial
		 * (scan_init → cs_cp_active) est déjà le bon. */

		count = count_active_beacons();
		LOG_INF("Beacon connected and secured (%d/%d)", count, MAX_BEACONS);

		if (count >= MAX_BEACONS) {
			bt_scan_stop();
			k_sem_give(&sem_all_connected);
		}
		continue;

fail:
		/* Release-once : si disconnected_cb a déjà libéré le slot (lien
		 * tombé pendant security/setup CS), free_beacon retourne false
		 * ici et on ne fait PAS de second unref — c'était la cause du
		 * "Found valid connection in disconnected state". */
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (free_beacon(conn)) {
			bt_conn_unref(conn);
		}
		/* Backoff : laisse la déconnexion se terminer avant de
		 * rescanner, sinon le scan retrouve immédiatement le même
		 * beacon et bt_conn_le_create échoue sur l'objet encore en
		 * "disconnecting state". */
		k_sleep(K_MSEC(200));
	}
}