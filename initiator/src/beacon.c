#include "beacon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

struct beacon_state beacons[MAX_BEACONS];
K_MUTEX_DEFINE(beacons_mutex);
K_MSGQ_DEFINE(new_conn_queue, sizeof(struct bt_conn *), MAX_BEACONS, sizeof(void *));

struct beacon_state *find_beacon(struct bt_conn *conn)
{
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].active && beacons[i].conn == conn) {
			return &beacons[i];
		}
	}
	return NULL;
}

struct beacon_state *alloc_beacon(struct bt_conn *conn)
{
	k_mutex_lock(&beacons_mutex, K_FOREVER);
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (!beacons[i].active) {
			beacons[i].conn     = conn;
			beacons[i].active   = true;
			beacons[i].cs_ready = false;
			bt_addr_le_copy(&beacons[i].addr, bt_conn_get_dst(conn));
			k_mutex_unlock(&beacons_mutex);
			return &beacons[i];
		}
	}
	k_mutex_unlock(&beacons_mutex);
	return NULL;
}

bool free_beacon(struct bt_conn *conn)
{
	bool released = false;

	if (conn == NULL) {
		return false;
	}

	k_mutex_lock(&beacons_mutex, K_FOREVER);
	for (int i = 0; i < MAX_BEACONS; i++) {
		if (beacons[i].conn == conn) {
			beacons[i].active        = false;
			beacons[i].conn          = NULL;
			beacons[i].cs_ready      = false;
			beacons[i].last_distance = -1.0f;
			released = true;
			break;
		}
	}
	k_mutex_unlock(&beacons_mutex);

	return released;
}

void decalage(void)
{
	k_mutex_lock(&beacons_mutex, K_FOREVER);
	for (int left = 0; left < MAX_BEACONS; left++) {
		if (beacons[left].active) {
			continue;
		}
		/* slot libre trouvé : cherche le prochain actif à droite */
		for (int right = left + 1; right < MAX_BEACONS; right++) {
			if (beacons[right].active) {
				beacons[left]         = beacons[right];
				beacons[right].active = false;
				beacons[right].conn   = NULL;
				break;
			}
		}
	}
	k_mutex_unlock(&beacons_mutex);
}
