#ifndef BEACON_H
#define BEACON_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>

#include "cs_config.h"

/* Sized by the application profile (cs_config.h) */
#define MAX_BEACONS CS_MAX_BEACONS

struct beacon_state {
	struct bt_conn *conn;
	bt_addr_le_t   addr;
	bool           active;
	/* Channel Sounding */
	bool           cs_ready;     /* CS setup done, ranging possible */
	uint8_t        cs_config_id; /* CS config id returned by the controller */
};

extern struct beacon_state beacons[MAX_BEACONS];
extern struct k_mutex      beacons_mutex;
extern struct k_msgq       new_conn_queue;

struct beacon_state *find_beacon(struct bt_conn *conn);
struct beacon_state *alloc_beacon(struct bt_conn *conn);
/* Frees the slot associated with conn. Returns true ONLY for the caller that
 * actually freed the slot: it is the one that must call bt_conn_unref.
 * Guarantees exactly one unref even if disconnected_cb and the pairing error
 * path run concurrently. */
bool                 free_beacon(struct bt_conn *conn);
void                 decalage(void);

#endif /* BEACON_H */
