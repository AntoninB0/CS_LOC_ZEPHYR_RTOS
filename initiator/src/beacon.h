#ifndef BEACON_H
#define BEACON_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>

#include "cs_config.h"

/* Dimensionné par le profil applicatif (cs_config.h) */
#define MAX_BEACONS CS_MAX_BEACONS

struct beacon_state {
	struct bt_conn *conn;
	bt_addr_le_t   addr;
	bool           active;
	/* Channel Sounding */
	bool           cs_ready;     /* setup CS terminé, ranging possible */
	uint8_t        cs_config_id; /* id de config CS retourné par le contrôleur */
};

extern struct beacon_state beacons[MAX_BEACONS];
extern struct k_mutex      beacons_mutex;
extern struct k_msgq       new_conn_queue;

struct beacon_state *find_beacon(struct bt_conn *conn);
struct beacon_state *alloc_beacon(struct bt_conn *conn);
/* Libère le slot associé à conn. Retourne true UNIQUEMENT pour l'appelant
 * qui a réellement libéré le slot : c'est lui qui doit faire bt_conn_unref.
 * Garantit exactement un unref même si disconnected_cb et le chemin
 * d'erreur du pairing s'exécutent en concurrence. */
bool                 free_beacon(struct bt_conn *conn);
void                 decalage(void);

#endif /* BEACON_H */
