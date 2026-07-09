/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <zephyr/bluetooth/cs.h>

/* Calcule la distance (phase slope) à partir des steps locaux et distants,
 * applique le filtre glissant propre au beacon `beacon_idx`, et exporte les
 * échantillons IQ via ble_protocol_data.
 * Retourne la distance filtrée en mètres, ou -1.0f si hors plage/échec. */
float estimate_distance(struct net_buf_simple *local_steps, struct net_buf_simple *peer_steps,
			uint8_t n_ap, enum bt_conn_le_cs_role role, uint8_t beacon_idx);
