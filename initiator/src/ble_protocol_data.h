#ifndef BLE_PROTOCOL_DATA_H_
#define BLE_PROTOCOL_DATA_H_

#include <stdint.h>

// #define MAX_NUM_IQ_SAMPLES (256 * CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

struct iq_sample_packet {
	uint8_t beacon_idx;   /* identifie le réflecteur source de la mesure */
	uint8_t channel;
	uint8_t antenna_index;
	int16_t local_i;
	int16_t local_q;
	int16_t peer_i;
	int16_t peer_q;
	float phase_distance_m;
};

void ble_protocol_data_send_batch(const struct iq_sample_packet *samples, uint16_t count);

#endif /* BLE_PROTOCOL_DATA_H_ */