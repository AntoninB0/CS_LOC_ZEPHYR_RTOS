#include "ble_protocol_data.h"
#include "if.h"
#include <string.h>
#include <zephyr/logging/log.h>
#define MAX_NUM_IQ_SAMPLES 148

LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#define MAX_PACKET_SIZE (2 + 15 * MAX_NUM_IQ_SAMPLES + 2)  /* 15 octets/sample : beacon_idx + channel + antenna + 4xIQ + float */

static uint8_t buffer[MAX_PACKET_SIZE];

void ble_protocol_data_send_batch(const struct iq_sample_packet *samples,
                                  uint16_t count) {
    if (count == 0) {
        return;
    }

    /* Avec la config CS validée (mode 2+1, chmap repetition 5), une
     * procédure peut produire plus de MAX_NUM_IQ_SAMPLES échantillons :
     * on découpe en plusieurs trames au lieu de jeter le lot entier
     * (l'ancien code faisait `return` silencieusement). */
    while (count > MAX_NUM_IQ_SAMPLES) {
        ble_protocol_data_send_batch(samples, MAX_NUM_IQ_SAMPLES);
        samples += MAX_NUM_IQ_SAMPLES;
        count -= MAX_NUM_IQ_SAMPLES;
    }

    uint16_t offset = 0;

    // Header: 'R~'
    const uint8_t header[] = {0x52, 0x7E};
    memcpy(&buffer[offset], header, sizeof(header));
    offset += sizeof(header);

    // IQ Sample Payload
    for (uint16_t i = 0; i < count; i++) {
        const struct iq_sample_packet *s = &samples[i];

        memcpy(&buffer[offset], &s->beacon_idx, sizeof(s->beacon_idx));
        offset += sizeof(s->beacon_idx);
        memcpy(&buffer[offset], &s->channel, sizeof(s->channel));
        offset += sizeof(s->channel);
        memcpy(&buffer[offset], &s->antenna_index, sizeof(s->antenna_index));
        offset += sizeof(s->antenna_index);

        memcpy(&buffer[offset], &s->local_i, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy(&buffer[offset], &s->local_q, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy(&buffer[offset], &s->peer_i, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy(&buffer[offset], &s->peer_q, sizeof(int16_t));
        offset += sizeof(int16_t);

        // New: Serialize phase_distance_m
        memcpy(&buffer[offset], &s->phase_distance_m, sizeof(float));
        offset += sizeof(float);
    }

    // Footer: '~E'
    const uint8_t footer[] = {0x7E, 0x45};
    memcpy(&buffer[offset], footer, sizeof(footer));
    offset += sizeof(footer);

    LOG_INF("Serialisation done, buffer length = %d", offset);
    if_send(buffer, offset);

}