#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>
#include <zephyr/bluetooth/hci_types.h>
#include "ble_protocol_data.h"
#include "beacon.h"
#include <math.h>

#define MAX_NUM_IQ_SAMPLES (256 * CONFIG_BT_RAS_MAX_ANTENNA_PATHS)
#define DISTANCE_FILTER_LEN 8

#define CS_FREQUENCY_MHZ(ch)   (2402u + 1u * (ch))
#define SPEED_OF_LIGHT_M_PER_S 299792458.0f
#define PI                     3.14159265358979323846f

struct iq_sample_and_channel {
	bool failed;
	uint8_t channel;
	uint8_t antenna_permutation;
	struct bt_le_cs_iq_sample local_iq_sample;
	struct bt_le_cs_iq_sample peer_iq_sample;
};

static struct iq_sample_and_channel iq_sample_channel_data[MAX_NUM_IQ_SAMPLES];

/* Un filtre glissant PAR beacon : un filtre global mélangerait les
 * distances mesurées vers des réflecteurs différents. */
static float distance_filter[MAX_BEACONS][DISTANCE_FILTER_LEN];
static uint8_t distance_filter_idx[MAX_BEACONS];
static uint8_t distance_filter_count[MAX_BEACONS];

static float filtered_distance(uint8_t beacon_idx, float new_sample)
{
	if (beacon_idx >= MAX_BEACONS) {
		return -1.0f;
	}
	if (new_sample <= 0.0f || new_sample > 50.0f) {
		return -1.0f;
	}
	distance_filter[beacon_idx][distance_filter_idx[beacon_idx]] = new_sample;
	distance_filter_idx[beacon_idx] =
		(distance_filter_idx[beacon_idx] + 1) % DISTANCE_FILTER_LEN;
	if (distance_filter_count[beacon_idx] < DISTANCE_FILTER_LEN) {
		distance_filter_count[beacon_idx]++;
	}
	float sum = 0.0f;
	for (uint8_t i = 0; i < distance_filter_count[beacon_idx]; i++) {
		sum += distance_filter[beacon_idx][i];
	}
	return sum / distance_filter_count[beacon_idx];
}

struct processing_context {
	uint16_t iq_sample_channel_data_index;
	uint8_t n_ap;
	enum bt_conn_le_cs_role role;
};

static void process_tone_info_data(struct processing_context *context,
			      struct bt_hci_le_cs_step_data_tone_info local_tone_info[],
			      struct bt_hci_le_cs_step_data_tone_info peer_tone_info[],
			      uint8_t channel, uint8_t antenna_permutation_index)
{
	for (uint8_t i = 0; i < (context->n_ap + 1); i++) {
		if (local_tone_info[i].extension_indicator != BT_HCI_LE_CS_NOT_TONE_EXT_SLOT ||
		    peer_tone_info[i].extension_indicator != BT_HCI_LE_CS_NOT_TONE_EXT_SLOT) {
			continue;
		}

		if (context->iq_sample_channel_data_index >= MAX_NUM_IQ_SAMPLES) {
			LOG_WRN("Too many IQ samples");
			return;
		}

		struct iq_sample_and_channel *entry =
			&iq_sample_channel_data[context->iq_sample_channel_data_index];

		entry->channel = channel;
		entry->antenna_permutation = antenna_permutation_index;
		entry->local_iq_sample =
			bt_le_cs_parse_pct(local_tone_info[i].phase_correction_term);
		entry->peer_iq_sample =
			bt_le_cs_parse_pct(peer_tone_info[i].phase_correction_term);

		entry->failed =
			(local_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
			 local_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE ||
			 peer_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
			 peer_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE);

		context->iq_sample_channel_data_index++;
	}
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	struct processing_context *context = (struct processing_context *)user_data;

	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2 ||
	    local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {

		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		process_tone_info_data(context, local_step_data->tone_info,
				       peer_step_data->tone_info, local_step->channel,
				       local_step_data->antenna_permutation_index);
	}

	return true;
}

static void calc_complex_product(int32_t z_a_real, int32_t z_a_imag, int32_t z_b_real,
				 int32_t z_b_imag, int32_t *z_out_real, int32_t *z_out_imag)
{
	*z_out_real = z_a_real * z_b_real - z_a_imag * z_b_imag;
	*z_out_imag = z_a_real * z_b_imag + z_a_imag * z_b_real;
}

static float linear_regression(float *x_values, float *y_values, uint16_t n_samples)
{
	if (n_samples == 0) {
		return 0.0f;
	}

	float y_mean = 0.0f;
	float x_mean = 0.0f;

	for (uint16_t i = 0; i < n_samples; i++) {
		y_mean += (y_values[i] - y_mean) / (i + 1);
		x_mean += (x_values[i] - x_mean) / (i + 1);
	}

	float b_est_upper = 0.0f;
	float b_est_lower = 0.0f;

	for (uint16_t i = 0; i < n_samples; i++) {
		b_est_upper += (x_values[i] - x_mean) * (y_values[i] - y_mean);
		b_est_lower += (x_values[i] - x_mean) * (x_values[i] - x_mean);
	}

	return b_est_upper / b_est_lower;
}

static void bubblesort_2(float *array1, float *array2, uint16_t len)
{
	for (uint16_t i = 0; i < len - 1; i++) {
		for (uint16_t j = 0; j < len - i - 1; j++) {
			if (array1[j] > array1[j + 1]) {
				float temp1 = array1[j];
				array1[j] = array1[j + 1];
				array1[j + 1] = temp1;

				float temp2 = array2[j];
				array2[j] = array2[j + 1];
				array2[j + 1] = temp2;
			}
		}
	}
}

static float estimate_distance_using_phase_slope(struct iq_sample_and_channel *data, uint16_t len)
{
	int32_t combined_i, combined_q;
	uint16_t num_angles = 0;
	static float theta[MAX_NUM_IQ_SAMPLES];
	static float frequencies[MAX_NUM_IQ_SAMPLES];

	for (uint16_t i = 0; i < len; i++) {
		if (!data[i].failed) {
			calc_complex_product(data[i].local_iq_sample.i, data[i].local_iq_sample.q,
					     data[i].peer_iq_sample.i, data[i].peer_iq_sample.q,
					     &combined_i, &combined_q);

			theta[num_angles] = atan2f((float)combined_q, (float)combined_i);
			frequencies[num_angles] = (float)CS_FREQUENCY_MHZ(data[i].channel);
			num_angles++;
		}
	}

	if (num_angles < 2) {
		return 0.0f;
	}

	bubblesort_2(frequencies, theta, num_angles);

	for (uint16_t i = 1; i < num_angles; i++) {
		float diff = theta[i] - theta[i - 1];
		if (diff > PI) {
			for (uint16_t j = i; j < num_angles; j++) {
				theta[j] -= 2.0f * PI;
			}
		} else if (diff < -PI) {
			for (uint16_t j = i; j < num_angles; j++) {
				theta[j] += 2.0f * PI;
			}
		}
	}

	float phase_slope = linear_regression(frequencies, theta, num_angles);
	float distance = -phase_slope * (SPEED_OF_LIGHT_M_PER_S / (4 * PI));

	return distance / 1e6f; // Convert to meters
}

float estimate_distance(struct net_buf_simple *local_steps, struct net_buf_simple *peer_steps,
			uint8_t n_ap, enum bt_conn_le_cs_role role, uint8_t beacon_idx)
{
	struct processing_context context = {
		.iq_sample_channel_data_index = 0,
		.n_ap = n_ap,
		.role = role,
	};
	static struct iq_sample_packet out_samples[MAX_NUM_IQ_SAMPLES];
	uint16_t valid_sample_count = 0;

	memset(iq_sample_channel_data, 0, sizeof(iq_sample_channel_data));

	bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, context.role, NULL,
					   NULL, process_step_data, &context);

	float distance_m = estimate_distance_using_phase_slope(iq_sample_channel_data,
		context.iq_sample_channel_data_index);

	float smoothed_m = filtered_distance(beacon_idx, distance_m);
	if (smoothed_m > 0.0f) {
		LOG_INF("Beacon[%u] distance: %.2f m (raw: %.2f m)",
			beacon_idx, (double)smoothed_m, (double)distance_m);
	} else {
		LOG_WRN("Beacon[%u] distance hors plage (raw: %.2f m)",
			beacon_idx, (double)distance_m);
	}

	for (uint16_t i = 0; i < context.iq_sample_channel_data_index; i++) {
		if (iq_sample_channel_data[i].failed) {
			continue;
		}

		out_samples[valid_sample_count].beacon_idx = beacon_idx;
		out_samples[valid_sample_count].channel = iq_sample_channel_data[i].channel;
		out_samples[valid_sample_count].antenna_index = iq_sample_channel_data[i].antenna_permutation;
		out_samples[valid_sample_count].local_i = iq_sample_channel_data[i].local_iq_sample.i;
		out_samples[valid_sample_count].local_q = iq_sample_channel_data[i].local_iq_sample.q;
		out_samples[valid_sample_count].peer_i = iq_sample_channel_data[i].peer_iq_sample.i;
		out_samples[valid_sample_count].peer_q = iq_sample_channel_data[i].peer_iq_sample.q;
		out_samples[valid_sample_count].phase_distance_m = distance_m;

		valid_sample_count++;
	}

	if (valid_sample_count > 0) {
		ble_protocol_data_send_batch(out_samples, valid_sample_count);
	}

	return smoothed_m;
}
