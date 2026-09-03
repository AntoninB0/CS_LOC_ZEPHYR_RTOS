/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/ras.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1

/* Ranging Service UUID 0x185B — filter used by the initiator */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── Advertising (re)start via workqueue ────────────────────────────────
 * NEVER call bt_le_adv_start() directly in disconnected_cb: the connection
 * object is not freed yet at that instant, and with CONFIG_BT_MAX_CONN=1
 * there is no free slot → -ENOMEM → the reflector never re-advertises
 * again. So we defer to the system workqueue, retrying until the slot is
 * released. */
static void adv_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_handler);

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err == -EALREADY) {
		return;
	}
	if (err) {
		LOG_WRN("Adv start failed (err %d), retry in 500 ms", err);
		k_work_schedule(&adv_work, K_MSEC(500));
		return;
	}
	LOG_INF("Advertising...");
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		/* A failed connection also stops advertising: restart it */
		k_work_schedule(&adv_work, K_MSEC(100));
		return;
	}

	dk_set_led_on(CON_STATUS_LED);

	/* The RAS RRSP instance is allocated AUTOMATICALLY by the service on
	 * connection (CONFIG_BT_RAS_RRSP_AUTO_ALLOC_INSTANCE=y, NCS 3.x default).
	 * An explicit bt_ras_rrsp_alloc() here would return an error (instance
	 * already taken) — this was the cause of the 0x13 disconnections that
	 * aborted the pairing (Security failed err 9 on the initiator side). */

	/* Channel Sounding role: reflector only.
	 * The responses to the CS procedures (mode 0/2) and to CS security are
	 * then handled automatically by the controller. */
	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role     = false,
		.enable_reflector_role     = true,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power              = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	int ret = bt_le_cs_set_default_settings(conn, &default_settings);

	if (ret) {
		LOG_ERR("CS default settings failed (err %d)", ret);
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02X)", reason);
	dk_set_led_off(CON_STATUS_LED);

	/* RAS instance freed automatically (AUTO_ALLOC_INSTANCE) */

	/* Deferred restart: the connection slot is only released after this
	 * callback returns. 100 ms + retry covers all cases. */
	k_work_schedule(&adv_work, K_MSEC(100));
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed (err %d)", err);
		return;
	}
	LOG_INF("Security level %d", level);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected        = connected_cb,
	.disconnected     = disconnected_cb,
	.security_changed = security_changed_cb,
};

int main(void)
{
	int err;

	LOG_INF("Starting CS reflector");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	k_work_schedule(&adv_work, K_NO_WAIT);
	return 0;
}