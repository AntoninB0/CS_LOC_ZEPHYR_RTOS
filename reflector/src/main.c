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

/* Ranging Service UUID 0x185B — filtre utilisé par l'initiateur */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── (Re)démarrage d'advertising via workqueue ──────────────────────────
 * Ne JAMAIS appeler bt_le_adv_start() directement dans disconnected_cb :
 * l'objet de connexion n'est pas encore libéré à cet instant, et avec
 * CONFIG_BT_MAX_CONN=1 il n'y a aucun slot libre → -ENOMEM → le
 * réflecteur ne ré-advertise plus jamais. On diffère donc vers la
 * workqueue système, avec retry tant que le slot n'est pas rendu. */
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
		/* Une connexion échouée stoppe aussi l'advertising : relancer */
		k_work_schedule(&adv_work, K_MSEC(100));
		return;
	}

	dk_set_led_on(CON_STATUS_LED);

	/* L'instance RAS RRSP est allouée AUTOMATIQUEMENT par le service à la
	 * connexion (CONFIG_BT_RAS_RRSP_AUTO_ALLOC_INSTANCE=y, défaut NCS 3.x).
	 * Un bt_ras_rrsp_alloc() explicite ici retournerait une erreur
	 * (instance déjà prise) — c'était la cause des déconnexions 0x13
	 * qui avortaient le pairing (Security failed err 9 côté initiateur). */

	/* Rôle Channel Sounding : réflecteur uniquement.
	 * Les réponses aux procédures CS (mode 0/2) et à la sécurité CS
	 * sont ensuite gérées automatiquement par le contrôleur. */
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

	/* Instance RAS libérée automatiquement (AUTO_ALLOC_INSTANCE) */

	/* Restart différé : le slot de connexion n'est rendu qu'après la
	 * sortie de ce callback. 100 ms + retry couvre tous les cas. */
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