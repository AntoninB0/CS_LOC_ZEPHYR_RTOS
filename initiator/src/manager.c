#include "manager.h"
#include "cs_ranging.h"   /* cs_iq_dump */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>            /* BIT() */
#include <zephyr/bluetooth/cs.h>        /* bt_le_cs_set_valid_chmap_bits */
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

/* ── Whitelist par ADRESSE ───────────────────────────────────────────────────
 * Filtre applicatif : à la connexion, connected_cb (main.c) rejette toute
 * adresse hors liste quand la whitelist est active. Les réflecteurs sont en
 * RANDOM STATIC (adresse stable) → l'adresse est un identifiant valable.
 * (RPA tournante = il faudrait du bonding/IRK, hors cas d'usage actuel.)
 * L'état est modifiable à chaud par le "manager" externe via l'UART. */
#define WL_MAX 8
static bt_addr_le_t wl[WL_MAX];
static int          wl_count;
static bool         wl_enabled;   /* false = pas de filtrage (tout autorisé) */

/* ── Channel map demandée par le manager (préparation hot-swap) ─────────────*/
static uint8_t cur_map[10];
static bool    has_map;           /* une map a été fixée via CHMAP:* */

/* ── Mutex : les commandes UART (workqueue) et les lectures (thread BT /
 * mesure) touchent whitelist et map. */
static K_MUTEX_DEFINE(mgr_lock);

void manager_init(void)
{
	k_mutex_init(&mgr_lock);
	wl_count   = 0;
	wl_enabled = false;
	has_map    = false;
}

bool manager_addr_allowed(const bt_addr_le_t *addr)
{
	bool ok = true;

	k_mutex_lock(&mgr_lock, K_FOREVER);
	if (wl_enabled) {
		ok = false;
		for (int i = 0; i < wl_count; i++) {
			if (bt_addr_le_cmp(&wl[i], addr) == 0) {
				ok = true;
				break;
			}
		}
	}
	k_mutex_unlock(&mgr_lock);
	return ok;
}

bool manager_get_chmap(uint8_t channel_map[10])
{
	bool set;

	k_mutex_lock(&mgr_lock, K_FOREVER);
	set = has_map;
	if (set) {
		memcpy(channel_map, cur_map, 10);
	}
	k_mutex_unlock(&mgr_lock);
	return set;
}

/* ── Commandes ──────────────────────────────────────────────────────────── */

static void wl_add(const char *addr_s, const char *type_s)
{
	bt_addr_le_t a;

	if (bt_addr_le_from_str(addr_s, type_s, &a)) {
		LOG_WRN("mgr: adresse invalide '%s %s'", addr_s, type_s);
		return;
	}
	k_mutex_lock(&mgr_lock, K_FOREVER);
	for (int i = 0; i < wl_count; i++) {
		if (bt_addr_le_cmp(&wl[i], &a) == 0) {   /* déjà présente */
			k_mutex_unlock(&mgr_lock);
			return;
		}
	}
	if (wl_count >= WL_MAX) {
		k_mutex_unlock(&mgr_lock);
		LOG_WRN("mgr: whitelist pleine (%d)", WL_MAX);
		return;
	}
	wl[wl_count++] = a;
	int n = wl_count;
	k_mutex_unlock(&mgr_lock);
	LOG_INF("mgr: whitelist += %s (%s), %d entree(s)", addr_s, type_s, n);
}

static void wl_del(const char *addr_s, const char *type_s)
{
	bt_addr_le_t a;

	if (bt_addr_le_from_str(addr_s, type_s, &a)) {
		return;
	}
	k_mutex_lock(&mgr_lock, K_FOREVER);
	for (int i = 0; i < wl_count; i++) {
		if (bt_addr_le_cmp(&wl[i], &a) == 0) {
			wl[i] = wl[--wl_count];    /* swap-remove */
			k_mutex_unlock(&mgr_lock);
			LOG_INF("mgr: whitelist -= %s", addr_s);
			return;
		}
	}
	k_mutex_unlock(&mgr_lock);
}

/* Prépare une channel map décimée 1/n (n=1 → pleine). Prend effet à la
 * prochaine création de config CS (reconnexion). Pour un hot-swap LIVE, il
 * faut re-créer la config en cours de connexion (redémarrage procédure). */
static void chmap_thin(int n)
{
	uint8_t map[10];

	if (n < 1 || n > 4) {
		LOG_WRN("mgr: CHMAP thinning 1..4");
		return;
	}
	bt_le_cs_set_valid_chmap_bits(map);
	if (n > 1) {
		int count = 0;

		for (int ch = 0; ch < 79; ch++) {
			if (!(map[ch / 8] & BIT(ch % 8))) {
				continue;
			}
			if (count % n != 0) {
				map[ch / 8] &= ~BIT(ch % 8);
			}
			count++;
		}
	}
	k_mutex_lock(&mgr_lock, K_FOREVER);
	memcpy(cur_map, map, 10);
	has_map = true;
	k_mutex_unlock(&mgr_lock);
	LOG_INF("mgr: chmap 1/%d preparee (effet a la prochaine (re)config CS)", n);
}

void manager_uart_line(const char *line)
{
	if (!strncmp(line, "IQON", 4)) {
		cs_iq_dump = true;
		LOG_INF("mgr: IQ dump ON");
		return;
	}
	if (!strncmp(line, "IQOFF", 5)) {
		cs_iq_dump = false;
		LOG_INF("mgr: IQ dump OFF");
		return;
	}

	if (!strncmp(line, "WL:ON", 5)) {
		wl_enabled = true;
		LOG_INF("mgr: whitelist ON");
		return;
	}
	if (!strncmp(line, "WL:OFF", 6)) {
		wl_enabled = false;
		LOG_INF("mgr: whitelist OFF");
		return;
	}
	if (!strncmp(line, "WL:CLR", 6)) {
		k_mutex_lock(&mgr_lock, K_FOREVER);
		wl_count = 0;
		k_mutex_unlock(&mgr_lock);
		LOG_INF("mgr: whitelist videe");
		return;
	}
	if (!strncmp(line, "WL:LIST", 7)) {
		char s[BT_ADDR_LE_STR_LEN];

		k_mutex_lock(&mgr_lock, K_FOREVER);
		LOG_INF("mgr: whitelist (%s, %d):", wl_enabled ? "ON" : "OFF", wl_count);
		for (int i = 0; i < wl_count; i++) {
			bt_addr_le_to_str(&wl[i], s, sizeof(s));
			LOG_INF("  [%d] %s", i, s);
		}
		k_mutex_unlock(&mgr_lock);
		return;
	}
	if (!strncmp(line, "WL:ADD ", 7) || !strncmp(line, "WL:DEL ", 7)) {
		char buf[48];

		strncpy(buf, line + 7, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = '\0';
		char *addr_s = strtok(buf, " ");
		char *type_s = strtok(NULL, " \r\n");

		if (!addr_s) {
			LOG_WRN("mgr: usage WL:ADD <addr> [public|random]");
			return;
		}
		if (!type_s) {
			type_s = "random";
		}
		if (line[3] == 'A') {
			wl_add(addr_s, type_s);
		} else {
			wl_del(addr_s, type_s);
		}
		return;
	}

	if (!strncmp(line, "CHMAP:THIN ", 11)) {
		chmap_thin(atoi(line + 11));
		return;
	}
	if (!strncmp(line, "CHMAP:FULL", 10)) {
		chmap_thin(1);
		return;
	}

	if (!strncmp(line, "HELP", 4)) {
		LOG_INF("cmds: IQON IQOFF | WL:ON WL:OFF WL:CLR WL:LIST | "
			"WL:ADD/DEL <addr> [public|random] | CHMAP:THIN <n> | CHMAP:FULL");
		return;
	}
	LOG_WRN("mgr: commande inconnue: %s", line);
}
