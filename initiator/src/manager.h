#ifndef MANAGER_H
#define MANAGER_H

#include <zephyr/bluetooth/addr.h>
#include <stdbool.h>
#include <stdint.h>

/* ── Manager applicatif (remplace le scheduler) ─────────────────────────────
 * Reçoit les commandes UART (contexte workqueue : mutex/log OK), pilote le
 * dump IQ, gère une whitelist d'adresses de réflecteurs modifiable à chaud, et
 * prépare le hot-swap de la channel map.
 *
 * Protocole UART (lignes ASCII, uart data) :
 *   IQON / IQOFF                 active / coupe le dump IQ (IQL/IQP)
 *   WL:ON / WL:OFF               active / désactive le filtrage whitelist
 *   WL:ADD <addr> [public|random]  ajoute une adresse (type random par défaut)
 *   WL:DEL <addr> [public|random]  retire une adresse
 *   WL:CLR                       vide la whitelist
 *   WL:LIST                      logue la whitelist
 *   CHMAP:THIN <n>               prépare une channel map décimée 1/n (1..4)
 *   CHMAP:FULL                   prépare la channel map pleine
 *   HELP                         liste les commandes
 * Toute ligne inconnue est ignorée (log WRN). */
void manager_init(void);

/* Handler de ligne UART, à passer à if_set_line_cb(). */
void manager_uart_line(const char *line);

/* Whitelist : true si la connexion à `addr` est autorisée. Toujours true si la
 * whitelist est désactivée (WL:OFF). Appelé depuis connected_cb (main.c). */
bool manager_addr_allowed(const bt_addr_le_t *addr);

/* Channel map demandée par l'UART (CHMAP:*). Retourne true et copie 10 octets
 * si une map a été fixée par le manager, false sinon (→ map compile-time du
 * profil). Utilisé par cs_build_channel_map à la création de config.
 * ⚠ Un changement ne prend effet qu'à la prochaine (re)création de config CS,
 * c.-à-d. à la (re)connexion d'un beacon. Un hot-swap LIVE (sans reconnexion)
 * impose disable → create_config(nouvelle map) → re-enable = redémarrage de la
 * procédure — à valider au banc (voir note dans manager.c). */
bool manager_get_chmap(uint8_t channel_map[10]);

#endif /* MANAGER_H */
