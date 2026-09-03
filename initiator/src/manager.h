#ifndef MANAGER_H
#define MANAGER_H

#include <zephyr/bluetooth/addr.h>
#include <stdbool.h>
#include <stdint.h>

/* ── Application manager (replaces the scheduler) ────────────────────────────
 * Receives the UART commands (workqueue context: mutex/log OK), drives the IQ
 * dump, manages a hot-editable whitelist of reflector addresses, and stages the
 * channel-map hot-swap.
 *
 * UART protocol (ASCII lines, data uart):
 *   IQON / IQOFF                 enable / disable the IQ dump (IQL/IQP)
 *   WL:ON / WL:OFF               enable / disable whitelist filtering
 *   WL:ADD <addr> [public|random]  add an address (random type by default)
 *   WL:DEL <addr> [public|random]  remove an address
 *   WL:CLR                       clear the whitelist
 *   WL:LIST                      log the whitelist
 *   CHMAP:THIN <n>               stage a 1/n decimated channel map (1..4)
 *   CHMAP:FULL                   stage the full channel map
 *   HELP                         list the commands
 * Any unknown line is ignored (WRN log). */
void manager_init(void);

/* UART line handler, to pass to if_set_line_cb(). */
void manager_uart_line(const char *line);

/* Whitelist: true if the connection to `addr` is allowed. Always true if the
 * whitelist is disabled (WL:OFF). Called from connected_cb (main.c). */
bool manager_addr_allowed(const bt_addr_le_t *addr);

/* Channel map requested over UART (CHMAP:*). Returns true and copies 10 bytes
 * if a map was set by the manager, false otherwise (→ the profile's
 * compile-time map). Used by cs_build_channel_map at config creation.
 * ⚠ A change only takes effect at the next CS config (re)creation, i.e. at a
 * beacon (re)connection. A LIVE hot-swap (without reconnection) requires
 * disable → create_config(new map) → re-enable = procedure restart — to be
 * validated on the bench (see the note in manager.c). */
bool manager_get_chmap(uint8_t channel_map[10]);

#endif /* MANAGER_H */
