#ifndef CS_RANGING_H
#define CS_RANGING_H

#include <zephyr/kernel.h>
#include "beacon.h"

/* Full Channel Sounding setup for a freshly secured (L2) beacon: RAS
 * discovery, capabilities, default settings, CS config, CS security, procedure
 * parameters. BLOCKING (internal semaphores).
 * Call from the pairing thread, sequentially.
 * Returns 0 if the beacon is ready for ranging. */
int cs_setup_beacon(struct beacon_state *beacon);

/* Init of the per-beacon measurement states. Call once before the loop. */
void cs_ranging_init(void);

/* FREE-RUNNING (max_procedure_count = 0: the controller repeats the procedures
 * every procedure_interval, the ~495 ms LL handshake is paid only at arming):
 *  - cs_enable_beacon(i)  : ARMS beacon i's procedures, ONCE (and after a
 *                           reconnection). Returns 0 if accepted.
 *  - cs_collect_beacon(i) : waits for the next paired measurement (local steps
 *                           + peer ranging data of the same counter) and emits
 *                           the timestamped IQ dump (IQL/IQP). Returns 0 if a
 *                           measurement was emitted, < 0 otherwise. *rearmed
 *                           goes false if the beacon must be re-armed (on-demand
 *                           path only). NO distance computation (done off-board).
 * Collection consumes no airtime: the N links' procedures run in the
 * background, the per-beacon rate = procedure_interval x connection interval
 * (measurements not collected in time: overwritten, not accumulated). */
/* Timestamped IQ dump to the data UART (ON by default, disabled by the UART
 * command IQOFF / re-enabled by IQON, line format documented in GUIDE.md). */
extern bool cs_iq_dump;

int cs_enable_beacon(int i);
int cs_collect_beacon(int i, bool *rearmed);

#endif /* CS_RANGING_H */
