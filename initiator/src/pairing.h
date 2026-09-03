#ifndef PAIRING_H
#define PAIRING_H

#include <zephyr/kernel.h>

#define PAIRING_STACK_SIZE 2048
#define PAIRING_PRIORITY   5

extern struct k_thread  pairing_thread_data;
extern k_thread_stack_t pairing_stack[];

void pairing_n_bluetooth(void);

/* Blocks until MAX_BEACONS reflectors are connected and secured.
 * Returns 0 on success, -EAGAIN on timeout. */
int pairing_wait_all_connected(k_timeout_t timeout);

#endif /* PAIRING_H */
