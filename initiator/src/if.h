#ifndef IF_H_
#define IF_H_

#include <stddef.h>
#include <stdint.h>

typedef void (*if_line_cb_t)(const char *line);

int  if_initialisation(void);
void if_send(uint8_t *buffer, size_t len);
/* Active la réception UART ligne à ligne ('\n' ou '\r') et enregistre le
 * handler, appelé en contexte workqueue (mutex/log autorisés). */
void if_set_line_cb(if_line_cb_t cb);

#endif /* IF_H_ */
