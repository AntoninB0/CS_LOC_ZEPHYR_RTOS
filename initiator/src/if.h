#ifndef IF_H_
#define IF_H_

#include <stddef.h>
#include <stdint.h>

typedef void (*if_line_cb_t)(const char *line);

int  if_initialisation(void);
void if_send(uint8_t *buffer, size_t len);
/* Enables line-by-line UART reception ('\n' or '\r') and registers the
 * handler, called in workqueue context (mutex/log allowed). */
void if_set_line_cb(if_line_cb_t cb);

#endif /* IF_H_ */
