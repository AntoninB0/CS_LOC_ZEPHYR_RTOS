#include "if.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

/* 1 = uart30 (VCOM Serial Port 0, bench PC via USB — cs_console.py)
 * 0 = uart21 (P1.08/P1.09, extension header — drone companion in flight)
 * See the full mapping in boards/nrf54l15dk_nrf54l15_cpuapp.overlay. */
#define IF_UART_ON_VCOM 1

#if IF_UART_ON_VCOM
#define UART_NODE DT_NODELABEL(uart30)
#else
#define UART_NODE DT_NODELABEL(uart21)
#endif

static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

/* ── RX: line commands (scheduler) ───────────────────────────────────────────
 * The ISR accumulates until '\n' then delegates parsing to the system
 * workqueue (thread context: the user handler may take mutexes/log). A line
 * arriving while the previous one is being processed overwrites it — of no
 * consequence for occasional configuration commands. */
#define IF_LINE_MAX 64

static if_line_cb_t line_cb;
static char         rx_acc[IF_LINE_MAX];
static size_t       rx_len;
static char         pending_line[IF_LINE_MAX];
static struct k_work line_work;

static void line_work_handler(struct k_work *work)
{
	if (line_cb) {
		line_cb(pending_line);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t c;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		while (uart_fifo_read(dev, &c, 1) == 1) {
			if (c == '\n' || c == '\r') {
				if (rx_len > 0) {
					memcpy(pending_line, rx_acc, rx_len);
					pending_line[rx_len] = '\0';
					rx_len = 0;
					k_work_submit(&line_work);
				}
			} else if (rx_len < IF_LINE_MAX - 1) {
				rx_acc[rx_len++] = (char)c;
			} else {
				rx_len = 0; /* line too long: discard */
			}
		}
	}
}

int if_initialisation(void)
{
	if (!device_is_ready(uart_dev)) {
		return -1;
	}
	return 0;
}

void if_set_line_cb(if_line_cb_t cb)
{
	line_cb = cb;
	k_work_init(&line_work, line_work_handler);
	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);
}

void if_send(uint8_t *buffer, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buffer[i]);
	}
}
