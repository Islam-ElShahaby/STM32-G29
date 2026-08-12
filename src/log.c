/*
 * Non-blocking printf ring — see include/log.h for why this exists.
 */
#include "log.h"
#include "board.h"

/* ── printf -> ring buffer -> USART1 ────────────────────────────────────── */
/*
 * printf used to call HAL_UART_Transmit(..., HAL_MAX_DELAY) directly, which
 * BLOCKS until the last bit is on the wire. At 115200 8N1 that is ~87 us per
 * character, so the ~105-character status line costs about 9.1 ms — very
 * nearly the whole 10 ms control period, once per second. The raw-report dump
 * was worse. Control jitter that big is not acceptable once the torque loop is
 * a real task.
 *
 * So printf now only copies into this ring and returns; the console task does
 * the waiting. One change at the bottom fixes every printf in the project,
 * including the ones inside g29_hid.c.
 *
 * Overflow drops characters rather than blocking. A dropped log line is a
 * cosmetic loss; a missed control period is a real one.
 */
#define LOG_BUF_SZ 2048U          /* power of two: the mask below depends on it */

static char log_buf[LOG_BUF_SZ];
static volatile uint16_t log_head, log_tail;
volatile uint32_t log_dropped;

int _write(int file, char *ptr, int len)
{
	(void)file;

	for (int i = 0; i < len; i++) {
		uint16_t next = (uint16_t)((log_head + 1U) & (LOG_BUF_SZ - 1U));

		if (next == log_tail) {         /* full */
			log_dropped++;
			break;
		}
		log_buf[log_head] = ptr[i];
		log_head = next;
	}
	return len;
}

/* Drain whatever is queued. Returns bytes written. */
uint16_t log_flush(void)
{
	uint16_t head = log_head;        /* snapshot: _write may add more */
	uint16_t tail = log_tail;
	uint16_t n;

	if (tail == head) {
		return 0;
	}
	/* Send up to the wrap point, so the UART sees one contiguous block. */
	n = (head > tail) ? (uint16_t)(head - tail)
			  : (uint16_t)(LOG_BUF_SZ - tail);
	HAL_UART_Transmit(&huart1, (uint8_t *)&log_buf[tail], n, 100);
	log_tail = (uint16_t)((tail + n) & (LOG_BUF_SZ - 1U));
	return n;
}

