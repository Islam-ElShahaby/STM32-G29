#ifndef LOG_H
#define LOG_H

#include <stdint.h>

/*
 * Non-blocking printf retarget.
 *
 * printf() used to call HAL_UART_Transmit(..., HAL_MAX_DELAY), which blocks
 * until the last bit is on the wire. At 115200 8N1 that is ~87 us per
 * character, so a ~105-character status line costs about 9.1 ms — very nearly
 * the whole 10 ms control period. Control jitter that big is not acceptable
 * when the torque loop is a real task.
 *
 * So printf() only copies into a ring here and returns; the console task calls
 * log_flush() and does the waiting. One change fixes every printf in the
 * project, including those inside g29_hid.c.
 *
 * Overflow DROPS characters rather than blocking: a lost log line is cosmetic,
 * a missed control period is not. log_dropped counts them so the loss is
 * visible rather than silent.
 */

/* Push whatever is queued to the UART. Returns bytes written, 0 if idle.
 * Call from the lowest-priority task — it blocks on the UART. */
uint16_t log_flush(void);

/* Characters lost to ring overflow since boot. Reported by the 'k' command. */
extern volatile uint32_t log_dropped;

#endif /* LOG_H */
