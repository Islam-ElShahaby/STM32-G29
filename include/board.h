#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

/*
 * Board bring-up for the WeAct BlackPill STM32F401CEU6.
 *
 * Everything here is F401-specific and has to be got right before anything
 * else runs:
 *   - 25 MHz HSE -> PLL -> 84 MHz SYSCLK (the F401 maximum) and, critically,
 *     exactly 48 MHz on PLLQ, without which USB will not enumerate.
 *   - PC13 status LED (active low).
 *   - USART1 on PA9/PA10 at 115200 8N1, with receive interrupt enabled.
 *
 * For an F411 board change the PLL to N=192, P=/2, Q=4 and FLASH_LATENCY_3.
 */

/* Console UART. Shared because the log ring drains it (log.c) and the console
 * receive ISR reads its data register directly (main.c). */
extern UART_HandleTypeDef huart1;

/*
 * Clock, LED and console UART, in that order. Call once from main() before
 * anything that prints or touches USB. Halts internally if the clock cannot
 * be configured — there is no sensible way to continue from that.
 */
void board_init(void);

/*
 * Start the DWT cycle counter. Not called from board_init(): FreeRTOS calls it
 * itself from vTaskStartScheduler() via portCONFIGURE_TIMER_FOR_RUN_TIME_STATS,
 * so the run-time stats and the counter start together. It is the timebase for
 * the cpu= figure in the console line.
 */
void cpu_cyccnt_init(void);

#endif /* BOARD_H */
