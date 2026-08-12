/**
 * FreeRTOS configuration for the G29 USB-host board (STM32F401CC, Cortex-M4F).
 *
 * Two decisions here are load-bearing and worth reading before changing:
 *
 * 1. SysTick is NOT handed to FreeRTOS via xPortSysTickHandler. Our handler in
 *    stm32f4xx_it.c calls HAL_IncTick() and then the FreeRTOS tick, because
 *    every timing decision in this project is written against HAL_GetTick()
 *    and the step-response identification was measured on that clock.
 *
 * 2. The tick runs at 1 kHz so HAL_GetTick() keeps meaning milliseconds.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                 12
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_APPLICATION_TASK_TAG          0

/*
 * Task stacks and TCBs come out of here. The four tasks below use ~8 KB, so
 * this leaves ~12 KB for whatever gets added next. Total RAM is 64 KB and the
 * whole image sits near 26 KB, so there is room to raise this further — check
 * the 'k' console command for the live figure before guessing.
 */
#define configTOTAL_HEAP_SIZE                   ((size_t)20480)
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0

/* Catches the two mistakes that actually happen: a task overflowing its stack
 * and a bad interrupt priority. Both hang in a way that is otherwise very hard
 * to diagnose on a board whose only output is a serial port. */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1

#define configUSE_TIMERS                        0
#define configUSE_TRACE_FACILITY                0

/*
 * CPU load, printed by the periodic console line. The kernel accumulates per
 * task run time from the DWT cycle counter (cpu_cyccnt_init() in board.c);
 * console_task() diffs the idle task's share against elapsed cycles, so the
 * figure is the average over the interval between two console lines, not an
 * instantaneous sample.
 *
 * Declared extern rather than #include "board.h" so the kernel sources do not
 * drag in the HAL — same reason SystemCoreClock is declared by hand above.
 */
#define configGENERATE_RUN_TIME_STATS           1
extern void cpu_cyccnt_init(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() cpu_cyccnt_init()
#define portGET_RUN_TIME_COUNTER_VALUE() \
	(*(volatile uint32_t *)0xE0001004UL)   /* DWT->CYCCNT */

/* Only the API this project actually calls. */
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
/* ulTaskGetIdleRunTimeCounter() is compiled only when this is also 1. */
#define INCLUDE_xTaskGetIdleTaskHandle          1

/* ── Cortex-M interrupt priorities ──────────────────────────────────────── */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                         __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                         4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
	(configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
	(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*
 * OTG_FS runs at priority 5 (usbh_conf.c), i.e. exactly at the syscall ceiling,
 * so it may not call FreeRTOS API from the ISR — it does not: our
 * HAL_HCD_HC_NotifyURBChange_Callback is empty and the class polls instead.
 */

/* SVC and PendSV go to FreeRTOS. SysTick deliberately does NOT — see the file
 * header; stm32f4xx_it.c drives HAL_IncTick() and the kernel tick together. */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler

#define configASSERT(x)                                                     \
	if ((x) == 0) {                                                     \
		taskDISABLE_INTERRUPTS();                                   \
		for (;;) {}                                                 \
	}

#endif /* FREERTOS_CONFIG_H */
