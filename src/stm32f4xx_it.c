/**
 * Interrupt handlers. SysTick drives HAL_GetTick() *and* the FreeRTOS tick;
 * OTG_FS routes the USB interrupt into the HAL HCD driver.
 */
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

extern HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* Defined by the FreeRTOS port; not declared in any public header because the
 * port normally claims SysTick_Handler itself. See the note below. */
extern void xPortSysTickHandler(void);

/*
 * Deliberately NOT mapped to xPortSysTickHandler in FreeRTOSConfig.h.
 *
 * Every timing decision in this project — the 10 ms control period, the probe
 * envelope, the step-response identification the car-feel model was fitted
 * from — is written against HAL_GetTick(). Handing SysTick wholly to FreeRTOS
 * would stop HAL_IncTick() and freeze that clock at zero, so both run from the
 * same 1 kHz tick and keep meaning the same millisecond.
 *
 * The scheduler-state check matters: HAL_Delay() is used during clock and USB
 * bring-up, before any task exists, and the kernel tick must not run then.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();

	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
		xPortSysTickHandler();
	}
}

void OTG_FS_IRQHandler(void)
{
	HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
}

/* ── FreeRTOS diagnostic hooks ──────────────────────────────────────────── */
/*
 * A stack overflow or a failed allocation otherwise shows up as a random hang,
 * which on a board whose only output is one UART is close to undebuggable.
 * Park in a loop with the name still in scope so a debugger can read it, and
 * leave the LED on so it is obvious from across the desk.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
	(void)task;
	(void)name;                       /* read this in the debugger */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	taskDISABLE_INTERRUPTS();
	for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	taskDISABLE_INTERRUPTS();
	for (;;) {}
}
