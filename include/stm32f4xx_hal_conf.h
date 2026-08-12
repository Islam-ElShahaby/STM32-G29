/**
 * Minimal HAL configuration for the G29 USB host project.
 * Key point vs. the stock template: HAL_HCD_MODULE_ENABLED is on (USB host),
 * and HSE_VALUE is 25 MHz to match the WeAct BlackPill crystal.
 */
#ifndef STM32F4xx_HAL_CONF_H
#define STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ##### Module Selection ##### */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED          /* <-- MCP2515 CAN controller */
#define HAL_HCD_MODULE_ENABLED          /* <-- USB host controller driver */
#define HAL_ADC_MODULE_ENABLED          /* <-- PRND shifter lever pot */

/* ##### Oscillator Values (adapt to your board) ##### */
#if !defined(HSE_VALUE)
#define HSE_VALUE    25000000U          /* BlackPill V2.0 = 25 MHz crystal */
#endif
#define HSE_STARTUP_TIMEOUT    100U

#if !defined(HSI_VALUE)
#define HSI_VALUE    16000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE    32000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE    32768U
#endif
#define LSE_STARTUP_TIMEOUT    5000U
#define EXTERNAL_CLOCK_VALUE   12288000U

/* ##### System Configuration ##### */
#define VDD_VALUE                     3300U
/* Lowest priority: FreeRTOS requires the kernel tick to sit at or below the
 * syscall ceiling, since SysTick_Handler can request a context switch. The
 * port ORs it down to 15 anyway — this just makes it explicit rather than
 * accidental. */
#define TICK_INT_PRIORITY             15U
#define USE_RTOS                      0U
#define PREFETCH_ENABLE               1U
#define INSTRUCTION_CACHE_ENABLE      1U
#define DATA_CACHE_ENABLE             1U
#define USE_SPI_CRC                   0U

#define assert_param(expr) ((void)0U)

/* ##### Module headers ##### */
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f4xx_hal_exti.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"
#endif
#ifdef HAL_HCD_MODULE_ENABLED
#include "stm32f4xx_hal_hcd.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f4xx_hal_adc.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F4xx_HAL_CONF_H */
