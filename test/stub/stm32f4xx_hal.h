/* Host-test stub: telemetry.c only uses HAL_GetTick(). See test_telemetry.c. */
#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
