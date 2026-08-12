#ifndef ADC_H
#define ADC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void adc_init(void);
uint16_t adc_read_channel(uint32_t channel);
uint16_t adc_read_mv(uint32_t channel);
#endif