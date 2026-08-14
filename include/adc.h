#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);

uint16_t adc_read_channel(uint32_t channel);

uint16_t adc_read_mv(uint32_t channel);

/*
 * Read ADC channels and update the filtered
 * fuel/temperature values.
 */
void adc_update(void);

/*
 * Return cached values.
 *
 * Range:
 *     0..100
 */
uint8_t adc_get_fuel(void);

uint8_t adc_get_temperature(void);

#endif