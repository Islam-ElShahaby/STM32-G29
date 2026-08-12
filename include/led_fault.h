#ifndef LED_FAULT_H
#define LED_FAULT_H

#include <stdint.h>
#include <stdbool.h>

#define LED_COUNT 4

typedef enum
{
    LED_OK = 0,
    LED_OPEN,
    LED_SHORT
} led_fault_state_t;

void led_fault_init(void);

/**
 * @brief  Samples one LED (round-robin, one per call -- see led_fault.c)
 *         and updates its fault verdict once a debounce window completes.
 *
 * @param  left_on   Left indicator  (LED index 0) commanded-on state.
 * @param  right_on  Right indicator (LED index 1) commanded-on state.
 * @param  low_on    Low beam        (LED index 2) commanded-on state.
 * @param  high_on   High beam       (LED index 3) commanded-on state.
 *
 * The "on" state has to come from the caller: led_fault.c has no way to
 * know whether a light is legitimately dark or actually open/shorted
 * without being told which one it is. It must be the actual driven pin
 * state -- i.e. telemetry.c:lights_gpio_set()'s left_out/right_out/
 * low_out/high_out outputs, AFTER the 1 Hz indicator blink gating is
 * applied -- not the raw toggle-enable state from lights_toggle_update().
 * Left/right indicators are commanded on for their whole blink cycle but
 * only physically driven for half of it (500 ms on / 500 ms off); feeding
 * the pre-blink toggle state here would sample the node as "should be lit"
 * during the dark half and misclassify every blink as a short. While a
 * given LED is off, its debounce window is held (not advanced, not reset)
 * rather than sampled: there is no defined "healthy" voltage for an
 * unpowered node in this circuit, so the correct behaviour is to freeze
 * that LED's last verdict, not guess.
 */
void led_fault_update(bool left_on, bool right_on, bool low_on, bool high_on);

led_fault_state_t led_fault_get(uint8_t led);
uint16_t led_fault_voltage(uint8_t led);


#endif