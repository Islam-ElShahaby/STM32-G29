#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "shifter.h"

/**
 * @brief  Periodically constructs and sends CAN telemetry payload over MCP2515.
 *         Transmits every 10 ms or immediately on Hands-On/Hands-Off state change.
 *
 * @param  shifter_sel  Current PRND selector reading, as sampled by main.c's
 *                       control_task() (shared_shifter). Needed so the CAN
 *                       Mode byte can report N and P correctly -- powertrain.c's
 *                       own state (gear/reverse) has no way to distinguish
 *                       neutral from reverse, both read gear==0 there, so the
 *                       real selector reading has to come from the caller.
 * @param  hands_on     The actual hands-on/off verdict from hod_update(), as
 *                       already computed by steer_feel_update() in main.c and
 *                       published to shared_hands_on. There is no field for
 *                       this on struct steer_feel_telemetry -- the verdict
 *                       only exists as steer_feel_update()'s return value --
 *                       so it has to come in as a parameter rather than being
 *                       re-derived inside telemetry.c.
 */
void can_telemetry_update(enum shifter_mode shifter_sel, bool hands_on);

/**
 * @brief  Runs the press-to-toggle edge detection for all four lights from
 *         the raw wheel button bitmap, and returns the result packed the
 *         same way the CAN lights frame (0x0A3) encodes it (see
 *         LIGHTS_BIT_* in telemetry.c).
 *
 *         This is the ONE place the four toggle states get computed --
 *         call it once per control_task tick and feed the result to both
 *         can_lights_update() (for the CAN frame) and lights_gpio_set()
 *         (which further gates left_on/right_on by the indicator blink
 *         phase and hands led_fault_update() the actual driven states --
 *         see lights_gpio_set() below). Do not re-derive these from
 *         `buttons` anywhere else; two independent toggle state machines
 *         reading the same bitmap at slightly different rates is exactly
 *         the split-brain risk this project avoids everywhere else (see
 *         shifter_sel/hands_on in can_telemetry_update() below).
 *
 * @param  buttons        Raw G29 button bitmap (shared_buttons in main.c).
 * @param[out] left_on    Left indicator commanded-on state after this call.
 * @param[out] right_on   Right indicator commanded-on state after this call.
 * @param[out] low_on     Low beam commanded-on state after this call.
 * @param[out] high_on    High beam commanded-on state after this call.
 * @return  The same four states packed into one byte, CAN-frame bit order
 *          (LIGHTS_BIT_HIGH_BEAM=0, LOW_BEAM=1, LEFT=2, RIGHT=3) -- pass
 *          this straight into can_lights_update().
 */
uint8_t lights_toggle_update(uint32_t buttons, bool *left_on, bool *right_on,
                              bool *low_on, bool *high_on);
                              
/**
 * @brief  Sends the 0x0A3 exterior-lights frame.
 *
 * CAN frame layout:
 *   Byte 0: Light states
 *   Byte 1: LED fault states
 *   Byte 2: Button states
 *           Bit 0 = X button
 *   Byte 3: Rolling counter
 *
 * @param lights_state  Packed light state from lights_toggle_update().
 * @param fault_state   Packed LED fault state.
 * @param buttons       Raw G29 button bitmap (shared_buttons).
 */
void can_lights_update(uint8_t lights_state,
                       uint8_t fault_state,
                       uint32_t buttons);



/**
 * @brief  Configures the 4 GPIO outputs that physically drive the lights
 *         (left/right indicator, low/high beam). Call once from main()
 *         during bring-up, alongside adc_init()/led_fault_init().
 *
 *         Pins: PA5 = left indicator, PA6 = right indicator,
 *               PA7 = low beam, PA8 = high beam. Push-pull output,
 *               driven low (off) before the pins are switched to output
 *               mode, so there's no glitch-high on power-up.
 */
void lights_gpio_init(void);

/**
 * @brief  Drives the 4 physical LED outputs to match the current toggle
 *         state. Call once per control_task tick, right after
 *         lights_toggle_update() produces the 4 booleans -- see main.c.
 *
 *         left_on/right_on are further gated by the 1 Hz indicator blink
 *         here (500 ms on / 500 ms off) before being applied to the pins;
 *         low_on/high_on are applied steady. The actual driven state of
 *         each of the 4 pins after that gating is returned via
 *         left_out/right_out/low_out/high_out -- pass THESE (not the raw
 *         left_on/right_on) to led_fault_update(), since fault sampling
 *         needs to know what's actually on the wire right now, not just
 *         whether the indicator function is toggled on. low_out/high_out
 *         always equal low_on/high_on (no blink gating on those two) but
 *         are returned for symmetry so callers never have to know which
 *         channels blink.
 */
void lights_gpio_set(bool left_on, bool right_on, bool low_on, bool high_on,
                      bool *left_out, bool *right_out, bool *low_out,
                      bool *high_out);

uint8_t get_led_fault_state(void);

#endif /* TELEMETRY_H */