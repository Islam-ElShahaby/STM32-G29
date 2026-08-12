#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Analog PRND selector lever — a potentiometer on PA1 (ADC1_IN1).
 *
 * Ported from the Node-1 vehicle ECU's gear-selector (there it was PTC29 on an
 * S32K148 under Zephyr; same idea, HAL instead of the Zephyr ADC API).
 *
 * The lever is ABSOLUTE: unlike the 'g' console command it has no state of its
 * own to get out of sync with, so whatever position it is in at power-up is the
 * gear the car is in. The only state kept here is the last valid mode, held
 * while the wiper crosses the dead zone between two detents.
 */

enum shifter_mode {
	SHIFTER_P = 0,
	SHIFTER_R,
	SHIFTER_N,
	SHIFTER_D,
};

/* ADC1 + PA1 in analog mode. Call once from main(), before the tasks start. */
void shifter_init(void);

/*
 * Sample the lever. Call at a steady rate (PT_TICK_MS is plenty — a hand
 * cannot move a lever faster than 50 Hz resolves).
 *
 * Returns false when no lever has ever been seen: an unconnected pin floats,
 * and a floating reading lands in a dead zone or nowhere at all, so a build
 * with no lever wired stays on the 'g' console command instead of being forced
 * into whatever gear the noise happens to name. Once a real detent has been
 * read once, this returns true forever and the lever owns the gear.
 *
 * `*out` is untouched when it returns false.
 */
bool shifter_update(enum shifter_mode *out);

/* Raw ADC counts from the last shifter_update(), 0..4095. This is the number
 * to watch in the status log when calibrating SHIFTER_*_MIN/MAX. */
uint16_t shifter_raw(void);

/* 'P', 'R', 'N' or 'D', for logs. */
char shifter_letter(enum shifter_mode m);

#endif /* SHIFTER_H */
