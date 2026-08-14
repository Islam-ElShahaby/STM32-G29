/*
 * Analog PRND selector lever on PA1 / ADC1_IN1 — see include/shifter.h.
 *
 * Pin choice: PA1 is the only ADC-capable pin this board has left. PA4-PA7 are
 * the MCP2515's SPI, PA9/PA10 the console UART, PA11/PA12 USB OTG. It is also
 * the same channel index the Node-1 reference used, so the wiring notes carry
 * over unchanged.
 */
#include "stm32f4xx_hal.h"
#include "shifter.h"
#include "adc.h"

#include <stdint.h>
#include <stdbool.h>


/*
 * ── Detent windows, in raw ADC counts ───────────────────────────────────
 *
 * Each mode owns a window; the GAPS between windows are dead zones where the
 * previous mode is held. That is what stops the gear flickering while the
 * wiper is between two detents, and it is why the windows deliberately do not
 * tile the whole range:
 *
 *      0 ....... 700 | 1000 ..... 2200 | 2500 ..... 3900 | 4000 .. 4095
 *          D         |       N         |       R         |      P
 *                  dead              dead              dead
 *
 * THESE ARE THE NODE-1 VALUES AND ALMOST CERTAINLY WRONG FOR YOUR LEVER. They
 * depend on the pot's taper, its end resistance and whatever divider is in
 * front of it. Calibrate before trusting the thing: move the lever to each
 * detent, read `shf=` in the status log, and set each MIN/MAX a comfortable
 * margin inside the value you saw. Leave the gaps wide — a gap that is too
 * narrow costs you a flickering gear, one that is too wide costs nothing but
 * a slightly longer throw.
 *
 * Order matters only in that the windows must not overlap; the lookup below
 * returns the first match.
 */
#define SHIFTER_P_MIN   4000
#define SHIFTER_P_MAX   4095
#define SHIFTER_R_MIN   2500
#define SHIFTER_R_MAX   3900
#define SHIFTER_N_MIN   1000
#define SHIFTER_N_MAX   2200
#define SHIFTER_D_MIN      0
#define SHIFTER_D_MAX    700

/* Conversion timeout. One 12-bit conversion at 84/4 MHz with a 480-cycle
 * sample time is ~25 us, so 2 ms is pure "the peripheral is wedged" cover —
 * this runs in the task that owns the 100 Hz torque loop and must not park
 * there. */



static uint16_t last_raw;
static enum shifter_mode mode = SHIFTER_P;
static bool present;

/* First matching window, or -1 in a dead zone. */
static int band(uint16_t raw)
{
	if (raw >= SHIFTER_P_MIN && raw <= SHIFTER_P_MAX) { return SHIFTER_P; }
	if (raw >= SHIFTER_R_MIN && raw <= SHIFTER_R_MAX) { return SHIFTER_R; }
	if (raw >= SHIFTER_N_MIN && raw <= SHIFTER_N_MAX) { return SHIFTER_N; }
	if (raw >= SHIFTER_D_MIN && raw <= SHIFTER_D_MAX) { return SHIFTER_D; }

	return -1;
}

bool shifter_update(enum shifter_mode *out)
{
	last_raw = adc_read_channel(ADC_CHANNEL_1);

	int b = band(last_raw);
	if (b >= 0) {
	    mode = (enum shifter_mode)b;
	    present = true;
	}

	/*
	 * A failed conversion holds the previous mode rather than falling back
	 * to anything. Dropping to Park because one sample timed out would put
	 * the car through a full-brake stop on a transient.
	 */
	if (!present) {
		return false;
	}
	*out = mode;

	return true;
}

uint16_t shifter_raw(void)
{
	return last_raw;
}

char shifter_letter(enum shifter_mode m)
{
	return "PRND"[m & 3];
}
