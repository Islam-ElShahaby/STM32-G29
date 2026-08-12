/*
 * Bring-up and characterisation tools — see include/bringup.h.
 *
 * Split out of the steering-feel model: this is measurement code, run by hand
 * during bring-up, not part of the 100 Hz control path.
 */
#include "bringup.h"

#include "stm32f4xx_hal.h"
#include "g29_hid.h"
#include <stdio.h>

/* ── FFB protocol sweep (bring-up only) ─────────────────────────────────── */
/*
 * In C294 compat mode the wheel accepts and echoes our effect commands but the
 * motor does not respond, while its own default centring spring clearly does.
 * So some command class works and some does not. This walks a list of
 * candidates, holds each one, and reports how far the wheel actually moved —
 * movement is measured from the steering reading, not judged by feel.
 *
 * Set 0 for normal operation.
 */
#define FFB_SWEEP 0
#define SWEEP_HOLD_MS 1500U

static const struct {
	const char *name;
	uint8_t cmd[7];
} sweep[] = {
	{ "stop-all           ", { 0xf3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ "default-spring ON  ", { 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ "default-spring OFF ", { 0xf5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ "autocenter set max ", { 0xfe, 0x0d, 0x0f, 0x0f, 0xff, 0x00, 0x00 } },
	{ "autocenter ACTIVATE", { 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ "constant  t00 full+", { 0x11, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00 } },
	{ "constant  t00 full-", { 0x11, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 } },
	{ "variable  t08 full+", { 0x11, 0x08, 0xff, 0x80, 0x00, 0x00, 0x00 } },
	{ "spring    t01      ", { 0x11, 0x01, 0x00, 0xff, 0x77, 0x00, 0x00 } },
	{ "damper    t02      ", { 0x11, 0x02, 0x07, 0x07, 0x00, 0x00, 0x00 } },
	{ "hi-res spring t0b  ", { 0x11, 0x0b, 0x00, 0xff, 0x77, 0x00, 0x00 } },
	{ "hi-res damper t0c  ", { 0x11, 0x0c, 0x07, 0x07, 0x00, 0x00, 0x00 } },
	{ "friction  t0e      ", { 0x11, 0x0e, 0x07, 0x07, 0x00, 0x00, 0x00 } },
};

/* Returns true while the sweep is still running (it owns the FFB channel). */
bool ffb_sweep(uint16_t steering)
{
	static uint32_t t0;
	static uint16_t lo = 0xFFFF, hi;
	static uint8_t idx;
	static bool started;
	uint32_t now = HAL_GetTick();

	/*
	 * The enable check lives HERE, deliberately, not at the call site.
	 *
	 * It used to be `!(FFB_SWEEP && ffb_sweep(...))` in main.c, and when that
	 * arbitration was collapsed into steer_feel_update() the guard was lost
	 * while the #define stayed — so the sweep ran on EVERY boot and drove the
	 * wheel lock to lock with full-scale constant force (see the ff/00 entries
	 * in the table above, 1.5 s each, alternating direction).
	 *
	 * A compile-time switch whose only enforcement is in a different file is a
	 * switch waiting to be dropped. Now the switch and its guard cannot be
	 * separated.
	 */
	if (!FFB_SWEEP) {
		return false;
	}

	if (idx >= (uint8_t)(sizeof(sweep) / sizeof(sweep[0]))) {
		return false;
	}

	if (!started) {
		started = true;
		t0 = now;
		lo = hi = steering;
		printf("\r\n--- FFB sweep: each effect held %lums, "
		       "'moved' = steering excursion ---\r\n",
		       (unsigned long)SWEEP_HOLD_MS);
		g29_send_raw(sweep[0].cmd);
		printf("  %s -> %02X %02X %02X %02X %02X\r\n", sweep[0].name,
		       sweep[0].cmd[0], sweep[0].cmd[1], sweep[0].cmd[2],
		       sweep[0].cmd[3], sweep[0].cmd[4]);
		return true;
	}

	if (steering < lo) { lo = steering; }
	if (steering > hi) { hi = steering; }

	if (now - t0 >= SWEEP_HOLD_MS) {
		/* /257 converts widened counts back to real 8-bit steering counts */
		printf("  %s   moved %u counts%s\r\n", sweep[idx].name,
		       (unsigned)((hi - lo) / 257U),
		       ((hi - lo) / 257U > 2U) ? "   <<< MOVED" : "");
		idx++;
		if (idx >= (uint8_t)(sizeof(sweep) / sizeof(sweep[0]))) {
			printf("--- sweep done ---\r\n");
			g29_send_no_effect();
			return false;
		}
		g29_send_raw(sweep[idx].cmd);
		printf("  %s -> %02X %02X %02X %02X %02X\r\n", sweep[idx].name,
		       sweep[idx].cmd[0], sweep[idx].cmd[1], sweep[idx].cmd[2],
		       sweep[idx].cmd[3], sweep[idx].cmd[4]);
		t0 = now;
		lo = hi = steering;
	}
	return true;
}

/* ── System identification: step response ───────────────────────────────── */
/*
 * Applies a constant torque step from rest and logs steering at the USB report
 * rate, so the wheel's torque->velocity dynamics can be fitted offline.
 *
 * The plant is a rotational mass with viscous and Coulomb friction:
 *     J*dw/dt = K*u - b*w - Tc*sign(w)
 * whose step response from rest is first order:
 *     w(t) = w_ss * (1 - exp(-t/tau)),  w_ss = (K*u - Tc)/b,  tau = J/b
 * Sweeping u gives w_ss(u): its slope is K/b and its x-intercept is the
 * breakaway (stiction) torque. tau comes from the exponential.
 *
 * Direction is chosen toward centre before each step so the wheel never runs
 * into an end stop mid-measurement and corrupts the trace.
 *
 * Output is CSV over serial: "sid,<force>,<ms>,<steer>", ending with "sid,end".
 * Start it with the 'S' console command.
 */
static const int32_t sysid_forces[] = {
	3000, 5000, 7000, 10000, 14000, 19000, 25000, 32000
};
#define SYSID_SETTLE_MS 500U
#define SYSID_STEP_MS   250U    /* travel is only ±180°; a long step hits a stop */
#define SYSID_CENTRE_F  9000    /* torque used to drive back to centre */
#define SYSID_CENTRE_TOL 7000U  /* ~27 real counts; a tight band never settles */
#define SYSID_CENTRE_MAX 2500U  /* give up centring after this and step anyway */
#define SYSID_TICK_MS   10U     /* matches the wheel's 10 ms report interval */

static bool sysid_active;

bool sysid_update(uint16_t steering)
{
	static uint32_t t0, t_tick, t_centre;
	static uint8_t idx;
	static int8_t dir;
	static bool stepping, centering;
	uint32_t now = HAL_GetTick();

	if (!sysid_active) {
		idx = 0;
		stepping = false;
		centering = true;
		t0 = t_centre = now;
		return false;
	}
	if (now - t_tick < SYSID_TICK_MS) {
		return true;                    /* keep ownership between ticks */
	}
	t_tick = now;

	/*
	 * Re-centre first. Travel is only ±180°, and a step at 32000 covers ~100°
	 * in 150 ms, so starting anywhere but the middle just parks the wheel on
	 * an end stop and the whole trace reads as "no motion".
	 */
	if (centering) {
		int32_t err = (int32_t)steering - 32768;   /* + = right of centre */
		bool off_centre = (err > (int32_t)SYSID_CENTRE_TOL ||
				   err < -(int32_t)SYSID_CENTRE_TOL);

		/*
		 * Bang-bang with no damping limit-cycles around the band, and the
		 * wheel drifts on its own even at zero torque, so "wait until
		 * centred AND settled" can never finish. Cap the whole phase.
		 */
		if (off_centre && now - t_centre < SYSID_CENTRE_MAX) {
			g29_send_constant_force((err > 0) ? -SYSID_CENTRE_F
							  :  SYSID_CENTRE_F);
			t0 = now;               /* restart the settle once centred */
			return true;
		}
		g29_send_constant_force(0);
		if (now - t0 >= SYSID_SETTLE_MS) {
			centering = false;
			/* Step away from whichever side we ended up on. */
			dir = (err > 0) ? -1 : 1;
			t0 = now;
			stepping = true;
		}
		return true;
	}

	if (!stepping) {
		g29_send_constant_force(0);
		if (now - t0 >= SYSID_SETTLE_MS) {
			centering = true;
			t0 = t_centre = now;
		}
		return true;
	}

	{
		int32_t f = sysid_forces[idx] * dir;

		g29_send_constant_force((int16_t)f);
		printf("sid,%ld,%lu,%u\r\n", (long)f,
		       (unsigned long)(now - t0), steering);

		if (now - t0 >= SYSID_STEP_MS) {
			stepping = false;
			centering = true;
			t0 = now;
			idx++;
			if (idx >= (uint8_t)(sizeof(sysid_forces) /
					     sizeof(sysid_forces[0]))) {
				sysid_active = false;
				g29_send_constant_force(0);
				printf("sid,end\r\n");
			}
		}
	}
	return true;
}

void sysid_start(void)
{
	sysid_active = true;
}

void sysid_abort(void)
{
	sysid_active = false;
}

bool sysid_running(void)
{
	return sysid_active;
}
