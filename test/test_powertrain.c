/*
 * Host-buildable self-check for src/powertrain.c. Not part of the firmware
 * build (PlatformIO only compiles src/); run directly:
 *
 *   gcc -I include src/powertrain.c test/test_powertrain.c -o /tmp/pt_test && /tmp/pt_test
 */
#include <assert.h>
#include <stdio.h>
#include "powertrain.h"

static struct powertrain_state st;

/* Drive (or reverse) -- the common case, neutral clutch engaged. */
static void tick(uint8_t throttle, uint8_t brake, bool reverse, int n)
{
	for (int i = 0; i < n; i++) {
		powertrain_tick(throttle, brake, reverse, false);
	}
	powertrain_get_state(&st);
}

/* Neutral/Park: gearbox disconnected from the wheels. */
static void tick_n(uint8_t throttle, uint8_t brake, int n)
{
	for (int i = 0; i < n; i++) {
		powertrain_tick(throttle, brake, false, true);
	}
	powertrain_get_state(&st);
}

int main(void)
{
	powertrain_init();
	powertrain_get_state(&st);
	assert(st.engine_rpm == 800);
	assert(st.gear == 1);
	assert(st.speed_kmh == 0);
	assert(!st.reverse);

	/* Full throttle, forward, 15 s: should climb through several gears and
	 * comfortably clear 90 km/h (reference tuning: ~100 km/h in 8-10 s).
	 * Gears 1..8 exist and downshift correctly through all of them (see
	 * below); WOT-from-a-stop only needs a few of them before aero drag
	 * balances the available force in this gear -- that's a property of
	 * the ported tuning constants, not of the shift logic. */
	tick(100, 0, false, 15000 / PT_TICK_MS);
	assert(st.speed_kmh > 90);
	assert(st.gear > 1 && st.gear <= PT_NUM_GEARS);
	assert(st.engine_rpm >= 800 && st.engine_rpm <= 7000);
	printf("after 15s WOT: speed=%u gear=%u rpm=%u\n",
	       st.speed_kmh, st.gear, st.engine_rpm);

	/*
	 * Held flat out, the car must settle at its designed top speed IN TOP
	 * GEAR. The gear matters as much as the number: 8th is only reachable
	 * if the 7->8 upshift speed sits below the force/drag equilibrium, and
	 * it did not used to -- 7th peaked at exactly the threshold it had to
	 * cross, so 8th took ~24 s and any extra drag locked it out entirely.
	 * See the gear_ratio[] note on why 7th breaks the progression.
	 */
	tick(100, 0, false, 45000 / PT_TICK_MS);
	assert(st.speed_kmh >= 190 && st.speed_kmh <= 200);
	assert(st.gear == PT_NUM_GEARS);
	printf("top speed (60s WOT): speed=%u gear=%u rpm=%u\n",
	       st.speed_kmh, st.gear, st.engine_rpm);

	/*
	 * The rev limiter must be REAL, not just a display clamp. clamp_rpm()
	 * caps the reported RPM, but force used to keep being applied on the
	 * clamped value, so the car ran past each gear's kinematic redline with
	 * the tacho pinned at 7000 -- measured at 3% of all WOT ticks, worst
	 * case 402 rpm over in 1st. Check the speed/gear pair directly rather
	 * than engine_rpm, because engine_rpm is exactly what was lying.
	 */
	powertrain_init();
	for (int i = 0; i < 30000 / PT_TICK_MS; i++) {
		powertrain_tick(100, 0, false, false);
		powertrain_get_state(&st);
		if (st.gear >= 1 && st.gear <= PT_NUM_GEARS) {
			/* same ladder as gear_ratio[] in powertrain.c */
			static const int ratio[PT_NUM_GEARS] = {
				4700, 3830, 3120, 2540, 2070, 1690, 1600, 1120
			};
			int demanded = st.speed_kmh * ratio[st.gear - 1] / 40;

			assert(demanded <= 7000);
		}
	}
	printf("30s WOT: no gear ever exceeded the 7000 rpm limiter\n");

	/*
	 * Lifting off must not stop the car like a brake pedal. Engine braking
	 * scales with RPM above idle now; before, it was a flat function of
	 * gear alone and killed 40 km/h in 2.1 s (17 km/h/s, half the full
	 * brake pedal) just for coming off the accelerator.
	 */
	powertrain_init();
	do {
		powertrain_tick(100, 0, false, false);
		powertrain_get_state(&st);
	} while (st.speed_kmh < 40);
	int coast_ticks = 0;
	while (st.speed_kmh > 0 && coast_ticks < 60000 / PT_TICK_MS) {
		powertrain_tick(0, 0, false, false);
		powertrain_get_state(&st);
		coast_ticks++;
	}
	assert(st.speed_kmh == 0);			/* must still finish  */
	assert(coast_ticks * PT_TICK_MS > 5000);	/* but not slam       */
	printf("coast from 40 km/h to standstill: %.1f s\n",
	       coast_ticks * PT_TICK_MS / 1000.0);

	powertrain_init();
	tick(100, 0, false, 20000 / PT_TICK_MS);

	/* Full brake: must come to a complete stop, downshifting through the
	 * gears on the way down -- never upshifting while braking (that was a
	 * real bug: see the throttle>0 guard in forward_tick()). 8 s, because
	 * this now brakes from the full 199 km/h above and PT_BRAKE_SCALE is
	 * tuned for ~33 km/h/s. */
	uint8_t gear_before_brake = st.gear;
	tick(0, 100, false, 8000 / PT_TICK_MS);
	assert(st.speed_kmh == 0);
	assert(st.gear <= gear_before_brake);
	assert(st.gear == 1);
	printf("after full brake from top speed: speed=%u gear=%u\n",
	       st.speed_kmh, st.gear);

	/* Reverse request while still rolling must be ignored (interlock);
	 * only takes effect once actually stopped (it already is here). */
	tick(0, 0, true, 1);
	assert(st.reverse);
	assert(st.gear == 0);

	/* Reverse throttle: moves, capped well under forward top speed. */
	tick(100, 0, true, 3000 / PT_TICK_MS);
	assert(st.reverse);
	assert(st.speed_kmh > 0 && st.speed_kmh <= 23);
	printf("after 3s reverse WOT: speed=%u rpm=%u\n", st.speed_kmh, st.engine_rpm);

	/* Still moving in reverse: forward request must be ignored. */
	tick(100, 0, false, 1);
	assert(st.reverse);

	/*
	 * Reverse tip-in after a coast must never make the car SLOWER. rpm and
	 * road speed genuinely diverge while coasting (rpm decays on its own
	 * timer, speed on drag), and reverse_tick() used to re-derive speed
	 * from the decayed rpm the moment throttle returned -- which dropped
	 * the car from 22 km/h to 15 km/h in a single tick.
	 */
	tick(0, 0, true, 50);		/* coast, letting rpm fall away  */
	uint16_t coast_speed = st.speed_kmh;
	assert(coast_speed > 0);
	tick(100, 0, true, 1);		/* back on the throttle          */
	assert(st.speed_kmh >= coast_speed);
	printf("reverse tip-in after coast: %u -> %u km/h\n",
	       coast_speed, st.speed_kmh);

	/* Brake to a stop, then forward request takes effect. */
	tick(0, 100, true, 4000 / PT_TICK_MS);
	assert(st.speed_kmh == 0);
	tick(0, 0, false, 1);
	assert(!st.reverse);
	assert(st.gear == 1);

	/*
	 * The box must be in 1st WITHIN ONE TICK of the car stopping -- not
	 * three ticks later. A shift still slipping as the car stopped used to
	 * run to completion and step the gear down again afterwards, so the
	 * reported gear walked 7, 7, 6, 1 over 60 ms while stationary. That
	 * walk goes out on CAN 0x0A2, so it is visible, not just internal.
	 */
	powertrain_init();
	tick(100, 0, false, 20000 / PT_TICK_MS);
	assert(st.speed_kmh > 100);
	do {
		powertrain_tick(0, 100, false, false);
		powertrain_get_state(&st);
	} while (st.speed_kmh > 0);
	tick(0, 100, false, 1);
	assert(st.gear == 1);
	printf("gear one tick after standstill: %u\n", st.gear);

	/*
	 * Neutral/Park: the gearbox is disconnected, so throttle free-revs the
	 * engine to the limiter and CANNOT drive the wheels. Both halves
	 * matter -- main.c passes the pedal through unchanged in N/P.
	 */
	powertrain_init();
	tick_n(100, 0, 1000 / PT_TICK_MS);
	assert(st.engine_rpm == 7000);
	assert(st.speed_kmh == 0);
	printf("N/P 1s WOT: rpm=%u speed=%u (must not move)\n",
	       st.engine_rpm, st.speed_kmh);

	/* Rolling into N must coast down, never accelerate. */
	powertrain_init();
	tick(100, 0, false, 3000 / PT_TICK_MS);
	uint16_t roll = st.speed_kmh;
	assert(roll > 0);
	tick_n(100, 0, 2000 / PT_TICK_MS);
	assert(st.speed_kmh < roll);
	printf("rolling into N at WOT: %u -> %u km/h (coasts down)\n",
	       roll, st.speed_kmh);

	/* --- Speed-dependent steering feel --- */
	powertrain_init();

	/* Parked: no self-aligning torque, no road rumble. This is what keeps
	 * the wheel where the driver leaves it at a standstill. */
	assert(powertrain_self_centre(10000, 12000) == 0);
	assert(powertrain_road_rumble(2000) == 0);

	/* Gentle throttle -> a low speed, comfortably below the saturation
	 * point, so the speed ramp is still doing something. */
	tick(30, 0, false, 2000 / PT_TICK_MS);
	assert(st.speed_kmh > 0 && st.speed_kmh < 80);
	int32_t c_low = powertrain_self_centre(20000, 12000);

	/* Faster -> centring must grow. */
	tick(100, 0, false, 6000 / PT_TICK_MS);
	assert(st.speed_kmh > 80);
	int32_t c_high = powertrain_self_centre(20000, 12000);
	printf("self-centre @20000 counts: low-speed=%d high-speed=%d\n",
	       c_low, c_high);
	assert(c_low < 0 && c_high < 0);	/* opposes a positive offset */
	assert(c_high < c_low);			/* stronger at speed */

	/* Grows with offset, is symmetric, and strength 0 disables it. */
	assert(powertrain_self_centre(20000, 12000) <
	       powertrain_self_centre(4000, 12000));
	assert(powertrain_self_centre(-20000, 12000) == -c_high);
	assert(powertrain_self_centre(20000, 0) == 0);

	/* Offset beyond full lock must clamp, not wrap. */
	assert(powertrain_self_centre(999999, 12000) ==
	       powertrain_self_centre(32768, 12000));

	/* Rumble stays inside the requested amplitude and actually moves. */
	int32_t peak = 0, trough = 0;
	for (int i = 0; i < 2000; i++) {
		int32_t r = powertrain_road_rumble(2000);

		assert(r >= -2000 && r <= 2000);
		if (r > peak) {
			peak = r;
		}
		if (r < trough) {
			trough = r;
		}
	}
	printf("rumble over 2000 ticks: peak=%d trough=%d\n", peak, trough);
	assert(peak > 0 && trough < 0);
	assert(powertrain_road_rumble(0) == 0);

	printf("OK\n");
	return 0;
}
