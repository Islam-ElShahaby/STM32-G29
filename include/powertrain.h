#ifndef POWERTRAIN_H
#define POWERTRAIN_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Force-based car powertrain simulation: accelerator and brake pedals drive
 * vehicle speed directly; engine RPM is a CONSEQUENCE of speed through an
 * 8-speed automatic gearbox (+ reverse), not the other way round. Ported
 * from the proven Node-1 vehicle-ECU model (throttle-only) and extended with
 * a real brake-pedal force term.
 *
 * Pure integer math, no HAL/RTOS/CAN dependency -- host-buildable, see
 * test/test_powertrain.c.
 *
 * Call powertrain_tick() once per PT_TICK_MS at a FIXED rate: drag, shift
 * timing and brake/accel force are all tuned against that period.
 */

#define PT_TICK_MS   20
#define PT_NUM_GEARS 8

struct powertrain_state {
	uint16_t engine_rpm;	/* 800..7000 */
	uint16_t speed_kmh;	/* magnitude; direction is `reverse` */
	uint8_t  gear;		/* 1..8 forward; 0 while reversing */
	bool     reverse;
};

/* Reset to standstill, 1st gear, idle RPM, forward. */
void powertrain_init(void);

/*
 * Advance the simulation by one PT_TICK_MS step.
 *   throttle_pct, brake_pct : 0..100 (clamped)
 *   reverse_request         : desired direction. Only takes effect once the
 *                              vehicle is stopped -- a real selector has the
 *                              same interlock.
 */
void powertrain_tick(uint8_t throttle_pct, uint8_t brake_pct,
		      bool reverse_request, bool neutral);

/* Copy of the current state. */
void powertrain_get_state(struct powertrain_state *out);

/*
 * ── Speed-dependent steering feel ───────────────────────────────────────
 *
 * Both live here rather than in main.c because both are functions of
 * VEHICLE SPEED, which this module owns -- they read the internal speed
 * directly, so the caller can't feed them a stale value. Both return
 * torque in the same units the wheel's step-response identification used
 * (see README), so they sum straight onto the damper/probe torque.
 */

/*
 * Self-aligning torque: pulls the wheel back toward centre, growing with
 * speed. A real car's steering pulls straight harder the faster it goes
 * (caster + tyre pneumatic trail); at a standstill there is none, which is
 * why a parked car's wheel just stays where you leave it.
 *
 *   offset   : widened steering counts from centre, signed (+ = right)
 *   strength : torque at full lock once at/above the saturation speed.
 *              0 disables it entirely.
 *
 * Returns torque OPPOSING the offset (negative for a positive offset).
 */
int32_t powertrain_self_centre(int32_t offset, int32_t strength);

/*
 * Road-surface rumble, amplitude growing with speed. Stateful (it filters
 * a noise source) -- call exactly once per control tick.
 *
 *   strength : peak amplitude once at/above the saturation speed.
 *              0 disables it entirely. The low-pass inside costs some of
 *              this, so the torque actually produced peaks at roughly 70%
 *              of `strength` -- it is a tuning knob, not a calibrated
 *              amplitude.
 */
int32_t powertrain_road_rumble(int32_t strength);

#endif /* POWERTRAIN_H */
