#ifndef STEER_FEEL_H
#define STEER_FEEL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Steering feel and hands-on detection.
 *
 * THE CENTRAL CONSTRAINT: in C294 compat mode the wheel exposes exactly ONE
 * force-feedback channel — a single constant force. Every effect this module
 * produces is summed into that one 16-bit number:
 *
 *     tyre scrub  + viscous damping        (car_feel, resists motion)
 *   + self-aligning torque + stiction comp (pulls toward centre, grows w/ speed)
 *   + road rumble                          (broadband, grows with speed)
 *   + hands-on probe                       (a deliberate nudge, when used)
 *
 * That shared channel is why the parts interact and why suppression rules
 * exist: two writers fighting over one torque makes both useless. The module
 * arbitrates internally so callers never have to.
 *
 * All gains are in the torque units the wheel's step-response identification
 * used (see README), which is what makes them physical rather than arbitrary.
 * Two numbers from that fit are load-bearing everywhere:
 *   - breakaway (stiction) = 3401 units = 10.4 % of full scale. Below this the
 *     wheel does not move AT ALL.
 *   - natural viscous damping = 13.8 units per deg/s.
 */

/*
 * Control period. steer_feel_update() MUST be called at exactly this rate:
 * velocity is a difference over a fixed interval, and every drag, ramp and
 * damping constant was tuned against it. main.c also divides it down to reach
 * the powertrain's own 20 ms tick.
 */
#define FEEL_MS 10

/* Total torque this module last applied, plus the pieces worth watching while
 * tuning. Cheap snapshot for the periodic status log. */
struct steer_feel_telemetry {
	int16_t  force;		/* total torque sent to the wheel, post-clamp */
	int16_t  vel;		/* steering velocity, widened counts / 50 ms */
	int16_t  centre;	/* self-aligning component alone */
	uint16_t p2p;		/* last hands-on probe excursion */
};

/*
 * Advance the model one control tick and return true if the driver's hands are
 * detected on the wheel. Call at a FIXED rate (FEEL_MS, 100 Hz) — velocity is
 * a difference over a fixed interval, and a drifting one silently rescales
 * every gain.
 *
 * Internally arbitrates the single FFB channel: the bring-up sweep and the
 * step-response identification each own the torque outright while running,
 * because the whole point of both is that nothing else perturbs the wheel.
 *
 * `steering` is the wheel position, 0..0xFFFF, centre 0x8000.
 */
bool steer_feel_update(uint16_t steering);

void steer_feel_get_telemetry(struct steer_feel_telemetry *out);

/*
 * Console plumbing. The tunables stay PRIVATE to steer_feel.c — this dispatch
 * is the seam, so adding a knob does not mean adding another extern global.
 *
 * steer_feel_console() returns false if `cmd` is not one of its knobs, letting
 * the caller try its own commands.
 */
bool steer_feel_console(char cmd, long value);
void steer_feel_print_values(void);	/* one line of current values */
void steer_feel_print_help(void);	/* the knob descriptions */

/* 'S' — start the step-response identification (CSV to serial). */
void steer_feel_sysid_start(void);

/* 's' — stop all effects and abort any identification in progress. */
void steer_feel_stop(void);

#endif /* STEER_FEEL_H */
