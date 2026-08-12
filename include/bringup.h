#ifndef BRINGUP_H
#define BRINGUP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Bring-up and characterisation tools. Neither runs during normal operation —
 * they exist to answer questions about the wheel itself, and each one OWNS the
 * force-feedback channel while active, because the whole point is that nothing
 * else perturbs the wheel mid-measurement.
 *
 * Kept apart from steer_feel.c so the runtime torque model is not read as
 * having diagnostics tangled into it. steer_feel_update() calls both and
 * stands aside while either reports true.
 */

/*
 * Walk a list of candidate Logitech FFB commands, hold each, and report how far
 * the wheel actually moved — movement measured from the steering reading, not
 * judged by feel. This is how the "which effects work in C294" table in the
 * README was produced: constant force works, damper and hi-res effects do not.
 *
 * Compiled out unless FFB_SWEEP is set in bringup.c. Returns true while running.
 */
bool ffb_sweep(uint16_t steering);

/*
 * Step-response identification. Applies constant torque steps from rest at
 * eight amplitudes, re-centring between them so the wheel never sits on an end
 * stop and corrupts the trace, and dumps CSV over serial:
 *
 *     sid,<force>,<ms>,<steer>      ... then  sid,end
 *
 * Fitted offline to  J*dw/dt = K*u - b*w - Tc*sign(w)  this is where the
 * breakaway (3401), natural damping (13.8/deg/s) and tau (50 ms) figures that
 * the whole feel model is scaled against came from. Re-run it with 'S' if the
 * wheel is ever replaced or reaches native mode.
 *
 * Returns true while running.
 */
bool sysid_update(uint16_t steering);

void sysid_start(void);
void sysid_abort(void);
bool sysid_running(void);

#endif /* BRINGUP_H */
