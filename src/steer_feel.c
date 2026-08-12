/*
 * Steering feel and hands-on detection — see include/steer_feel.h for the
 * one-FFB-channel constraint that shapes this whole file.
 *
 * Split out of main.c, which had grown past 1800 lines. Everything here is the
 * torque model and its calibration; main.c keeps the tasks, the CAN frames and
 * the wheel setup sequence. The tunables below are deliberately static — the
 * console reaches them through steer_feel_console(), not through externs.
 */
#include "steer_feel.h"
#include "stm32f4xx_hal.h"
#include "g29_hid.h"
#include "powertrain.h"
#include "bringup.h"
#include <stdio.h>


/* ── Steering feel + hands-on detection (test) ──────────────────────────── */
/*
 * Two torques summed into the one constant-force channel the wheel gives us:
 *
 *   damper  — resistance proportional to how fast the wheel is being turned.
 *             It opposes *motion*, never position, so the wheel stays wherever
 *             the driver leaves it and simply feels heavy to move. A spring
 *             would haul it back to centre, which is not wanted here.
 *   dither  — a small alternating buzz on top, used to probe for hands. A free
 *             wheel swings with it; a gripped wheel damps it to nothing. That
 *             is how lane-keep systems infer hands-on-wheel without a
 *             capacitive rim sensor.
 *
 * Both are recomputed every FEEL_MS: velocity is meaningless without a fixed
 * sample interval, and a torque sent once at the flip would go stale.
 *
 * While enabled this owns the FFB channel — CAN 0x0B0 commands are ignored,
 * because two writers fighting over the same torque makes both useless.
 */
#define HOD_TEST        1

/*
 * Wheel-arrest gain applied while a probe is being cut short, in torque units
 * per widened-count-per-velocity-window. ~20 puts it near 2x the measured
 * natural damping (13.8 units per deg/s) — comfortably under the 3.11x that
 * self-oscillates. MAX bounds it so a noise spike in the velocity estimate can
 * never produce a jolt.
 */
#define HOD_ARREST_GAIN 20
#define HOD_ARREST_MAX  12000


/*
 * NOTE the scale. In C294 compat mode the wheel reports steering with only 8
 * bits, widened to 16 for the rest of the code, so ONE physical count is 257
 * widened counts. Gains are per widened count. Retune if the wheel ever
 * reaches native C24F mode, or the damper will be ~257x too weak.
 *
 * Velocity is sampled over FEEL_VEL_MS, not every FEEL_MS: at 8-bit resolution
 * and 100 Hz a normal turn moves less than one count per tick, so a
 * tick-to-tick difference is mostly zeros with occasional spikes.
 *
 * These are variables, not defines — the serial console tunes them live.
 *
 * The damper is a feedback loop closed over a coarse sensor and a ~60 ms delay,
 * so gain is bounded by stability, not by taste. Measured on this wheel:
 *   d=800 t=12000 -> force pinned at ±32767, wheel swings lock to lock
 *   d=300 t=6000  -> 444-count swing, still overshooting
 *   d=150 t=6000  -> 18-count swing, settles     <-- these values
 * Raise `d` past ~300 and it self-oscillates rather than damping.
 */
/*
 * Velocity deadband, below which car_feel renders nothing so the wheel can
 * hold position. 150 was under one 8-bit count, so in C294 it meant "any
 * movement at all". Native resolution makes the same 150 a real 2 deg per
 * window — 41 deg/s of steering with no damping at all before it engages.
 * 20 LSB restores the original intent at the new resolution.
 */
static int32_t feel_vel_dead  = 20;
/*
 * THIS IS THE FLOOR. Measured directly with 900 ms probes and a 350 ms ramp,
 * i.e. every advantage given to a weak push:
 *     4200 -> 2.0 counts of travel
 *     3600 -> 2.0 counts, but only SOMETIMES: at 6% over breakaway many
 *             probes failed to break stiction at all and read as "held"
 *     4600 -> reliable                            <-- here, margin matters
 *     3200 -> zero
 *     2800 -> zero
 *     2400 -> zero
 * Between 3200 and 3600 the wheel stops responding at all, matching the 3401
 * breakaway from the step-response fit. Lowering this further does not make the
 * nudge gentler, it makes hands-on detection impossible — a free wheel and a
 * held wheel both read zero movement.
 *
 * Perceived strength is dominated by onset rate, not peak torque, so any
 * remaining softening should come from hod_ramp_ms, not from here.
 *
 * This is now a CEILING, not the torque a probe applies. Only a wheel that
 * refuses to move ever sees it — see the seek below.
 *
 * Was 4000, which is BELOW this wheel's real breakaway (see WHEEL_BREAKAWAY)
 * and so could not move a free wheel at all: every probe measured 1-2 counts
 * of quantisation noise and reported HANDS ON with nobody on the wheel.
 * Remember the wire quantises to 256 units (g29_hid.c), so 4000 was really
 * delivering 3840.
 *
 * 5500 (delivered 5376) rather than the 7000 first tried, because the probe
 * only has to CLEAR breakaway, not tower over it, and everything above it is
 * spent accelerating a free wheel into a longer coast after the abort:
 *
 *     total excursion ~= threshold + peak_speed * (release_time + tau)
 *
 * and peak_speed goes with (delivered - breakaway), which 7000 nearly trebles
 * over 5500. So the swing shrank far more than the torque did, while the
 * free/held separation — the thing the extra torque was bought for — is
 * unchanged, since a held wheel does not move at either value.
 */
static int32_t hod_torque     = 5500;

/*
 * ── Seek the least torque that moves THIS wheel ─────────────────────────
 *
 * How far the nudge swings is
 *
 *     travel ~= hod_free_counts + speed_at_abort * (release + tau)
 *
 * and a flat 4000 push accelerates a free wheel toward w_ss(4000) = 43 deg/s
 * (nominal units, ~107 real) with tau = 50 ms, so the wheel is near full speed
 * by the time the threshold trips and the coast term is worth MORE than the
 * threshold. That is why lowering hod_free_counts stopped helping.
 *
 * Braking it back is not available on this drivetrain. The backlash is on the
 * FFB gears only — the angle sensor reads the rim directly, so the MEASUREMENT
 * is honest and it is the ACTUATOR that has the gap. Any braking torque is
 * opposite in sign to the push, so it must cross the whole lash gap before it
 * reaches the rim: late, and as a knock. The only quiet brake here is to stop
 * pushing and let the natural 13.8 units per deg/s take the speed out.
 *
 * So a probe creeps its torque up from a deliberately SUB-BREAKAWAY floor and
 * snaps straight back to that floor the moment the wheel moves. A free wheel
 * then walks to the threshold in ~1-count steps at barely over its own
 * breakaway, with almost no speed to coast on. A HELD wheel never moves, so the
 * torque climbs to hod_torque and stays there — the full-strength stimulus is
 * unchanged, which is what keeps the held/free separation and the
 * hod_free_counts calibration valid.
 *
 * Dropping torque never knocks (the teeth stay loaded on the same flank), so
 * the drop is immediate and only the rise is rate-limited.
 *
 * Bonus: it self-calibrates. Whatever this unit's real breakaway is, the seek
 * finds it on every probe instead of trusting the 3401 from the fit.
 */
static int32_t hod_creep = 4000;   /* MUST stay under the ~4200 breakaway, or
				    * the wheel never stops between steps */
#define HOD_SEEK  25       /* torque units per FEEL_MS tick while climbing */
/*
 * "Has the wheel moved yet", in a velocity window. Was 257 = one 8-bit count,
 * which in native mode is 3.5 deg — the wheel would have to be doing 70 deg/s
 * before the seek noticed and backed off, so it would never back off at all
 * and the probe would just shove at full torque. 20 LSB is 0.27 deg, clear of
 * the ~10 LSB of jitter seen on a still wheel.
 */
#define HOD_MOVED 20

/*
 * Lash take-up, and the other half of the backlash story.
 *
 * Probe direction alternates, so every probe starts by crossing the WHOLE lash
 * gap: the onset is spent on nothing and the teeth meet at whatever torque the
 * ramp had reached by then. That variance is the likeliest reason 3600 broke
 * stiction only SOMETIMES.
 *
 * So each probe opens by ramping to hod_creep over this long. Being under the
 * breakaway it cannot move the wheel however long it is held — which is exactly
 * what makes it free to take up the lash in silence.
 */
static int32_t hod_preload_ms = 150;

/*
 * Return-to-start deadband, widened counts. 500 is ~2 physical counts.
 *
 * Stopping one step short of the target is free; crossing it is not — an
 * overshoot means reversing the torque, and reversing is the one thing that
 * knocks the lash. So the return never corrects an overshoot, it just stops.
 *
 * 0 disables the return entirely.
 */
static int32_t hod_return_dead = 500;
#define RETURN_MS 1200         /* give up rather than shove at a held wheel */

#define FEEL_VEL_MS     50     /* velocity sample window */
#define HOD_WINDOW_MS   400    /* excursion is measured over this window */

/*
 * Sized against the seek, not by feel. A free wheel advances one count per
 * seek cycle — climb from hod_creep back over breakaway (~200 units at
 * HOD_SEEK, so ~80 ms) plus the ~50 ms velocity window that sees the step —
 * so roughly 3 counts per 500 ms. hod_free_counts (700 = 2.7 counts) is
 * therefore reached in well under half a probe, with plenty of margin before
 * the probe times out and the wheel is declared held.
 *
 * Shorten this and a free wheel can run out of probe before it has crept far
 * enough, which reads as HANDS ON with nobody there.
 */
/*
 * 2000, not 1600, because the seek climbs at a fixed HOD_SEEK per FEEL_MS tick
 * (2500 units/s) and now has further to go: from hod_creep to hod_torque is
 * 3000 units = 1200 ms, against a usable window of probe_ms minus the pre-load
 * and the tail ramp. At 1600 the ceiling was simply unreachable — the seek ran
 * out of probe before it ran out of climb.
 */
static int32_t hod_probe_ms  = 2000;
/*
 * Probe once per this interval.
 *
 * 8000 was chosen when probes only ran at a standstill, where a stationary
 * wheel is not about to change who is holding it. Now that they run while
 * driving straight (PROBE_MIN_KMH), 8 s may be slower than you want a hands-on
 * monitor to notice a change. Left alone because it is the least intrusive
 * default and 'i' tunes it live — drop it once you have watched p2p= at speed.
 */
static int32_t hod_period_ms = 8000;
static int32_t hod_ramp_ms   = 300;    /* release ramp at the END of a probe that
					* ran its full length. The onset is the
					* pre-load + seek now, not a ramp. */
static int32_t hod_abort_ms  = 40;     /* release time once a probe is aborted */

/*
 * The wheel must be still this long before a probe may start, and any steering
 * input restarts the clock. Two reasons:
 *   - probing into live steering is unmeasurable (a hand moving WITH the nudge
 *     looks exactly like a free wheel from position alone), and
 *   - a driver who is actively steering has demonstrably got hands on it, so
 *     there is nothing to ask.
 */
static int32_t hod_idle_ms   = 2000;
/*
 * Over-speed tell. The identified plant says a FREE wheel under probe torque u
 * settles at (0.07253*u - 246.7) deg/s — only ~43 deg/s at u=4000, which is
 * 1.5 counts per 50 ms sample. A hand deliberately following the nudge moves
 * far faster than that. So "moved a lot" is not enough to call it free; it must
 * have moved at roughly the speed a free wheel WOULD have. Anything faster is
 * being driven by something, and that something is a hand.
 * Percent of predicted free speed above which we call it hands-on.
 */
static int32_t hod_overspeed = 180;    /* 180 = 1.8x the predicted free speed */
static int32_t hod_move_cnt  = 900;    /* widened counts that count as steering */

/*
 * Torque for the hands-on probe, and whether a probe is currently running.
 *
 * The noise is BACKLASH, not the waveform. Every time torque crosses zero the
 * gear teeth cross the lash gap and knock, and a buzz crosses zero dozens of
 * times a second. So this probe never changes sign: it is one smooth push,
 * ramped in and out, with the direction alternating between probes so repeated
 * probes cancel and the wheel does not walk away.
 *
 * It also only runs for hod_probe_ms out of every hod_period_ms — silent ~90%
 * of the time. Real lane-keep systems probe periodically for the same reason.
 */
static uint32_t hod_last_move;   /* tick of the last driver steering input */

/*
 * Explicitly triggered rather than free-running on `now % period`, so a probe
 * can be held off while the driver is actually steering. Probing into live
 * steering input is what made "moving my hand with the nudge" read as hands
 * off: the wheel travels far, which is exactly the free-wheel signature, and
 * position alone cannot tell a free wheel from one a hand is helping along.
 *
 * A probe only starts once the wheel has been still for hod_idle_ms. Once
 * started it runs to completion (or is cut short by the abort).
 */
/*
 * Highest torque the seek reached during the current probe, and how long the
 * probe took to reach hod_free_counts. DIAGNOSTIC ONLY — nothing votes on
 * these yet.
 *
 * The excursion verdict has stopped carrying information: at native
 * resolution hod_free_counts is ~2 deg, which is inside the compliance of an
 * arm holding the wheel, so held and free BOTH cross it and p2p reads ~161
 * either way. The overspeed tiebreak is dead too — it compares against
 * hod_free_speed()'s 151 deg/s while the seek holds the wheel to 2-4 deg/s by
 * design.
 *
 * These are the two candidates to replace it, and which one separates real
 * hands is a question about arms and gearboxes, not about code:
 *   peakT — a free wheel breaks loose near breakaway and the seek never needs
 *           more; a held wheel drives the seek to hod_torque. Degrades if a
 *           hand gives in small increments, since each one resets the seek.
 *   t2thr — a free wheel crosses the threshold quickly, a compliant hand
 *           takes far longer. Degrades if the hand is loose enough to move
 *           with the nudge.
 * Watch both, held and free, then wire up whichever actually splits.
 */
static int32_t  hod_peak_t;    /* max seek torque this probe */
static uint16_t hod_t2thr;     /* ms from probe start to the abort, 0 = never */

/*
 * The one push primitive, unsigned: ramp to the creep floor over
 * hod_preload_ms (the lash take-up), then seek up while the wheel is still and
 * snap back to the floor the moment it moves.
 *
 * Both users are this and differ only in what ends them — the probe stops on
 * excursion, the return stops on position.
 */
static int32_t hod_push(uint32_t phase, bool moving, int32_t *peak)
{
	uint32_t pre = (uint32_t)(hod_preload_ms < 0 ? 0 : hod_preload_ms);

	/* Lash first. Seeking while the teeth are not yet engaged is measuring
	 * nothing, and it would leave a step at the handover. Unreachable when
	 * pre == 0, so the divide is safe. */
	if (phase < pre) {
		int32_t base = (*peak < hod_creep) ? *peak : hod_creep;

		return base * (int32_t)phase / (int32_t)pre;
	}

	if (moving) {
		*peak = hod_creep;
	} else if (*peak < hod_torque) {
		*peak += HOD_SEEK;
		if (*peak > hod_torque) { *peak = hod_torque; }
	}
	return *peak;
}

/* Hoisted out of hod_probe so an abort can end the probe on the spot rather
 * than idling out the rest of the window — the return wants to start now. */
static bool hod_running;

static int32_t hod_probe(uint32_t now, bool moving, bool *active, bool inhibit)
{
	static uint32_t t_start, t_last;
	static int8_t dir = 1;
	static int32_t peak;
	uint32_t period = (uint32_t)(hod_period_ms < 200 ? 200 : hod_period_ms);
	uint32_t plen   = (uint32_t)(hod_probe_ms  <  50 ?  50 : hod_probe_ms);
	uint32_t phase, r;
	int32_t t;

	if (!hod_running) {
		bool due  = (now - t_last >= period);
		bool idle = (now - hod_last_move >= (uint32_t)hod_idle_ms);

		/*
		 * `inhibit` is set when the centring spring is strong enough to move
		 * the wheel on its own. Two reasons not to probe then, and the first
		 * is what the driver actually notices:
		 *
		 *   - Probing suppresses the centring torque (it has to, or the
		 *     spring fights the probe). Holding the wheel at an angle reads
		 *     as "still", which is exactly what arms a probe -- so holding a
		 *     steady cornering angle made the steering go slack every few
		 *     seconds. That is the bug this fixes.
		 *   - It is redundant. A spring that can move the wheel IS a
		 *     stimulus, so hands-on can be read off whether the wheel
		 *     returns, with nothing intrusive added. See the spring test in
		 *     hod_update().
		 */
		if (due && idle && !inhibit) {
			hod_running = true;
			t_start = now;
			t_last = now;
			peak = hod_creep;     /* every probe starts from the floor */
			dir = (int8_t)-dir;   /* alternate, so probes do not walk the wheel */
		}
		*active = false;
		return 0;
	}

	phase = now - t_start;
	if (phase >= plen) {
		hod_running = false;
		*active = false;
		return 0;
	}
	*active = true;

	t = hod_push(phase, moving, &peak);
	if (peak > hod_peak_t) { hod_peak_t = peak; }

	/* Release ramp at the tail of a probe that ran its full length. What
	 * used to be the rising half of the trapezoid IS the seek now. */
	r = (uint32_t)(hod_ramp_ms < 0 ? 0 : hod_ramp_ms);
	if (r > plen / 2U) { r = plen / 2U; }
	if (r > 0U && phase > plen - r) {
		t = t * (int32_t)(plen - phase) / (int32_t)r;
	}

	return t * dir;
}

/*
 * Threshold on the excursion measured during a probe, in widened counts (257
 * per real 8-bit step). MUST be calibrated per unit: watch p2p= with the wheel
 * free, then held, and put this between the two. Measured free-wheel p2p on
 * this unit: ~2500 at t=6000, and 4000..45000 at t=12000-20000. Retune this
 * whenever hod_torque or hod_probe_ms changes — they set the stimulus.
 */
/*
 * The fix is not more torque, it is more TIME. A held wheel moves ~1 count no
 * matter how long you push; a free one keeps travelling. From the identified
 * model, 4000 units for 1200 ms moves a free wheel ~22 counts, so a threshold
 * here clears held-wheel noise by a wide margin — and lets the torque come DOWN.
 *
 * HOW FAR THE NUDGE SWINGS IS SET HERE, because the probe aborts the moment the
 * excursion crosses this. That makes the value range-dependent in a way it was
 * not when this was tuned: it is in COUNTS, and unlocking the wheel to 900°
 * made one 8-bit count 3.5° instead of 0.7°. The old 2000 (7.8 real counts)
 * therefore went from a ~5.5° nudge to a ~27° one — "the nudge moves the wheel
 * a lot".
 *
 * 900 is ~3.5 real counts, so the nudge is back to ~12° while still clearing a
 * held wheel's ~1 count by 3-4x. Going much lower re-enters the quantisation
 * floor where free-vs-held is a coin flip (400 = 1.5 counts was exactly that,
 * and showed up as hands-on reported as hands-off). At 900° over 8 bits, a
 * position-excursion test simply cannot be both reliable and invisible — so the
 * probe is confined to the one case nothing else covers, see PROBE_MIN_KMH.
 *
 * Still worth calibrating per unit: watch p2p= free vs held and sit between.
 */
/*
 * 75 = ~1 deg of travel before the probe aborts.
 *
 * Turned down from 150 on request, with the excursion verdict already known to
 * be saturated at that size: p2p reads ~161 whether the wheel is held or free,
 * because an arm's compliance is wider than the threshold, so both cross it.
 * Halving it does not make that worse in kind — the verdict was carrying no
 * information at 150 either — but do not expect hands-ON to improve until the
 * verdict moves onto peakT or t2thr. See the diagnostics on the HANDS line.
 *
 * The old floor is GONE. In C294 this could not go under ~514 (7 deg): one
 * 8-bit count was 3.5 deg, a held wheel rattled +-1 count, and anything
 * tighter was reading quantisation noise — free-vs-held became a coin flip
 * and the probe was a shove you could feel from the passenger seat. Native
 * mode makes one LSB 0.0137 deg, so 150 still clears the noise by an order of
 * magnitude while injecting a quarter of the steering.
 *
 * Now genuinely a matter of taste, so tune it with 'h'. Watch p2p= free vs
 * held and sit between the two, as ever; there is simply a lot more room
 * between them than there used to be.
 */
static int32_t hod_free_counts = 75;
static int32_t hod_confirm     = 2;      /* probes that must agree to flip state */

static uint16_t hod_p2p;       /* last measured peak-to-peak, for the log */
static int16_t  feel_force;    /* last torque actually applied, for the log */
static int16_t  feel_vel;      /* last measured counts/tick, for the log */
static int16_t  feel_centre;   /* last self-aligning torque, for the log */

/*
 * Resistance opposing motion. vel is counts of travel per FEEL_MS tick, signed
 * — positive means turning right, and force is positive for right torque, so
 * the sign inverts to push back against whichever way it is moving.
 * Stop moving and this returns 0, which is what lets the wheel stay put.
 */
/* ── Car steering-feel impedance model ──────────────────────────────────── */
/*
 * Identified from step response ('S' command, 8 torque levels, fitted offline).
 * Over the linear regime (u <= 14000, w < 800 deg/s), R^2 = 0.9998:
 *
 *     w_ss(u) = 0.07253*u - 246.7   deg/s
 *     natural viscous b   = 13.8 torque units per deg/s
 *     breakaway (stiction)= 3401 units = 10.4% of full scale
 *     tau = J/b           = 50 ms
 *     saturates above ~800 deg/s (motor back-EMF), slope falls to 0.0252
 *
 * The model predicts its own validation point: 3401 breakaway is why the
 * 3000-unit step produced no motion at all.
 *
 * A car resists being steered, at parking speed, mostly through tyre scrub —
 * a roughly constant torque opposing whichever way you turn — plus viscous
 * damping from the steering system. So render
 *
 *     u = -[ Tc_car*sgn(w) + b_car*w ]
 *
 * in the SAME torque units the identification used, which is what makes these
 * numbers physical instead of arbitrary gains. car_visc is quoted against the
 * measured natural 13.8, so 138 means "double the wheel's own damping".
 *
 * sgn() is blended over car_w_eps rather than a hard sign.
 *
 * STICK-SLIP. The Coulomb term is a function of velocity, and velocity here is
 * quantised to whole 8-bit counts per FEEL_VEL_MS window — 28 deg/s per step.
 * Turning slowly, the raw estimate alternates 0 / 28 / 56 deg/s, so an
 * unsmoothed Coulomb term snaps its full magnitude on and off with it and the
 * wheel judders: resistance appears, stalls the motion, disappears, repeat.
 * Two things prevent it, and both matter:
 *   - the velocity estimate is low-pass filtered (FEEL_VEL_SHIFT), and
 *   - car_w_eps is wide enough that one quantisation step is a small force
 *     step: at 220 deg/s one count moves the Coulomb term by only
 *     car_coulomb*28/220, about an eighth of it, instead of all of it.
 * Raise car_w_eps ('e') if any judder remains; lower it for a crisper edge.
 */
#define CAR_B_NATURAL  138     /* 13.8 units per deg/s, x10 for integer maths */
#define FEEL_VEL_SHIFT 2       /* IIR: vf += (v - vf) >> 2 */

static int32_t car_coulomb = 3500;   /* tyre-scrub torque, in torque units */
static int32_t car_visc    = 138;    /* x10 units per deg/s; 138 = 1.0x natural */
static int32_t car_w_eps   = 220;    /* deg/s over which sgn() is blended */

/*
 * ── Speed-dependent feel (rendered by powertrain.c) ─────────────────────
 *
 * Self-aligning torque at FULL LOCK, once up to speed. Note this deliberately
 * REVERSES the "damper, not a spring" rule below the parking-speed model was
 * built on — and that is the physically correct thing to do: a parked car's
 * wheel stays where you leave it (speed 0 -> this term is 0), a moving car's
 * wheel pulls straight, harder the faster it goes.
 *
 * Sized against the measured 3401 breakaway: at 12000, the wheel only
 * self-returns from beyond roughly 60° off centre, and closer in the spring is
 * felt but cannot overcome its own stiction. Raising it much past ~16000 risks
 * oscillation — this is a POSITION loop closed over a coarse 8-bit sensor and
 * a ~60 ms delay, the same stability limit that caps car_visc at ~300.
 * 0 restores the old pure-damper behaviour.
 *
 * A LINEAR spring alone cannot both self-return at small angles and stay
 * civilised at full lock here, because stiction is 10.4% of full scale: with
 * strength 12000 over 450° of lock, self-return needed 12000*off/32768 > 3401,
 * i.e. ~127° before the wheel moved AT ALL. That is the "no centring between
 * -90 and +90, and weak" complaint, and no value of this knob fixes it —
 * raising it enough to return at 20° (>30000) is brutal at full lock.
 *
 * So the spring provides the WEIGHT and car_stiction below provides the
 * BREAKAWAY. Tune this for how heavy the steering feels.
 */
static int32_t car_centre = 16000;

/*
 * Stiction compensation — the term that actually makes the wheel return.
 *
 * The wheel does not move until torque exceeds ~3401, so a physically correct
 * spring is simply invisible below that. This adds sign(centre)*car_stiction
 * on top whenever the wheel is off centre, which lifts even a small spring
 * force over the breakaway and lets it return from any angle.
 *
 * It is FADED OUT as the wheel starts moving (STICTION_VEL_REF): once sliding,
 * friction is lower and continuing to shove would overshoot centre and hunt.
 * That taper, plus CENTRE_DEADBAND zeroing the whole thing near centre, is
 * what keeps it from oscillating.
 *
 * Default is the measured breakaway, so centre+comp clears stiction for any
 * non-zero spring force. Lower it if the return feels like it snaps; 0 reverts
 * to the pure linear spring (and its dead zone).
 *
 * 4600, not WHEEL_BREAKAWAY itself, because the wire quantises to 256-unit
 * levels: 4200 rounds DOWN to 4096, which is under breakaway and leaves the
 * wheel parked off centre with the spring straining at it — observed as the
 * wheel sitting at +51 deg, ctr=-3936, vel=0. One level of headroom is the
 * difference between "the spring can move it" being an assertion and being
 * true, and centre_can_move stakes the whole spring test on that claim.
 */
static int32_t car_stiction = 4600;

/* Wheel speed, in the same widened-counts-per-window unit as feel_vel, at
 * which stiction compensation has faded to nothing. */
#define STICTION_VEL_REF 500

/*
 * Spring-as-stimulus hands-on test (see hod_update). Window over which the
 * wheel is watched for closing on centre, and how much closer it must get to
 * count as "the spring won, so nobody is holding it".
 *
 * 200 counts is ~2.7 deg. Was 900 (12.3 deg) to clear the 8-bit quantisation
 * floor, the same reason hod_free_counts was pinned high; native resolution
 * retires that constraint for both. It matters that this came down alongside
 * CENTRE_DEADBAND: the ON verdict needs SPRING_RETURN_CNT + CENTRE_DEADBAND of
 * room (see hod_update), and between the deadband and that sum NOTHING votes —
 * the probe is inhibited and the spring test abstains. At 900 + 600 that blind
 * band ran from 8 to 20 deg off centre. At 200 + 150 it is 2 to 4.8 deg.
 *
 * The window is a DEADLINE for cumulative closure, not a rate. It was 400 ms
 * with the reference reset every window, which made the test "did it close
 * 900 counts in the last 400 ms" = "is it moving faster than ~31 deg/s". A
 * free wheel breaking away from rest is slower than that for its first second,
 * so switching the spring on at a big angle reported HANDS ON for ~2 s while
 * the wheel was visibly returning on its own. Closure now accumulates from
 * where the test armed and only the ON verdict waits for the deadline, so a
 * slow return still gets there.
 *
 * Cost: a genuinely held wheel takes hod_confirm deadlines (~3 s) to report
 * ON, up from 800 ms. Tune here if that is too slow — but the cheap direction
 * is latency on the ON verdict, not false ONs with nobody on the wheel.
 */
#define SPRING_TEST_MS    1500U
#define SPRING_RETURN_CNT 200

/*
 * BELOW this road speed the intrusive probe is switched off entirely.
 *
 * The question the probe answers only matters while the car is moving: nobody
 * is hurt by letting go of a parked car's wheel, so a nudge at a standstill is
 * an annoyance that buys nothing.
 *
 * What is left above it is narrow, and deliberately so. Off centre at speed the
 * centring spring can move the wheel on its own, which sets `centre_can_move`
 * and inhibits the probe anyway — the spring test reads hands-on from the
 * wheel's own behaviour with nothing injected. So the probe now fires in
 * exactly the gap that test cannot cover: driving straight, on centre, where
 * the spring gives no signal and the last verdict would otherwise just persist.
 *
 * This does mean the steering of a MOVING car gets nudged, which the earlier
 * standstill-only rule existed to prevent. Two things make it defensible now:
 * the seek keeps the excursion to hod_free_counts instead of threshold-plus-
 * coast, and the return puts the wheel back where it found it. Watch p2p= at
 * speed before trusting it, and raise this if the gap is too wide for comfort.
 */
#define PROBE_MIN_KMH 10

/*
 * Road rumble amplitude, once up to speed. Kept well under the 3401 breakaway
 * on purpose: it should be FELT through a gripped wheel, not drive the wheel
 * around by itself.
 *
 * Caveat worth knowing before turning this up: this wheel's gear lash knocks
 * every time torque crosses zero, which is exactly why the hands-on probe is a
 * single-sign ramped pulse rather than a buzz (see hod_probe()). A rumble
 * crosses zero constantly and so reintroduces some of that noise. It is
 * slew-limited to ~8 Hz to keep the edges soft, but if the mechanical
 * chattering bothers you, this is the knob to back off — `u 0` disables it.
 */
/*
 * 1200, down from 2500. The rumble generator itself is unchanged by the move
 * to native mode — it is speed-scaled filtered noise with no sensor input —
 * but what the wheel does with it changed. At 8-bit resolution the motion it
 * induced was under one count, so feel_vel read zero and neither the damper
 * nor the Coulomb term responded. Native resolution makes that motion visible
 * and they now answer every ripple, which is felt as more vibration for the
 * same commanded amplitude. 'u' tunes it live; `u 0` disables it.
 */
static int32_t car_rumble = 1200;

/*
 * Breakaway (stiction) — below this the wheel does not move at all. Used to
 * decide whether the centring spring is strong enough to turn the wheel on its
 * OWN, which changes what wheel movement is evidence of. See hod_update().
 *
 * 3401 came out of the step-response FIT, and it was wrong by ~25%. Observed
 * directly, off the centring spring alone, with nobody touching the wheel:
 *
 *     ctr=-3936 at +51 deg  ->  vel=0 for three seconds  (delivered 3840)
 *     ctr=+4143 at -128 deg ->  vel=+1, one count        (delivered 4096)
 *     ctr=+4426 at -111 deg ->  vel=+132 then +390       (delivered 4352)
 *
 * Note "delivered": g29_send_constant_force() quantises to 256-unit wire
 * levels, so what the motor sees is the value rounded DOWN to a multiple of
 * 256. Any constant near breakaway has to clear it after that rounding.
 *
 * Understating this broke both hands-on mechanisms at once — the probe pushed
 * below breakaway and measured nothing, and centre_can_move asserted the
 * spring could move a wheel it demonstrably could not, so the spring test ran
 * on a stationary wheel and read "held". Re-measure per unit the same way if
 * the gearbox is ever opened.
 */
#define WHEEL_BREAKAWAY 4200

/*
 * Startup centring: torque, and how long to keep trying. 6500 clears the 4200
 * breakaway with a level of wire quantisation to spare without being a shove;
 * 5 s is enough to walk in from full lock and is the backstop that stops it
 * pushing forever against a hand that happens to be on the wheel at boot.
 */
#define STARTUP_CENTRE_T   6500
#define STARTUP_CENTRE_MS  5000U

/*
 * Centring deadband. Inside it the spring reads exactly zero; without it,
 * sensor quantisation makes the torque hunt back and forth across centre — the
 * same stick-slip failure described for the Coulomb term above.
 *
 * 600 was ~2 counts of an 8-bit sensor, i.e. 8.2 deg of dead zone at centre,
 * which is a lot of slop to accept purely to dodge quantisation. Native
 * resolution buys it back: 150 is 2 deg and is still ~15x the observed jitter.
 *
 * It doubles as the probe window — the nudge only fires inside this band, see
 * probe_inhibit — so shrinking it also confines the nudge to nearer centre.
 */
#define CENTRE_DEADBAND 150

/* widened counts per FEEL_VEL_MS window -> deg/s */
/*
 * CAUTION, the "deg" here is NOMINAL, not real degrees. The 0.005472 below
 * assumes 360° of travel, but the wheel was since measured to give the full
 * 900° (steer= reads +-449 at the stops), so one count is really 0.01368° and
 * every value this returns is ~2.5x too small to be a true deg/s.
 *
 * It is left alone DELIBERATELY, because it is self-consistent: the step
 * response identification ('S') was fitted through this same conversion, so
 * w_ss(u), car_visc, car_w_eps and hod_free_speed() are all expressed in the
 * same nominal unit and the factor cancels everywhere it is used. Scaling this
 * without re-running the identification and rescaling those constants by the
 * same 2.5 would silently detune the damper and break the hands-on overspeed
 * test.
 *
 * So: treat these numbers as internal units. If you ever re-run the fit, fix
 * BOTH together.
 */
static int32_t vel_to_deg_s(int32_t vel)
{
	/* one widened count = 360/256/257 deg = 0.005472 deg (see caution above) */
	return vel * 5472 / (1000 * FEEL_VEL_MS);
}

/*
 * `probing` suppresses the tyre-scrub term. The probe has to overcome the
 * wheel's own 3401 breakaway; if the simulated 3500 of scrub is also fighting
 * it, a 4000-unit probe nets ~500 and the wheel barely moves — which reads as
 * "hands on" with nobody touching it. The probe is a measurement of the wheel
 * plus the driver's grip, so the simulated car must step out of the way for it.
 * Viscous damping stays: it is small at probe speeds and keeps the transition
 * from feeling hollow.
 */
static int32_t car_feel(int32_t vel, bool probing)
{
	int32_t w, s;

	if (vel > -feel_vel_dead && vel < feel_vel_dead) {
		return 0;          /* not moving: hold position, apply nothing */
	}
	w = vel_to_deg_s(vel);
	s = w * 1024 / car_w_eps;
	if (s >  1024) { s =  1024; }
	if (s < -1024) { s = -1024; }

	return -((probing ? 0 : car_coulomb) * s / 1024 + car_visc * w / 10);
}

/*
 * Static-friction feedforward for the centring spring. Pushes in the same
 * direction the spring already wants to go, so centre+boost clears the wheel's
 * ~3401 breakaway even when the spring alone is far below it.
 *
 * Faded out with wheel speed: at rest it is full (break the wheel loose), and
 * by STICTION_VEL_REF it is gone (the wheel is already sliding, so more shove
 * just overshoots centre and starts a limit cycle).
 */
static int32_t stiction_boost(int32_t centre, int32_t vel)
{
	int32_t a = (vel < 0) ? -vel : vel;
	int32_t scale;

	if (car_stiction <= 0 || centre == 0) {
		return 0;
	}
	if (a >= STICTION_VEL_REF) {
		return 0;
	}
	scale = (STICTION_VEL_REF - a) * 1024 / STICTION_VEL_REF;

	return ((centre > 0) ? car_stiction : -car_stiction) * scale / 1024;
}

/*
 * A single probe is not trustworthy. The probe torque sits close to the wheel's
 * measured 3401 breakaway, so some probes fail to break stiction at all and
 * return "no movement" — which is indistinguishable from a firmly held wheel.
 * That produced a state that alternated OFF/ON every probe with hands nowhere
 * near it (p2p=514, then 257, then 0).
 *
 * So require hod_confirm consecutive probes to agree before changing state.
 * A verdict that disagrees with the pending one restarts the count rather than
 * flipping immediately.
 *
 * The latched verdict lives at file scope, not in a local static, because the
 * standstill override has to be able to CLEAR it. It used to only force the
 * returned hands_on false while leaving this latched at true, so the first
 * vote after pulling away handed the stale true straight back — pressing the
 * throttle reported HANDS ON within a second or two, with no probe having
 * confirmed anything. The override has to reset the machine, not mask it.
 */
static bool hod_state;

static bool hod_vote(bool verdict, int8_t *votes, int8_t *pending)
{
	if (*pending != (int8_t)verdict) {
		*pending = (int8_t)verdict;
		*votes = 1;
	} else if (*votes < 100) {
		(*votes)++;
	}
	if (*votes >= (int8_t)hod_confirm) {
		hod_state = verdict;
	}
	return hod_state;
}

/* Predicted steady speed of a FREE wheel at the current probe torque, deg/s.
 * w_ss = 0.07253*u - 246.7, from the step-response fit (R^2 = 0.9998). */
static int32_t hod_free_speed(void)
{
	int32_t w = (7253 * hod_torque) / 100000 - 247;

	return (w < 5) ? 5 : w;
}

static bool hod_update(uint16_t steering)
{
	static uint32_t t_feel, t_vel;
	static uint16_t lo = 0xFFFF, hi, prev_steer;
	static int32_t vel_filt, vel_raw;
	static bool hands_on, primed, was_probing, aborted;
	static int8_t pending = -1, votes;
	static uint16_t idle_ref;
	static uint32_t t_move_vote;
	static int32_t probe_peak_w;
	static int8_t last_reported = -1;   /* -1 = nothing reported yet */
	static uint32_t t_abort, t_probe0;
	static bool centred;            /* startup centring done */
	static uint32_t t_centre0;      /* when it started, for the timeout */
	static int32_t abort_from;
	static uint32_t t_spring;	/* spring-test window start */
	static int32_t spring_ref;	/* offset at the start of that window */
	static uint16_t probe_ref;	/* where the probe found the wheel */
	static uint32_t t_return;
	static int32_t ret_peak;
	static int8_t ret_dir;
	static bool returning, ret_wanted;
	bool probing = false;
	int32_t probe_torque;
	int32_t centre_torque, centre_off;
	bool centre_can_move, probe_inhibit, moving, standstill;
	uint32_t now = HAL_GetTick();

	/*
	 * Re-prime on the first call and after any gap (wheel unplugged, or the
	 * setup sequence running): a stale prev_steer would read as an enormous
	 * velocity and slam the wheel with a full-scale kick.
	 */
	if (!primed || now - t_feel > 500U) {
		t_feel = t_vel = now;
		prev_steer = steering;
		idle_ref = steering;
		vel_filt = vel_raw = 0;
		primed = true;
		centred = false;      /* a fresh wheel is wherever it powered up */
		t_centre0 = now;
		return false;
	}

	/* Velocity on its own slower cadence — see FEEL_VEL_MS note above. */
	if (now - t_vel >= FEEL_VEL_MS) {
		int32_t raw = (int32_t)steering - (int32_t)prev_steer;

		/* IIR at x16 so the fraction survives integer maths. Without this
		 * the estimate is whole counts only and the Coulomb term chatters
		 * on and off with the quantisation — felt as stick-slip. */
		vel_filt += ((raw << 4) - vel_filt) >> FEEL_VEL_SHIFT;
		feel_vel = (int16_t)(vel_filt >> 4);
		/*
		 * Unfiltered too. The IIR above is ~4 windows = 200 ms of lag,
		 * fine for rendering a damper and useless for "has it moved
		 * YET", which the seek has to answer inside one window.
		 */
		vel_raw = raw;
		prev_steer = steering;
		t_vel = now;
	}

	/*
	 * ── Startup centring ────────────────────────────────────────────────
	 *
	 * The wheel powers up wherever its own calibration sweep left it —
	 * steer=-382 on this unit — and nothing brings it back, because the
	 * simulated self-aligning torque is deliberately zero at a standstill
	 * (a parked car's wheel stays where you leave it). So the car started
	 * with the steering wound most of the way to full lock.
	 *
	 * A plain constant push rather than the seek: the seek creeps a count
	 * at a time to keep the hands-on probe quiet, and from 382 deg out it
	 * would still be crawling minutes later. Nothing is being measured
	 * here, so it can just drive.
	 *
	 * Damped by the same velocity term the probe abort uses, so it stops
	 * AT centre instead of winding up on the far side and hunting back.
	 * Ends on the deadband or on the timeout, whichever comes first — the
	 * timeout is what stops it shoving indefinitely if someone is holding
	 * the wheel while it tries.
	 */
	if (!centred) {
		int32_t off = (int32_t)steering - 32768;
		int32_t a   = (off < 0) ? -off : off;

		if (a <= CENTRE_DEADBAND || now - t_centre0 >= STARTUP_CENTRE_MS) {
			centred = true;
		} else {
			int32_t t = (off > 0) ? -STARTUP_CENTRE_T : STARTUP_CENTRE_T;

			t -= (int32_t)feel_vel * HOD_ARREST_GAIN;
			if (t >  32767) { t =  32767; }
			if (t < -32767) { t = -32767; }
			if (now - t_feel >= FEEL_MS) {
				feel_force = (int16_t)t;
				g29_send_constant_force(feel_force);
				t_feel = now;
			}
			return false;
		}
	}

	/*
	 * Self-aligning torque for this position and speed. Computed every call,
	 * not just on the FEEL_MS tick, because the hands-on inference below
	 * needs to know whether the spring is strong enough to move the wheel.
	 */
	centre_off = (int32_t)steering - 32768;
	if (centre_off > -CENTRE_DEADBAND && centre_off < CENTRE_DEADBAND) {
		centre_off = 0;
	}
	centre_torque = powertrain_self_centre(centre_off, car_centre);
	/* The boost is part of what the wheel actually gets, so it counts toward
	 * "can this move the wheel unaided" — with compensation enabled that is
	 * true for any non-zero spring force, which is the whole point. */
	centre_torque += stiction_boost(centre_torque, feel_vel);
	centre_can_move = (centre_torque >  WHEEL_BREAKAWAY ||
			   centre_torque < -WHEEL_BREAKAWAY);
	feel_centre = (int16_t)centre_torque;

	/* Read the speed straight from the model rather than via a shared copy:
	 * this runs in the same task that ticks it, so it is always current. */
	{
		struct powertrain_state pt;

		powertrain_get_state(&pt);
		probe_inhibit = centre_can_move ||
				pt.speed_kmh < PROBE_MIN_KMH;
		standstill = (pt.speed_kmh == 0U);
	}

	/*
	 * Evaluated every call, NOT inside the FEEL_MS block below: this loop runs
	 * at kHz, so gating `probing` on the 10 ms torque tick left it false on
	 * almost every pass and reset the measurement window continuously — the
	 * excursion then measured one sample and always read zero.
	 */
	/*
	 * Steering input while no probe is running is positive evidence: a free
	 * wheel does not turn itself, so someone is holding it. It also restarts
	 * the idle clock, which is what keeps a probe from firing into live input.
	 *
	 * ...EXCEPT once the centring spring is in play, because then a free wheel
	 * DOES turn itself — that premise only held while the model was a pure
	 * damper. Whenever the self-aligning torque exceeds the wheel's own
	 * breakaway it can drive the wheel unaided, so movement stops being proof
	 * of a hand and the vote has to be skipped or it reports HANDS ON with
	 * nobody touching the wheel. The idle clock is still restarted either way:
	 * a wheel that is moving, for whatever reason, is not a wheel to probe.
	 */
	/* `returning` is excluded for the same reason `was_probing` is: the
	 * firmware moved the wheel, so it is not evidence of a hand. */
	if (!was_probing && !returning) {
		int32_t d = (int32_t)steering - (int32_t)idle_ref;

		if (d > hod_move_cnt || d < -hod_move_cnt) {
			idle_ref = steering;
			hod_last_move = now;
			if (now - t_move_vote >= 200U && !centre_can_move) {
				t_move_vote = now;
				hands_on = hod_vote(true, &votes, &pending);
			}
		}
	} else {
		idle_ref = steering;      /* probe motion is not driver input */
		hod_last_move = now;      /* and the idle clock restarts after it */
	}

	/*
	 * Spring-as-stimulus hands-on test, used INSTEAD of a probe whenever the
	 * centring torque can move the wheel unaided.
	 *
	 * No extra torque is injected: the spring is already pulling toward
	 * centre, so the wheel's own behaviour is the measurement. A free wheel
	 * walks back; a held wheel sits at whatever angle the hand chose. That
	 * removes the intrusive probe exactly where it was most annoying (a
	 * steady cornering angle) and keeps the centring live while it does it.
	 */
	if (centre_can_move && !probing && !returning) {
		int32_t aoff = (centre_off  < 0) ? -centre_off  : centre_off;
		int32_t aref = (spring_ref  < 0) ? -spring_ref  : spring_ref;

		/*
		 * Crossing centre — or landing on it — settles the question
		 * outright, whatever |offset| did. The spring overshoots (that
		 * is what the stiction feedforward is FOR), and a swing from
		 * -51 deg to +54 deg leaves aref - aoff negative, so the
		 * magnitude test alone read a wheel the spring had visibly
		 * thrown across centre as "did not close" and voted HANDS ON.
		 */
		bool crossed = (spring_ref > 0 && centre_off <= 0) ||
			       (spring_ref < 0 && centre_off >= 0);

		/*
		 * Moving AWAY from centre, against a spring that is by now well
		 * over breakaway, is the one unambiguous reading this test has.
		 * The spring cannot do it, so something else did, and the only
		 * something else is a hand. Take it immediately rather than
		 * waiting out the window.
		 *
		 * Without this the closure test had to carry the whole verdict
		 * at an angle, and it cannot: turning further into a corner
		 * looks identical to the spring failing to return the wheel,
		 * which is why holding a steady angle flapped between ON and
		 * OFF. The gain is asymmetric on purpose — this only ever votes
		 * ON, so a genuinely free wheel, which by definition is being
		 * pulled toward centre and not away from it, never reaches it.
		 *
		 * Threshold is the same SPRING_RETURN_CNT the closure test uses,
		 * so "moved away" and "moved back" mean the same distance.
		 */
		if (aoff - aref >= SPRING_RETURN_CNT && !crossed) {
			hands_on = hod_vote(true, &votes, &pending);
			t_spring = now;
			spring_ref = centre_off;
		} else if (crossed) {
			/*
			 * Reached centre, or went through it. The spring
			 * COMPLETED the job, so nothing is holding it.
			 *
			 * Note this is now the only way to a hands-off verdict
			 * here, and deliberately so — see the abstain below.
			 * Judged as soon as it happens, however long it took: a
			 * wheel breaking away from rest takes about a second to
			 * get going, and calling that "held" was the false ON at
			 * a big angle.
			 */
			hands_on = hod_vote(false, &votes, &pending);
			t_spring = now;
			spring_ref = centre_off;
		} else if (aref - aoff >= SPRING_RETURN_CNT) {
			/*
			 * Closing on centre, but not home yet. This USED to vote
			 * hands-off and it is the reason a steady cornering
			 * angle flapped: steering back toward centre yourself
			 * produces exactly this, and 98 deg -> 64 deg was read as
			 * the spring winning while a hand was plainly doing it.
			 *
			 * A spring over breakaway does not stop halfway, so
			 * partial closure distinguishes nothing — only ARRIVING
			 * does, which is the branch above. Progress is being
			 * made either way, so abstain rather than time out into a
			 * hands-on verdict, and re-reference so the next window
			 * measures from here.
			 */
			t_spring = now;
			spring_ref = centre_off;
		} else if (now - t_spring >= SPRING_TEST_MS) {
			/*
			 * Failed to close in time. Only a verdict if the wheel had
			 * ROOM to close: the spring stops at CENTRE_DEADBAND, so
			 * from nearer than SPRING_RETURN_CNT + CENTRE_DEADBAND out
			 * the travel simply is not there and no free wheel can
			 * pass this test. A wheel that has finished returning sits
			 * exactly there, and voting ON was pinning the verdict on
			 * every 400 ms against a probe that can only clear it once
			 * per 8 s. No room means no measurement: abstain, leaving
			 * both the verdict and the vote count alone.
			 */
			if (aref >= SPRING_RETURN_CNT + CENTRE_DEADBAND) {
				hands_on = hod_vote(true, &votes, &pending);
			}
			t_spring = now;
			spring_ref = centre_off;
		}
	} else {
		t_spring = now;
		spring_ref = centre_off;
	}

	/*
	 * Held for a whole velocity window after the wheel stops, which is the
	 * settle time the seek wants anyway before it starts pushing again.
	 */
	moving = (vel_raw >= HOD_MOVED || vel_raw <= -HOD_MOVED);
	probe_torque = hod_probe(now, moving, &probing, probe_inhibit);

	/*
	 * Measure only while a probe is pushing — between probes there is no
	 * stimulus, so movement there says nothing about hands on the wheel.
	 *
	 * This runs BEFORE the torque is computed so an abort takes effect on this
	 * very tick, not the next one. The moment the excursion clears the
	 * threshold the answer is already "nobody is holding it", and pushing any
	 * further just spins a free wheel across its travel.
	 */
	if (probing) {
		int32_t w;

		if (!was_probing) {
			lo = hi = steering;     /* probe started: fresh window */
			probe_ref = steering;   /* ...and where to put it back */
			aborted = false;
			probe_peak_w = 0;
			t_probe0 = now;         /* for the time-to-threshold diag */
			hod_peak_t = 0;
			hod_t2thr = 0;          /* 0 stays 0 if it never gets there */
		}
		if (steering < lo) { lo = steering; }
		if (steering > hi) { hi = steering; }

		w = vel_to_deg_s(feel_vel);
		if (w < 0) { w = -w; }
		if (w > probe_peak_w) { probe_peak_w = w; }

		if (!aborted && (uint16_t)(hi - lo) >= hod_free_counts) {
			/* Moved far — but a free wheel and a hand following the nudge
			 * both do that. Speed is what separates them. */
			bool driven = probe_peak_w >
				      hod_free_speed() * hod_overspeed / 100;

			hod_p2p = (uint16_t)(hi - lo);
			hod_t2thr = (uint16_t)(now - t_probe0);
			hands_on = hod_vote(driven, &votes, &pending);
			aborted = true;
			/* Free wheel: the travel is ours to give back. Driven:
			 * a hand is moving it, so hands off it. */
			ret_wanted = !driven;
			t_abort = now;
			abort_from = probe_torque;
		}
	} else if (was_probing) {
		if (!aborted) {
			bool driven = probe_peak_w >
				      hod_free_speed() * hod_overspeed / 100;

			hod_p2p = (uint16_t)(hi - lo);
			/* Didn't travel far enough to be free, or moved faster
			 * than a free wheel could — either way a hand is on it. */
			hands_on = hod_vote(hod_p2p < hod_free_counts || driven,
					    &votes, &pending);
		}
		if (ret_wanted) {
			int32_t err = (int32_t)probe_ref - (int32_t)steering;

			ret_wanted = false;
			if (hod_return_dead > 0 &&
			    (err > hod_return_dead || err < -hod_return_dead)) {
				returning = true;
				t_return  = now;
				/* From the floor, like a probe. Starting at 0
				 * would spend the whole RETURN_MS seeking up
				 * to breakaway and never move the wheel. */
				ret_peak  = hod_creep;
				ret_dir   = (err > 0) ? 1 : -1;
			}
		}
	}
	was_probing = probing;

	/*
	 * Release the abort over hod_abort_ms rather than dropping to zero in one
	 * step. A step from full torque to nothing knocks the gear lash exactly
	 * like a square wave does; 40 ms reads as instant to a hand but is silent.
	 * Set 'a 0' for a genuinely abrupt cut.
	 */
	if (probing && aborted) {
		uint32_t el = now - t_abort;

		if (hod_abort_ms <= 0 || el >= (uint32_t)hod_abort_ms) {
			probe_torque = 0;
			/* Nothing left to measure — end the probe here instead
			 * of idling out the window, so the return starts while
			 * the nudge still reads as one gesture. */
			hod_running = false;
		} else {
			probe_torque = abort_from *
				       (int32_t)((uint32_t)hod_abort_ms - el) /
				       hod_abort_ms;
		}

		/*
		 * ARREST the wheel, don't merely stop pushing it.
		 *
		 * Releasing torque leaves a free wheel coasting on its own inertia
		 * (tau = J/b = 50 ms from the step-response fit), and at the ~107
		 * deg/s a free wheel reaches under a 4000-unit probe that coast is
		 * worth about as much travel again as the probe itself. Roughly:
		 *
		 *     total ~= threshold + peak_speed * (release_time + tau)
		 *
		 * so the excursion could not be brought down by lowering the
		 * threshold alone — the second term does not depend on it.
		 *
		 * The seek is now the primary answer to that term (it aborts
		 * from a crawl, not from 107 deg/s), so this is a backstop for
		 * the case the seek cannot cover: a hand that was driving the
		 * wheel and lets go mid-probe. It costs nothing when the wheel
		 * is already slow, which is now the normal case.
		 *
		 * This term is VELOCITY-PROPORTIONAL, which matters for three
		 * reasons: it opposes motion only, it fades to nothing exactly as
		 * the wheel stops (so it cannot overshoot or drive the wheel back
		 * the other way like a fixed counter-pulse would), and it never
		 * changes sign, so it does not knock the gear lash — the same
		 * property that made the probe a single-sign pulse rather than a
		 * buzz.
		 *
		 * Gain is kept near 2x the measured natural damping. car_visc at
		 * 3.11x natural is known to self-oscillate on this wheel; this is
		 * well under that, and it is transient (it ends with the probe)
		 * rather than a permanently closed loop.
		 */
		int32_t arrest = -(int32_t)feel_vel * HOD_ARREST_GAIN;

		if (arrest >  HOD_ARREST_MAX) { arrest =  HOD_ARREST_MAX; }
		if (arrest < -HOD_ARREST_MAX) { arrest = -HOD_ARREST_MAX; }
		probe_torque += arrest;
	}

	/*
	 * ── Put the wheel back where the probe found it ──────────────────
	 *
	 * A hands-off verdict is a licence to move the wheel: nobody is holding
	 * it, so the travel the nudge just spent is ours to give back. Without
	 * this the wheel keeps every probe's excursion — the dir alternation
	 * only cancels it on average, and only while probes alternate cleanly.
	 *
	 * It is hod_push() again, aimed at a position instead of an excursion,
	 * which matters more here than in the probe: creeping home in ~1-count
	 * steps means it stops AT the target rather than overshooting and
	 * hunting across it, and a hunt would reverse the torque — the one
	 * thing that knocks the lash.
	 *
	 * ret_dir is fixed at the start and never re-derived from the error, so
	 * an overshoot ends the return rather than triggering a push back.
	 */
	if (returning) {
		int32_t err = ((int32_t)probe_ref - (int32_t)steering) * ret_dir;
		int32_t step;

		if (err <= hod_return_dead || now - t_return >= RETURN_MS) {
			/* Unload over hod_abort_ms rather than dropping: a step
			 * to zero clacks the same way an edge does. */
			step = (hod_abort_ms > 0)
			     ? hod_creep * FEEL_MS / hod_abort_ms : hod_creep;
			ret_peak -= (step < 1) ? 1 : step;
			if (ret_peak <= 0) {
				ret_peak = 0;
				returning = false;
			}
			probe_torque = ret_peak * ret_dir;
		} else {
			probe_torque = hod_push(now - t_return, moving,
						&ret_peak) * ret_dir;
		}
	}

	if (now - t_feel >= FEEL_MS) {
		int32_t f = car_feel(feel_vel, probing || returning) + probe_torque;

		/*
		 * Speed-dependent car feel, summed onto the same single constant-
		 * force channel. Both are suppressed while a probe runs: the probe
		 * measures the wheel plus the driver's grip, so the simulated car
		 * must step out of the way, and rumble would add noise to the very
		 * excursion the verdict is computed from. It also keeps the probe
		 * stimulus identical to before these effects existed, so
		 * hod_free_counts does NOT need recalibrating.
		 *
		 * This no longer costs the driver anything noticeable: probes are
		 * now inhibited whenever the spring is strong enough to matter, so
		 * by construction any probe that DOES run has a sub-breakaway
		 * spring to suppress. Holding a cornering angle keeps full centring.
		 */
		if (!probing && !returning) {
			f += centre_torque + powertrain_road_rumble(car_rumble);
		}

		/* Clamp: the terms are independently tunable and can be turned
		 * up past what a single int16 torque can carry. */
		if (f >  32767) { f =  32767; }
		if (f < -32767) { f = -32767; }

		feel_force = (int16_t)f;
		g29_send_constant_force(feel_force);
		t_feel = now;
	}

	/*
	 * At a genuine standstill, force the verdict to OFF rather than
	 * trusting whatever vote logic ran above this tick.
	 *
	 * Neither mechanism that can ever flip hands_on to false is reachable
	 * at speed_kmh == 0: the spring test requires centre_can_move, but
	 * powertrain_self_centre() unconditionally returns 0 whenever
	 * state.speed_kmh == 0, so centre_torque can never exceed
	 * WHEEL_BREAKAWAY there; and the probe is disabled outright below
	 * PROBE_MIN_KMH. So once hands_on latches true while parked (e.g. the
	 * driver turns the wheel before pulling away), there is no verdict-
	 * producing path left that could ever clear it again -- it would stay
	 * ON indefinitely until the car starts moving, which is exactly what
	 * showed up in testing: hands=1 held for thousands of ticks at
	 * spd=0, p2p=0, with no probe having run since the initial vote.
	 *
	 * Hands-on-wheel monitoring is not a meaningful claim while parked
	 * anyway, so the simplest correct answer is to just say OFF outright
	 * rather than build a standstill-specific probe.
	 */
	if (standstill) {
		hands_on = false;
		/* Reset the machine, not just its output. Leaving the latch and
		 * the tally alone meant pulling away resumed from whatever was
		 * decided before stopping, and the next vote returned it
		 * immediately — so the verdict at 10 km/h was never actually
		 * measured. Pulling away now starts from OFF and costs
		 * hod_confirm fresh verdicts to say otherwise. */
		hod_state = false;
		pending = -1;
		votes = 0;
	}

	/*
	 * Edge triggered, so it reports the transition once rather than every pass.
	 * Printed even when the periodic log is muted ('q') — this is an event, not
	 * telemetry. -1 start means the first determination is announced too.
	 */
	if (last_reported != (int8_t)hands_on) {
		last_reported = (int8_t)hands_on;
		printf("[%lu] HANDS %s   p2p=%u thr=%ld  peak=%ld free=%ld deg/s"
		       "  peakT=%ld t2thr=%ums%s\r\n",
		       (unsigned long)now, hands_on ? "ON " : "OFF",
		       hod_p2p, (long)hod_free_counts,
		       (long)probe_peak_w, (long)hod_free_speed(),
		       (long)hod_peak_t, hod_t2thr,
		       (!hands_on && aborted) ? "  (cut short)" : "");
	}

	return hands_on;
}


/* ── Public entry points ────────────────────────────────────────────────── */
/*
 * The FFB channel is single-owner. The sweep and the identification each take
 * it outright while active; only when neither is running does the normal
 * feel + hands-on loop get to drive the torque.
 */
bool steer_feel_update(uint16_t steering)
{
	if (ffb_sweep(steering)) {
		return false;
	}
	if (sysid_update(steering)) {
		return false;
	}
	return HOD_TEST ? hod_update(steering) : false;
}

void steer_feel_get_telemetry(struct steer_feel_telemetry *out)
{
	out->force  = feel_force;
	out->vel    = feel_vel;
	out->centre = feel_centre;
	out->p2p    = hod_p2p;
}

void steer_feel_sysid_start(void)
{
	sysid_start();
}

void steer_feel_stop(void)
{
	g29_send_no_effect();
	sysid_abort();
}

bool steer_feel_console(char cmd, long v)
{
	switch (cmd) {
	case 't': hod_torque      = v; break;
	case 'v': feel_vel_dead   = v; break;
	case 'h': hod_free_counts = v; break;
	case 'p': hod_probe_ms    = v; break;
	case 'i': hod_period_ms   = v; break;
	case 'r': hod_ramp_ms     = v; break;
	case 'a': hod_abort_ms    = v; break;
	case 'k': hod_creep       = (v < 0) ? 0 : v; break;
	case 'L': hod_preload_ms  = (v < 0) ? 0 : v; break;
	case 'R': hod_return_dead = (v < 0) ? 0 : v; break;
	case 'c': car_coulomb     = v; break;
	case 'b': car_visc        = v; break;
	case 'e': car_w_eps    = (v <  10) ?  10 : v; break;
	case 'n': hod_confirm  = (v <   1) ?   1 : v; break;
	case 'y': hod_idle_ms     = v; break;
	case 'm': hod_move_cnt = (v <   1) ?   1 : v; break;
	case 'o': hod_overspeed = (v < 100) ? 100 : v; break;
	/*
	 * Clamped, unlike the knobs above: these three are multiplied by the
	 * steering offset (up to 32768) inside powertrain.c, so an unbounded
	 * value here would overflow int32 rather than merely feel wrong.
	 */
	case 'C': car_centre   = (v < 0) ? 0 : ((v > 32767) ? 32767 : v); break;
	case 'u': car_rumble   = (v < 0) ? 0 : ((v > 32767) ? 32767 : v); break;
	case 'x': car_stiction = (v < 0) ? 0 : ((v > 32767) ? 32767 : v); break;
	default:
		return false;	/* not ours — let the caller try */
	}
	return true;
}

void steer_feel_print_values(void)
{
	printf("c=%ld b=%ld C=%ld x=%ld u=%ld t=%ld k=%ld L=%ld R=%ld v=%ld h=%ld p=%ld i=%ld r=%ld\r\n",
	       (long)car_coulomb, (long)car_visc,
	       (long)car_centre, (long)car_stiction, (long)car_rumble,
	       (long)hod_torque, (long)hod_creep, (long)hod_preload_ms,
	       (long)hod_return_dead, (long)feel_vel_dead,
	       (long)hod_free_counts, (long)hod_probe_ms,
	       (long)hod_period_ms, (long)hod_ramp_ms);
}

void steer_feel_print_help(void)
{
	printf("  c=tyre scrub (torque units, wheel's own breakaway=3401)  e=sgn blend deg/s\r\n"
	       "  b=viscous x10 per deg/s (138 = 1.0x the measured natural)\r\n"
	       "  C=self-centring torque at full lock, scales with speed (0=off)\r\n"
	       "  x=stiction comp: lifts the spring over breakaway so it RETURNS (0=off)\r\n"
	       "  u=road rumble amplitude, scales with speed (0=off)\r\n"
	       "  t=probe torque CEILING (only a wheel that will not move sees it)\r\n"
	       "  k=creep floor: keep under breakaway 3401, THE nudge-size knob after h\r\n"
	       "  L=lash pre-load ms  v=vel deadband  h=hands thr\r\n"
	       "  R=return-to-start deadband: puts the wheel back after a hands-off probe (0=off)\r\n"
	       "  p=probe ms  i=interval ms  r=release ramp ms  a=abort release ms  n=probes to confirm\r\n"
	       "  y=idle ms before probing  m=steering-input threshold\r\n"
	       "  o=overspeed %% of free-wheel speed that means a hand is driving\r\n");
}
