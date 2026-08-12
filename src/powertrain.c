/*
 * Powertrain simulation -- see include/powertrain.h.
 *
 * Flow:  pedals -> power_factor(rpm)/brake -> gear_ratio -> wheel force
 *        -> drag -> speed_accum -> speed -> rpm (through the current ratio)
 *
 * speed_accum is in units of 0.01 km/h so launch from rest is smooth -- no
 * rounding jumps below 1 km/h.
 *
 * Constants and the shift/torque-curve model are carried over unchanged
 * from the reference Node-1 ECU sim (same PT_TICK_MS, so the tuning still
 * applies): 0-100 km/h in ~8-10 s at WOT in Drive.
 */
#include "powertrain.h"

#define PT_REVERSE_RATIO_X1000 7000
#define PT_SPEED_FACTOR        40	/* speed_kmh = rpm * FACTOR / ratio_x1000 */

#define PT_IDLE_RPM        800
#define PT_MAX_RPM         7000
#define PT_REVERSE_MAX_RPM 4000

#define PT_UPSHIFT_RPM_LIGHT 2200	/* 0% throttle shift point   */
#define PT_UPSHIFT_RPM_WOT   6800	/* 100% throttle shift point */
#define PT_DOWNSHIFT_RPM     1100
/*
 * Minimum time after ANY completed shift before the next automatic downshift
 * may arm, even if the RPM condition is already satisfied that same tick.
 *
 * Without this, hard braking from a high top-gear speed cascades through
 * every gear almost instantly: with gear_ratio[] spaced this closely in the
 * upper gears, the speed gap between one gear's downshift trigger and the
 * next's is only ~2-7 km/h, but a single PT_SHIFT_DURATION_MS clutch-slip
 * loses ~11-12 km/h under the ~33 km/h/s braking this sim is tuned for (see
 * PT_BRAKE_SCALE). So the car is ALREADY past the next gear's own threshold
 * before the current shift even finishes, and re-arms on the very next tick
 * -- 8th falls straight through to 1st in barely more than one shift
 * duration, which reads as the transmission being stuck in top gear and then
 * dumping, not as a normal deceleration through the gears.
 *
 * A real automatic paces sequential downshifts one at a time and lets each
 * one be felt; this cooldown reproduces that pacing without having to
 * re-space every gear ratio (which would fight the ~250 km/h top-speed
 * target the ladder above is tuned for). Set a bit longer than
 * PT_SHIFT_DURATION_MS so there is a genuine gap between gears, not just
 * back-to-back shifts with no daylight between them.
 */
#define PT_DOWNSHIFT_COOLDOWN_MS 500
#define PT_KICKDOWN_THROTTLE   85
#define PT_KICKDOWN_RPM      3500
#define PT_SHIFT_DURATION_MS  350

/* Consecutive PT_TICK_MS ticks the upshift condition must hold before it is
 * actually committed with start_shift(). A shift, once armed, runs its
 * clutch-slip to completion regardless of what throttle does afterward, so a
 * single-tick throttle sample can otherwise latch a shift the driver never
 * meant to hold through -- see the debounce comment in forward_tick(). At
 * PT_TICK_MS = 20 ms, 3 ticks is ~60 ms: enough to reject a one-sample blip,
 * short enough that a genuine sustained upshift is not felt as sluggish. */
#define PT_UPSHIFT_DEBOUNCE 3

/*
 * Retuned from 15. The Node-1 reference value (15) gave a genuine 0-100 in
 * ~5.7s, but with the retuned gear_ratio[] below it settled at only ~173
 * km/h top speed -- because top speed is a force-vs-drag EQUILIBRIUM, not a
 * redline number, and no choice of top-gear ratio can raise that equilibrium
 * on its own: a taller gear proportionally cuts force at every RPM as fast
 * as it raises the RPM-per-km/h, so equilibrium speed actually falls, not
 * rises, as the top gear gets taller (confirmed by direct simulation of the
 * force/drag balance across a range of ratios). Top speed and 0-100 time are
 * therefore coupled through this ONE scale, not independently tunable via
 * gearing -- raising top speed requires raising this.
 *
 * 22 was chosen (full 8-gear WOT simulation, including real shift dynamics)
 * to land close to the middle of the target ranges on both axes at once.
 * Measured with the ratios and PT_AERO_DRAG_DIV below: 0-100 in ~4.3 s, top
 * speed 197 km/h in 8th. Raised from 21 to pay for the rev limiter in
 * apply_forward_force(): the car had been buying acceleration by over-revving
 * gears 1-3 past redline, and taking that back cost ~0.3 s to 100 km/h.
 * Neither axis lands exactly on an arbitrary target
 * because the two pull in opposite directions under this linear force model;
 * this is the closest simultaneous balance available without either an
 * unrealistically violent launch or a materially undersized top speed.
 *
 * CEILING: apply_forward_force() evaluates throttle * pf * ratio * this
 * before dividing, which is 987e6 at the maximum (100 * 100 * 4700 * 21).
 * int32 runs out at ~45. Past that the multiply overflows and force goes
 * NEGATIVE at full throttle in 1st -- widen the intermediate to int64 first
 * if the scale ever needs to go that high.
 */
#define PT_ACCEL_SCALE      22
/*
 * Gear ratios and PT_AERO_DRAG_DIV are tuned together for a 199 km/h top
 * speed -- an equilibrium figure, measured over a 60 s WOT run, not the
 * kinematic redline speed of 8th (which is higher; the car never gets there).
 * They replace the original Node-1 values, which were tuned for a much taller
 * and internally inconsistent top end -- see the note below.
 *
 * The original ladder let wheel force fall off faster (with each upshift's
 * smaller ratio) than aero drag rose with speed, so force-vs-drag
 * equilibrium landed inside 5th/6th gear: the car could never reach 6800 RPM
 * (the WOT upshift trigger) in 5th or higher, so gears 6/7/8 were
 * unreachable except by lifting off throttle to collapse the upshift
 * threshold -- and even then the car couldn't hold speed in them, since
 * force there was already below drag. That is what produced "stuck at 174
 * km/h in 5th/6th" and upshifts that only ever happened on lift-off.
 *
 * Re-derived from speed_kmh = rpm * PT_SPEED_FACTOR / ratio_x1000. Gears
 * 1-7 keep a geometric progression (~0.81x per gear, typical for a road-car
 * box) down from the unchanged 1st gear, with ONE deliberate exception at
 * 7th -- see below.
 *
 * DRAG IS THE TOP-SPEED KNOB, not the gearing. Sweeping all three levers
 * against the 199 km/h target:
 *   - PT_AERO_DRAG_DIV : hits it at ~zero cost to 0-100 (4.10 s). Chosen.
 *   - PT_ACCEL_SCALE   : also works, but drags 0-100 out to 4.6 s.
 *   - shorter gears    : BACKFIRES -- measured top speed RISING to 221 km/h,
 *                        because clamp_rpm() limits the reported RPM but
 *                        apply_forward_force() keeps pulling on the clamped
 *                        value, so there is no rev limiter in the physics.
 *
 * FLOOR: do not take PT_AERO_DRAG_DIV below ~1950. At 1900 the car can no
 * longer reach the 7->8 upshift speed at all and 8th becomes permanently
 * unreachable -- the same "gears you can never get to" failure described
 * above, just relocated to the top of the ladder.
 */
#define PT_AERO_DRAG_DIV  2200		/* drag = speed^2 / DIV (accum units) */
#define PT_ROLL_DRAG_DIV   100		/* drag = speed / DIV                 */
#define PT_ENGINE_BRAKE_BASE 8		/* scaled by gear ratio AND rpm -- see
					 * apply_forward_force()             */
/*
 * Floor under engine braking, so a coast actually finishes.
 *
 * Once road speed drops far enough that rpm clamps at PT_IDLE_RPM, the
 * rpm-scaling term above goes to zero and engine braking would collapse to
 * 1/tick -- which left the last 5 km/h taking 10 s to bleed off. That is not
 * just slow to watch: powertrain_tick()'s R<->D interlock only engages at
 * speed_accum == 0, so the selector stayed locked out long after the car had
 * visually stopped. 3/tick clears that tail in ~3 s instead.
 */
#define PT_COAST_MIN 3

/* Brakes act on the wheels directly, independent of gear ratio -- unlike
 * engine braking. Tuned for ~100 km/h -> 0 in ~3 s at full pedal
 * (~33 km/h/s, firm braking). */
#define PT_BRAKE_SCALE       67

#define PT_RPM_RISE_FREE_TICK 30	/* reverse: throttle -> rpm rise/tick */
#define PT_RPM_FALL_TICK      25	/* rpm decay/tick with no throttle    */

/*
 * Free-rev in N/P: the gearbox is disconnected from the wheels, so throttle
 * only has to spin the engine itself against its own internal friction --
 * no vehicle mass, no aero drag, nothing like the load forward_tick() or
 * reverse_tick() fight. A real engine free-revs MUCH faster than it pulls a
 * loaded car, so this gets its own rise rate rather than reusing
 * PT_RPM_RISE_FREE_TICK (which is tuned for reverse, where the drivetrain is
 * still coupled to the wheels through PT_REVERSE_RATIO_X1000).
 *
 * ~200 rpm/tick at PT_TICK_MS puts idle-to-redline at full throttle around
 * 0.6 s -- snappy and immediate, the way blipping the throttle in neutral
 * actually feels, and bounded by the same PT_MAX_RPM rev limiter every other
 * mode already respects.
 */
#define PT_RPM_FREEREV_RISE_TICK 200
#define PT_RPM_FREEREV_FALL_TICK  40	/* faster than PT_RPM_FALL_TICK: no
					 * load also means less to slow it */

/* --- Speed-dependent steering feel ---
 *
 * Both effects ramp linearly from nothing at a standstill to full at their
 * saturation speed, then hold. Real self-aligning torque keeps climbing a
 * while longer and then falls off as the tyre saturates, but the wheel runs
 * out of usable torque range long before that matters. */
#define PT_CENTRE_FULL_KMH   80
#define PT_RUMBLE_FULL_KMH  120
#define PT_STEER_FULL_LOCK 32768	/* widened counts, centre -> full lock */

/* Rumble low-pass, as an IIR shift: alpha = 1/(2^N).
 *
 * This is set by the control rate, not by taste. The torque loop runs at
 * 100 Hz (FEEL_MS in main.c), so Nyquist is 50 Hz and real road vibration
 * (mostly 20-200 Hz) is largely NOT renderable here -- what we can produce
 * is a low rumble. Corner frequency is roughly fs*alpha/(2*pi):
 *     N=3 -> alpha 0.125 ->  ~2 Hz : feels like the wheel slowly wandering
 *     N=1 -> alpha 0.5   ->  ~8 Hz : reads as a rumble, still slew-limited
 * N=1 it is. Going to 0 (no filter at all) is the most vibration per unit
 * amplitude but puts the sharpest edges into the gear lash. */
#define PT_RUMBLE_SHIFT       1

/*
 * 7th breaks the ~0.81x geometric progression on purpose (1600, not 1370).
 *
 * The 7->8 upshift fires at PT_UPSHIFT_RPM_WOT in 7th, which at a 1370 ratio
 * needs 6800 * 40 / 1370 = 198 km/h -- essentially the car's whole top speed.
 * 8th was therefore reachable only after ~24 s of flat-out driving, with zero
 * margin: 7th peaked at exactly the threshold it had to cross. Shortening 7th
 * moves that upshift to 6800 * 40 / 1600 = 170 km/h, so 8th arrives at ~13 s
 * with real daylight above it.
 *
 * Free on both headline figures -- top speed stays 199 km/h and 0-100 stays
 * 4.10 s, because neither is set by the intermediate ratios (see the
 * force-vs-drag note on PT_ACCEL_SCALE). The cost is a wider 7->8 step than
 * the rest of the ladder, which is what a real box does at the top anyway:
 * the overdrive gear is the odd one out.
 */
static const uint16_t gear_ratio[PT_NUM_GEARS] = {
	4700, 3830, 3120, 2540, 2070, 1690, 1600, 1120
};

enum shift_phase { SHIFT_IDLE = 0, SHIFT_UP, SHIFT_DOWN };

static struct powertrain_state state;
static int32_t speed_accum;	/* 0.01 km/h, magnitude only */
static struct {
	enum shift_phase phase;
	int32_t timer_ms;
	uint8_t target_gear;
} shift;
/* Consecutive ticks the upshift condition has held — see PT_UPSHIFT_DEBOUNCE
 * in forward_tick(). Reset in powertrain_init() and on every direction change
 * so stale progress can never leak across a reverse/forward boundary. */
static uint8_t up_confirm;
/*
 * Cooldown after any completed shift before the NEXT automatic downshift may
 * arm, even if the RPM condition is already met that same tick — see
 * PT_DOWNSHIFT_COOLDOWN_MS below for why this exists.
 */
static int32_t down_cooldown_ms;

/* Rumble generator state. Module-scope rather than function-static so
 * powertrain_init() resets it -- which is what makes the host test
 * repeatable. */
static uint32_t rumble_rng;
static int32_t  rumble_filt;	/* x16, so the IIR keeps sub-unit resolution */

void powertrain_init(void)
{
	state.engine_rpm = PT_IDLE_RPM;
	state.speed_kmh = 0;
	state.gear = 1;
	state.reverse = false;
	speed_accum = 0;
	shift.phase = SHIFT_IDLE;
	shift.timer_ms = 0;
	shift.target_gear = 0;
	up_confirm = 0;
	down_cooldown_ms = 0;
	rumble_rng = 1;		/* any non-zero seed; xorshift dies on 0 */
	rumble_filt = 0;
}

/*
 * Engine RPM the current road speed and gear DEMAND, before the limiter.
 *
 * Derived from speed_accum rather than from whole km/h: speed_accum already
 * carries 0.01 km/h resolution, and the old `speed_accum / 100 * ratio / 40`
 * threw all of it away -- the tacho stepped in 117 rpm jumps in 1st (95 in
 * 2nd, 78 in 3rd), and that quantised value drives the shift thresholds and
 * goes out on CAN 0x0A2.
 *
 * Returns the UNCLAMPED value on purpose: the rev limiter in
 * apply_forward_force() needs to see the over-rev that clamp_rpm() hides.
 * Worst case product is ~9.4e7, well inside int32.
 */
static int32_t kinematic_rpm(int32_t ratio)
{
	return speed_accum * ratio / (PT_SPEED_FACTOR * 100);
}

static int32_t clamp_rpm(int32_t rpm)
{
	if (rpm < PT_IDLE_RPM) {
		return PT_IDLE_RPM;
	}
	if (rpm > PT_MAX_RPM) {
		return PT_MAX_RPM;
	}
	return rpm;
}

/* Throttle-dependent upshift threshold: light throttle shifts early
 * (economy, ~2200 RPM), full throttle holds the gear to redline (~6800). */
static int32_t upshift_threshold(uint8_t throttle)
{
	return PT_UPSHIFT_RPM_LIGHT +
	       (PT_UPSHIFT_RPM_WOT - PT_UPSHIFT_RPM_LIGHT) * (int32_t)throttle / 100;
}

/* Engine torque curve, % of peak: weak at idle, peaks mid-range, tapers
 * near redline. Directly modulates wheel force. */
static int32_t power_factor(int32_t rpm)
{
	if (rpm <= 800) {
		return 40;
	}
	if (rpm <= 2500) {
		return 40 + (rpm - 800) * 55 / 1700;
	}
	if (rpm <= 4000) {
		return 95 + (rpm - 2500) * 5 / 1500;
	}
	if (rpm <= PT_MAX_RPM) {
		return 100 - (rpm - 4000) * 30 / 3000;
	}
	return 70;
}

/*
 * Aero + rolling drag, in speed_accum units per tick.
 *
 * Computed x16 and ROUNDED, not truncated. At these divisors the honest value
 * is a fraction of one accum unit through the whole low-speed range -- plain
 * integer division made drag literally ZERO below ~47 km/h and left it 30-40%
 * light across 50-90 km/h (0.71 -> 0 at 30 km/h, 4.58 -> 3 at 90). Rolling
 * drag alone (sp / PT_ROLL_DRAG_DIV) vanished entirely below 100 km/h.
 *
 * x16 is enough: it puts the residual error at half a unit everywhere instead
 * of a whole one, and keeps the intermediate tiny (sp*sp*16 is ~1e6 at the
 * top of the speed range).
 */
static int32_t road_drag(int32_t sp)
{
	int32_t d16 = (sp * sp * 16) / PT_AERO_DRAG_DIV +
		      (sp * 16) / PT_ROLL_DRAG_DIV;

	return (d16 + 8) / 16;
}

/* Net force on the drivetrain this tick, in speed_accum units: throttle
 * force (torque curve x gear ratio, halved-in-thirds during clutch slip)
 * minus brake (wheel-applied, gear-independent) or engine braking, minus
 * aero+rolling drag. Updates speed_accum in place.
 *
 * over_rev is the rev limiter: see the call sites in forward_tick(). */
static void apply_forward_force(uint8_t throttle, uint8_t brake,
				 int32_t ratio, int32_t rpm, bool slipping,
				 bool over_rev)
{
	int32_t sp = speed_accum / 100;
	int32_t drag = road_drag(sp);

	if (brake > 0) {
		int32_t decel = (int32_t)brake * PT_BRAKE_SCALE / 100;

		speed_accum -= decel + drag;
	} else if (throttle > 0) {
		int32_t pf = power_factor(rpm);
		int32_t force = (int32_t)throttle * pf * ratio * PT_ACCEL_SCALE /
				 (100 * 100 * 1000);

		if (slipping) {
			force /= 3;
		}
		/*
		 * Fuel cut. clamp_rpm() only ever capped the REPORTED rpm --
		 * force kept being applied on the clamped value, so the car
		 * accelerated past each gear's kinematic redline with the tacho
		 * pinned at PT_MAX_RPM. Measured on a WOT run before this: 1st,
		 * 2nd and 3rd over-revved on every upshift, worst case 402 rpm
		 * past the limiter. There was no limiter in the physics at all,
		 * only in the display.
		 */
		if (over_rev) {
			force = 0;
		}
		speed_accum += force - drag;
	} else {
		int32_t engine_brake = PT_ENGINE_BRAKE_BASE * ratio / 1000;

		/*
		 * Engine braking is pumping loss, so it scales with ENGINE
		 * SPEED and all but vanishes at idle -- it is not a function of
		 * gear alone. Without this term the model applied a flat
		 * gear-only deceleration right down to a standstill: lifting
		 * off at 40 km/h in 1st stopped the car in 2.1 s at a constant
		 * 17 km/h/s, half the full brake pedal, just for coming off the
		 * accelerator.
		 */
		engine_brake = engine_brake * (rpm - PT_IDLE_RPM) /
			       (PT_MAX_RPM - PT_IDLE_RPM);
		if (engine_brake < PT_COAST_MIN) {
			engine_brake = PT_COAST_MIN;
		}
		speed_accum -= engine_brake + drag;
	}
	if (speed_accum < 0) {
		speed_accum = 0;
	}
}

static void start_shift(uint8_t from_gear, uint8_t to_gear)
{
	shift.phase = (to_gear > from_gear) ? SHIFT_UP : SHIFT_DOWN;
	shift.target_gear = to_gear;
	shift.timer_ms = PT_SHIFT_DURATION_MS;
}

static void forward_tick(uint8_t throttle, uint8_t brake)
{
	uint8_t gear = state.gear;
	int32_t rpm;
	int32_t ratio;

	if (gear < 1 || gear > PT_NUM_GEARS) {
		gear = 1;
	}
	ratio = gear_ratio[gear - 1];

	/* Wall-clock, not per-decision: the cooldown paces shifts in TIME, so it
	 * has to tick every tick -- including while a shift is slipping and while
	 * an upshift is arming. It used to live inside the else-branch of the
	 * upshift test below, where it froze in exactly those two cases. */
	if (down_cooldown_ms > 0) {
		down_cooldown_ms -= PT_TICK_MS;
	}

	/*
	 * Stopped mid-cascade: abandon the shift in flight instead of letting it
	 * land and step the gear down one more notch AFTER the car is already
	 * stationary. Without this the box visibly stair-steps (7, 7, 6, then 1)
	 * over the 60 ms following a stop, and that walk goes out on CAN 0x0A2.
	 *
	 * Threshold is < 100, not == 0, to match how the rest of this function
	 * decides "stopped": it tests sp == 0 where sp = speed_accum / 100, so
	 * anything under 1 km/h already reads as stopped. Guarding on
	 * speed_accum == 0 leaves a 1..99 window where the two disagree and the
	 * shift survives anyway.
	 */
	if (speed_accum < 100 && shift.phase != SHIFT_IDLE) {
		shift.phase = SHIFT_IDLE;
	}

	if (shift.phase != SHIFT_IDLE) {
		/* Clutch slip: RPM eases from the old gear's value to the new
		 * gear's value over SHIFT_DURATION_MS; torque is reduced. */
		int32_t old_rpm = clamp_rpm(kinematic_rpm(ratio));
		int32_t new_ratio = (int32_t)gear_ratio[shift.target_gear - 1];
		int32_t new_rpm = clamp_rpm(kinematic_rpm(new_ratio));
		int32_t progress;

		shift.timer_ms -= PT_TICK_MS;
		progress = (PT_SHIFT_DURATION_MS - shift.timer_ms) * 100 /
			   PT_SHIFT_DURATION_MS;
		rpm = old_rpm + (new_rpm - old_rpm) * progress / 100;

		apply_forward_force(throttle, brake, ratio, rpm, true,
				    kinematic_rpm(ratio) > PT_MAX_RPM);

		if (shift.timer_ms <= 0) {
			gear = shift.target_gear;
			shift.phase = SHIFT_IDLE;
			/* Cooldown applies after ANY shift, not just downshifts:
			 * an upshift completing right as the car keeps
			 * decelerating (e.g. a brief lift mid-brake) should not
			 * let a downshift immediately re-arm on the very next
			 * tick either -- same pacing reasoning either direction. */
			down_cooldown_ms = PT_DOWNSHIFT_COOLDOWN_MS;
		}
	} else {
		int32_t sp = speed_accum / 100;
		int32_t up_thr;

		rpm = clamp_rpm(kinematic_rpm(ratio));
		apply_forward_force(throttle, brake, ratio, rpm, false,
				    kinematic_rpm(ratio) > PT_MAX_RPM);

		sp = speed_accum / 100;
		rpm = clamp_rpm(kinematic_rpm(ratio));

		up_thr = upshift_threshold(throttle);
		/* throttle > 0 guard: with the foot off the gas (coasting or
		 * braking) the 0%-throttle threshold is only 2200 RPM, well
		 * below a typical cruising/braking RPM -- without this guard
		 * the car spuriously UPSHIFTS while braking instead of
		 * downshifting through the gears as it slows. */
		if (throttle > 0 && rpm >= up_thr && gear < PT_NUM_GEARS) {
			/*
			 * Debounced, not armed on one tick. up_thr swings by
			 * PT_UPSHIFT_RPM_WOT - PT_UPSHIFT_RPM_LIGHT (4600 RPM)
			 * between 0% and 100% throttle, so a single noisy or
			 * transient throttle sample right at a high-RPM instant
			 * -- e.g. one tick still reading nonzero the moment the
			 * driver lifts off -- could otherwise arm an upshift that
			 * then runs its clutch-slip to completion under
			 * SHIFT_DURATION_MS regardless of what throttle does a
			 * tick later. That is what put the car in 6th at 3500 RPM
			 * coasting at 0% throttle: the shift had already latched
			 * before the log showed thr=0.
			 *
			 * Requiring the condition to hold for PT_UPSHIFT_DEBOUNCE
			 * consecutive ticks means one stray sample can never
			 * single-handedly commit to a shift; the counter resets
			 * the instant the condition fails, so it cannot accumulate
			 * across unrelated moments either.
			 */
			if (++up_confirm >= PT_UPSHIFT_DEBOUNCE) {
				up_confirm = 0;
				start_shift(gear, gear + 1);
			}
		} else {
			up_confirm = 0;

			/*
			 * Gated on the cooldown — see PT_DOWNSHIFT_COOLDOWN_MS
			 * for why. Without this, hard braking from a high gear
			 * cascades through every gear almost instantly: each
			 * gear's own downshift RPM threshold is already behind
			 * the car by the time the PREVIOUS shift's clutch-slip
			 * finishes, so it re-arms on the very next tick. This
			 * paces automatic downshifts one at a time instead.
			 *
			 * Also gated on sp > 0 — see the standstill snap below
			 * for the sp == 0 case, which this deliberately excludes.
			 */
			if (down_cooldown_ms <= 0 && sp > 0) {
				if (rpm < PT_DOWNSHIFT_RPM && gear > 1) {
					start_shift(gear, gear - 1);
				} else if (throttle >= PT_KICKDOWN_THROTTLE &&
					   rpm < PT_KICKDOWN_RPM && gear > 1) {
					start_shift(gear, gear - 1);
				}
			}

			/*
			 * Standstill snap: the instant the car is genuinely
			 * stopped, go straight to 1st rather than trusting the
			 * per-tick RPM cascade above to have gotten there in
			 * time. That cascade is paced by
			 * PT_DOWNSHIFT_COOLDOWN_MS (one gear per cooldown
			 * window) specifically so hard braking doesn't dump
			 * through every gear in under a second -- but that same
			 * pacing means it can genuinely run out of speed before
			 * it runs out of gears on an ordinary (not maximally
			 * hard) stop, leaving the car sitting at a red light in,
			 * say, 6th. A real automatic never does that: whatever
			 * gear the ladder happened to reach during deceleration,
			 * the box is always in 1st the moment the car is
			 * actually stopped, ready to pull away.
			 *
			 * No shift.phase guard needed: the top of this function
			 * already cancels any shift in flight once the car is
			 * stopped, so nothing can be slipping by the time we
			 * get here.
			 *
			 * Clearing the cooldown matters too -- a stopped car is
			 * ready to pull away immediately, and leaving up to
			 * PT_DOWNSHIFT_COOLDOWN_MS on the clock would gate the
			 * first shift of the next launch on a timer left over
			 * from the previous stop.
			 */
			if (sp == 0 && gear != 1) {
				gear = 1;
				down_cooldown_ms = 0;
			}
		}
	}

	state.engine_rpm = (uint16_t)rpm;
	state.speed_kmh = (uint16_t)(speed_accum / 100);
	state.gear = gear;
}

/* Reverse: low-speed manoeuvring only, so RPM couples straight to speed
 * through the fixed reverse ratio rather than through the force model --
 * matches the reference sim. Braking decays speed via drag+brake while RPM
 * falls toward idle (foot off the "gas"). No gearbox, no shifts. */
static void reverse_tick(uint8_t throttle, uint8_t brake)
{
	int32_t rpm = state.engine_rpm;

	if (brake > 0) {
		int32_t sp = speed_accum / 100;
		int32_t drag = road_drag(sp);
		int32_t decel = (int32_t)brake * PT_BRAKE_SCALE / 100;

		speed_accum -= decel + drag;
		if (speed_accum < 0) {
			speed_accum = 0;
		}
		rpm -= PT_RPM_FALL_TICK;
		if (rpm < PT_IDLE_RPM) {
			rpm = PT_IDLE_RPM;
		}
	} else if (throttle > 0) {
		int32_t rise = (int32_t)throttle * PT_RPM_RISE_FREE_TICK / 100;
		int32_t target;

		rpm += rise;
		if (rpm > PT_REVERSE_MAX_RPM) {
			rpm = PT_REVERSE_MAX_RPM;
		}
		target = rpm * PT_SPEED_FACTOR * 100 / PT_REVERSE_RATIO_X1000;

		/*
		 * Throttle may only ever pull HARDER, never slow the car down.
		 * This used to assign `target` outright, but rpm and road speed
		 * genuinely disagree after a coast -- rpm decays on its own
		 * PT_RPM_FALL_TICK timer while speed decays on drag -- so
		 * re-deriving speed from the decayed rpm made the car JUMP
		 * BACKWARDS the instant the driver got back on the throttle
		 * (measured: 22 km/h -> 15 km/h in a single tick).
		 */
		if (target > speed_accum) {
			speed_accum = target;
		}
	} else {
		/*
		 * Coasting: RPM eases to idle like a real engine, but road speed
		 * must be free to decay to an ACTUAL zero independent of that —
		 * idle RPM floors at PT_IDLE_RPM (800) and never goes lower, so
		 * re-deriving speed_accum from rpm (as the throttle branch above
		 * does) pins it at idle-rpm's equivalent speed (~4 km/h) forever.
		 * That parked non-zero speed_accum permanently blocked the R->D
		 * interlock in powertrain_tick(), which only switches direction
		 * at speed_accum == 0 — the car could brake out of it (brake
		 * branch unconditionally zeroes speed_accum) but never coast out
		 * of it. Drag it down the same way forward_tick() coasts down.
		 */
		int32_t sp = speed_accum / 100;
		int32_t drag = road_drag(sp);

		if (drag < 1) {
			drag = 1;	/* guarantee progress at low speed */
		}
		speed_accum -= drag;
		if (speed_accum < 0) {
			speed_accum = 0;
		}
		rpm -= PT_RPM_FALL_TICK;
		if (rpm < PT_IDLE_RPM) {
			rpm = PT_IDLE_RPM;
		}
	}

	state.engine_rpm = (uint16_t)rpm;
	state.speed_kmh = (uint16_t)(speed_accum / 100);
	state.gear = 0;
}

/*
 * Neutral / Park: gearbox disconnected from the wheels, so throttle only
 * spins the engine against its own internal friction -- RPM follows the
 * pedal directly and climbs to PT_MAX_RPM (the same rev limiter every other
 * mode respects) with no vehicle mass or aero drag to fight, which is why
 * this free-revs far faster than reverse_tick() or forward_tick() ever do.
 *
 * Road speed is NOT driven by RPM here at all (unlike reverse_tick(), where
 * the drivetrain stays coupled through the fixed reverse ratio) -- a
 * disconnected gearbox cannot push the car no matter how hard the engine
 * revs. Speed can only ever fall: engine braking doesn't apply either (nothing
 * connects the engine to the wheels to brake against), so with the pedal off
 * the car just coasts down on drag alone, and the brake pedal still works
 * because brakes act on the wheels directly, independent of the gearbox.
 */
static void neutral_tick(uint8_t throttle, uint8_t brake)
{
	int32_t rpm = state.engine_rpm;
	int32_t sp = speed_accum / 100;
	int32_t drag = (sp * sp) / PT_AERO_DRAG_DIV + sp / PT_ROLL_DRAG_DIV;

	if (brake > 0) {
		int32_t decel = (int32_t)brake * PT_BRAKE_SCALE / 100;

		speed_accum -= decel + drag;
	} else {
		if (drag < 1) {
			drag = 1;	/* guarantee progress at low speed, same
					 * reasoning as the coast branches above */
		}
		speed_accum -= drag;
	}
	if (speed_accum < 0) {
		speed_accum = 0;
	}

	if (throttle > 0) {
		int32_t rise = (int32_t)throttle * PT_RPM_FREEREV_RISE_TICK / 100;

		rpm += rise;
		if (rpm > PT_MAX_RPM) {
			rpm = PT_MAX_RPM;
		}
	} else {
		rpm -= PT_RPM_FREEREV_FALL_TICK;
		if (rpm < PT_IDLE_RPM) {
			rpm = PT_IDLE_RPM;
		}
	}

	state.engine_rpm = (uint16_t)rpm;
	state.speed_kmh = (uint16_t)(speed_accum / 100);
	/* gear left untouched: N/P have no gear of their own to report, and
	 * main.c's console printf already takes the mode letter from the
	 * shifter selection directly rather than from state.gear/state.reverse
	 * when the selector reads N or P -- see the gear= fix there. */
}

void powertrain_tick(uint8_t throttle_pct, uint8_t brake_pct,
		      bool reverse_request, bool neutral)
{
	if (throttle_pct > 100) {
		throttle_pct = 100;
	}
	if (brake_pct > 100) {
		brake_pct = 100;
	}

	if (neutral) {
		/*
		 * Neutral pre-empts the D/R interlock below entirely: a
		 * neutral gearbox has no direction to hold, so there is
		 * nothing for reverse_request to interlock against here. The
		 * interlock's own state (state.reverse) is left untouched so
		 * whichever direction was engaged before N/P is what D/R picks
		 * back up the instant the lever leaves N/P -- exactly like a
		 * real column selector remembers nothing on its own, the
		 * detent position is what remembers it.
		 */
		neutral_tick(throttle_pct, brake_pct);
		return;
	}

	/* Direction change only takes effect at a standstill -- the same
	 * interlock a real PRND selector has against shifting into R while
	 * rolling forward. */
	if (reverse_request != state.reverse && speed_accum == 0) {
		state.reverse = reverse_request;
		state.gear = state.reverse ? 0 : 1;
		shift.phase = SHIFT_IDLE;
		up_confirm = 0;
		down_cooldown_ms = 0;
	}

	if (state.reverse) {
		reverse_tick(throttle_pct, brake_pct);
	} else {
		forward_tick(throttle_pct, brake_pct);
	}
}

void powertrain_get_state(struct powertrain_state *out)
{
	*out = state;
}

int32_t powertrain_self_centre(int32_t offset, int32_t strength)
{
	int32_t gain;

	if (strength <= 0 || state.speed_kmh == 0) {
		return 0;
	}
	if (offset > PT_STEER_FULL_LOCK) {
		offset = PT_STEER_FULL_LOCK;
	}
	if (offset < -PT_STEER_FULL_LOCK) {
		offset = -PT_STEER_FULL_LOCK;
	}

	gain = (int32_t)state.speed_kmh * 1024 / PT_CENTRE_FULL_KMH;
	if (gain > 1024) {
		gain = 1024;
	}

	/* Divide by full lock BEFORE applying the speed gain: strength*offset
	 * is already ~1e9 at the caller's maximum strength, so scaling first
	 * would overflow int32. */
	return -(strength * offset / PT_STEER_FULL_LOCK) * gain / 1024;
}

int32_t powertrain_road_rumble(int32_t strength)
{
	int32_t amp, target;

	if (strength <= 0 || state.speed_kmh == 0) {
		rumble_filt = 0;
		return 0;
	}

	/* xorshift32. Broadband noise, because a road surface IS broadband --
	 * a fixed tone reads as a motor whine, not tarmac. */
	rumble_rng ^= rumble_rng << 13;
	rumble_rng ^= rumble_rng >> 17;
	rumble_rng ^= rumble_rng << 5;

	amp = strength * (int32_t)state.speed_kmh / PT_RUMBLE_FULL_KMH;
	if (amp > strength) {
		amp = strength;
	}

	target = ((int32_t)(rumble_rng & 0xFFFFU) - 32768) * amp / 32768;

	/* Low-pass. Two jobs: it turns white noise into a rumble rather than a
	 * hiss, and it caps the slew rate -- this wheel's gear lash knocks on
	 * fast torque edges, which is the whole reason the hands-on probe is a
	 * ramped pulse instead of a buzz (see main.c). */
	rumble_filt += ((target << 4) - rumble_filt) >> PT_RUMBLE_SHIFT;
	return rumble_filt >> 4;
}
