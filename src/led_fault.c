/*
 * LED electrical fault detection: open/short monitoring for the four
 * indicator/beam LEDs, plus a startup self-test.
 *
 * Each LED has its own voltage-sense ADC channel (see led_cfg[] below) --
 * separate from the GPIO pins that actually drive the LEDs (owned by
 * telemetry.c). A healthy LED pulls its sense node into [ok_min_mv,
 * ok_max_mv] when driven on; an open circuit reads high (no load pulling
 * it down), a short reads low.
 *
 * Two independent consumers of the same classify/decide logic:
 *   - Normal monitoring (led_fault_update()): round-robin, one LED sampled
 *     per call, debounced over DEBOUNCE_SAMPLES on-samples.
 *   - Startup self-test (led_fault_selftest_start() / selftest_process()):
 *     drives each LED on in turn, in isolation, and classifies it before
 *     handing control back to normal monitoring.
 */
#include "led_fault.h"



/* ============================================================================
 * Configuration
 * ============================================================================ */

/*
 * led_fault_update() samples one LED per call and is called once per 10 ms
 * control tick, so each individual LED is re-sampled roughly every
 * LED_COUNT * 10 ms = 40 ms. DEBOUNCE_SAMPLES on-samples per decision is
 * therefore roughly DEBOUNCE_SAMPLES * 40 ms of real time.
 */
#define DEBOUNCE_SAMPLES      5U
#define SELFTEST_SAMPLES      5U

/* Ticks (at the 10 ms control period) to wait after physically turning an
 * LED on before the first self-test ADC sample, so the sense node settles. */
#define SELFTEST_SETTLE_TICKS 2U

/* ============================================================================
 * Hardware configuration
 * ============================================================================ */

typedef struct {
	uint32_t adc_channel;
	uint16_t ok_min_mv;
	uint16_t ok_max_mv;
	uint16_t margin_mv;
} led_config_t;

static const led_config_t led_cfg[LED_COUNT] = {
	[0] = { ADC_CHANNEL_0, 2000U, 2300U, 200U }, /* Left indicator */
	[1] = { ADC_CHANNEL_2, 2000U, 2300U, 200U }, /* Right indicator */
	[2] = { ADC_CHANNEL_3, 2700U, 2800U, 200U }, /* Low beam */
	[3] = { ADC_CHANNEL_9, 2700U, 2800U, 200U }, /* High beam */
};

/* ============================================================================
 * Normal-monitoring state
 * ============================================================================ */

typedef struct {
	led_fault_state_t state;
	uint16_t voltage;

	uint8_t good_count;
	uint8_t low_count;
	uint8_t high_count;
	uint8_t sample_count;

	bool prev_on;
} led_channel_t;

static led_channel_t leds[LED_COUNT];
static uint8_t next_led; /* Round-robin sampling cursor. */

/* ============================================================================
 * Self-test state
 * ============================================================================ */

typedef enum {
	SELFTEST_IDLE = 0,
	SELFTEST_TURN_ON,   /* Turn the current LED on. */
	SELFTEST_WAIT,      /* Wait for the sense node to settle. */
	SELFTEST_SAMPLE,    /* Take ADC samples. */
	SELFTEST_TURN_OFF,  /* Turn the current LED off, advance. */
	SELFTEST_COMPLETE,  /* Every LED tested. */
} selftest_state_t;

static selftest_state_t selftest_state;
static bool selftest_running;

static uint8_t selftest_led;
static uint8_t selftest_wait_ticks;
static uint8_t selftest_samples;
static uint8_t selftest_good, selftest_low, selftest_high;

static led_selftest_state_t selftest_result[LED_COUNT];

/* Outputs requested by the self-test -- NOT GPIO registers. control_task()
 * reads these and passes them to lights_gpio_set(). */
static bool selftest_out[LED_COUNT];

/* ============================================================================
 * Shared classification logic
 * ============================================================================ */

/*
 * Splits the sense range into three bands around [ok_min_mv, ok_max_mv]:
 * below trip_low = short, above trip_high = open, in between = good. The
 * two "gap" midpoints are the actual decision thresholds -- comparing
 * against ok_min_mv/ok_max_mv directly would leave no hysteresis margin.
 *
 * trip_low = ok_min_mv - margin_mv can underflow a uint16_t if a future
 * led_cfg[] entry ever sets margin_mv >= ok_min_mv; every current entry has
 * a comfortable margin, but the clamp below makes that a documented
 * invariant instead of a silent assumption.
 */
static void classify_common(const led_config_t *cfg, uint16_t mv,
			    uint8_t *good, uint8_t *low, uint8_t *high)
{
	uint16_t trip_low = (cfg->margin_mv < cfg->ok_min_mv)
				     ? (uint16_t)(cfg->ok_min_mv - cfg->margin_mv)
				     : 0U;
	uint16_t trip_high = (uint16_t)(cfg->ok_max_mv + cfg->margin_mv);

	uint16_t gap_low_mid = (uint16_t)((trip_low + cfg->ok_min_mv) / 2U);
	uint16_t gap_high_mid = (uint16_t)((cfg->ok_max_mv + trip_high) / 2U);

	if (mv < gap_low_mid) {
		(*low)++;
	} else if (mv > gap_high_mid) {
		(*high)++;
	} else {
		(*good)++;
	}
}

/* GOOD wins ties; between OPEN and SHORT, OPEN wins ties; SHORT only wins
 * once it strictly exceeds both of the others. */
static led_fault_state_t decide_common(uint8_t good, uint8_t low, uint8_t high)
{
	if (good >= low && good >= high) {
		return LED_OK;
	}
	if (high >= good && high >= low) {
		return LED_OPEN;
	}
	return LED_SHORT;
}

/* ============================================================================
 * Normal-monitoring helpers
 * ============================================================================ */

static void reset_window(uint8_t led)
{
	leds[led].good_count = 0U;
	leds[led].low_count = 0U;
	leds[led].high_count = 0U;
	leds[led].sample_count = 0U;
}

static void classify(uint8_t led, uint16_t mv)
{
	classify_common(&led_cfg[led], mv, &leds[led].good_count,
			&leds[led].low_count, &leds[led].high_count);
}

static led_fault_state_t decide(uint8_t led)
{
	return decide_common(leds[led].good_count, leds[led].low_count,
			     leds[led].high_count);
}

/* ============================================================================
 * Self-test helpers
 * ============================================================================ */

static void reset_selftest_window(void)
{
	selftest_samples = 0U;
	selftest_good = 0U;
	selftest_low = 0U;
	selftest_high = 0U;
}

static void selftest_outputs_off(void)
{
	for (uint8_t i = 0U; i < LED_COUNT; i++) {
		selftest_out[i] = false;
	}
}

/* Uses the same voltage boundaries as normal monitoring, deliberately. */
static void classify_selftest(uint8_t led, uint16_t mv)
{
	classify_common(&led_cfg[led], mv, &selftest_good, &selftest_low,
			&selftest_high);
}

static led_fault_state_t decide_selftest(void)
{
	return decide_common(selftest_good, selftest_low, selftest_high);
}

/* ============================================================================
 * Self-test state machine
 * ============================================================================ */

static void selftest_process(void)
{
	switch (selftest_state) {

	case SELFTEST_TURN_ON:
		selftest_result[selftest_led] = LED_SELFTEST_RUNNING;
		reset_selftest_window();
		selftest_wait_ticks = 0U;

		/* Request this LED on; control_task() drives the GPIO. */
		selftest_outputs_off();
		selftest_out[selftest_led] = true;

		selftest_state = SELFTEST_WAIT;
		break;

	case SELFTEST_WAIT:
		if (++selftest_wait_ticks >= SELFTEST_SETTLE_TICKS) {
			selftest_state = SELFTEST_SAMPLE;
		}
		break;

	case SELFTEST_SAMPLE: {
		uint16_t mv = adc_read_mv(led_cfg[selftest_led].adc_channel);

		leds[selftest_led].voltage = mv; /* keep normal API current */
		classify_selftest(selftest_led, mv);

		if (++selftest_samples < SELFTEST_SAMPLES) {
			break;
		}

		led_fault_state_t result = decide_selftest();

		if (result == LED_OK) {
			selftest_result[selftest_led] = LED_SELFTEST_PASS;
			leds[selftest_led].state = LED_OK;
		} else {
			selftest_result[selftest_led] = LED_SELFTEST_FAIL;
			leds[selftest_led].state = result; /* preserve fault type */
		}

		selftest_state = SELFTEST_TURN_OFF;
		break;
	}

	case SELFTEST_TURN_OFF:
		selftest_outputs_off();

		/* Do not let normal debounce inherit self-test samples. */
		reset_window(selftest_led);
		leds[selftest_led].prev_on = false;

		if (++selftest_led >= LED_COUNT) {
			selftest_running = false;
			selftest_state = SELFTEST_COMPLETE;
		} else {
			reset_selftest_window();
			selftest_wait_ticks = 0U;
			selftest_state = SELFTEST_TURN_ON;
		}
		break;

	case SELFTEST_IDLE:
	case SELFTEST_COMPLETE:
		break;

	default:
		selftest_state = SELFTEST_COMPLETE;
		selftest_running = false;
		selftest_outputs_off();
		break;
	}
}

/* ============================================================================
 * Public: initialization
 * ============================================================================ */

void led_fault_init(void)
{
	for (uint8_t i = 0U; i < LED_COUNT; i++) {
		leds[i].state = LED_OK;
		leds[i].voltage = 0U;
		leds[i].prev_on = false;
		reset_window(i);

		selftest_result[i] = LED_SELFTEST_NOT_RUN;
		selftest_out[i] = false;
	}

	next_led = 0U;

	selftest_state = SELFTEST_IDLE;
	selftest_running = false;
	selftest_led = 0U;
	selftest_wait_ticks = 0U;
	reset_selftest_window();
}

/* ============================================================================
 * Public: normal update + self-test dispatch
 * ============================================================================ */

void led_fault_update(bool left_on, bool right_on, bool low_on, bool high_on)
{
	/* Normal monitoring is suspended while the startup self-test owns
	 * the LEDs. */
	if (selftest_running) {
		selftest_process();
		return;
	}

	const bool on_state[LED_COUNT] = { left_on, right_on, low_on, high_on };
	uint8_t led = next_led;
	bool on = on_state[led];

	/* A physical-state change discards any partial debounce window --
	 * samples taken under the old state are not valid evidence for the
	 * new one. */
	if (on != leds[led].prev_on) {
		reset_window(led);
		leds[led].prev_on = on;
	}

	/* Only sample while physically commanded on; while off, hold the
	 * last known fault state and debounce window. */
	if (on) {
		uint16_t mv = adc_read_mv(led_cfg[led].adc_channel);

		leds[led].voltage = mv;
		classify(led, mv);

		if (++leds[led].sample_count >= DEBOUNCE_SAMPLES) {
			leds[led].state = decide(led);
			reset_window(led);
		}
	}

	next_led = (uint8_t)((next_led + 1U) % LED_COUNT);
}

/* ============================================================================
 * Public: normal status
 * ============================================================================ */

led_fault_state_t led_fault_get(uint8_t led)
{
	return (led >= LED_COUNT) ? LED_OPEN /* fail-safe */ : leds[led].state;
}

uint16_t led_fault_voltage(uint8_t led)
{
	return (led >= LED_COUNT) ? 0U : leds[led].voltage;
}

/* ============================================================================
 * Public: self-test control + status
 * ============================================================================ */

void led_fault_selftest_start(void)
{
	for (uint8_t i = 0U; i < LED_COUNT; i++) {
		selftest_result[i] = LED_SELFTEST_NOT_RUN;
	}
	selftest_outputs_off();

	selftest_led = 0U;
	selftest_wait_ticks = 0U;
	reset_selftest_window();

	selftest_running = true;
	selftest_state = SELFTEST_TURN_ON;
}

bool led_fault_selftest_running(void)
{
	return selftest_running;
}

bool led_fault_selftest_complete(void)
{
	return selftest_state == SELFTEST_COMPLETE;
}

bool led_fault_selftest_passed(void)
{
	if (!led_fault_selftest_complete()) {
		return false;
	}
	for (uint8_t i = 0U; i < LED_COUNT; i++) {
		if (selftest_result[i] != LED_SELFTEST_PASS) {
			return false;
		}
	}
	return true;
}

led_selftest_state_t led_fault_selftest_get(uint8_t led)
{
	return (led >= LED_COUNT) ? LED_SELFTEST_FAIL : selftest_result[led];
}

led_selftest_state_t led_fault_selftest_state(void)
{
	if (selftest_running) {
		return LED_SELFTEST_RUNNING;
	}
	if (selftest_state == SELFTEST_IDLE) {
		return LED_SELFTEST_NOT_RUN;
	}
	if (selftest_state != SELFTEST_COMPLETE) {
		return LED_SELFTEST_RUNNING;
	}
	return led_fault_selftest_passed() ? LED_SELFTEST_PASS : LED_SELFTEST_FAIL;
}

/*
 * Physical outputs requested by the startup self-test. NULL out-parameters
 * are accepted and skipped -- callers may only want a subset.
 */
void led_fault_selftest_outputs(bool *left_on, bool *right_on, bool *low_on,
				bool *high_on)
{
	if (left_on) {
		*left_on = selftest_out[0];
	}
	if (right_on) {
		*right_on = selftest_out[1];
	}
	if (low_on) {
		*low_on = selftest_out[2];
	}
	if (high_on) {
		*high_on = selftest_out[3];
	}
}