/*
 * ADC1 driver -- single-conversion, software-triggered, all channels
 * time-multiplexed through one hardware instance.
 *
 * Channels in use:
 *   PA0 = ADC_CHANNEL_0  -> LED fault sense: left indicator
 *   PA1 = ADC_CHANNEL_1  -> Shifter (PRND pot)          [owned by shifter.c]
 *   PA2 = ADC_CHANNEL_2  -> LED fault sense: right indicator
 *   PA3 = ADC_CHANNEL_3  -> LED fault sense: low beam
 *   PA4 = ADC_CHANNEL_4  -> Fuel sender
 *   PA5 = ADC_CHANNEL_5  -> Temperature sender
 *   PA6 = ADC_CHANNEL_6  -> Servo position feedback
 *   PB1 = ADC_CHANNEL_9  -> LED fault sense: high beam
 *
 * CONCURRENCY
 * -----------
 * hadc1 is one hardware peripheral shared by three call sites running in
 * two different FreeRTOS tasks:
 *   - control_task  -> shifter_update()      (channel 1)
 *   - can_task       -> adc_update()          (channels 4, 5)
 *                    -> adc_get_servo_angle() (channel 6)
 *   - can_task       -> led_fault self-test / debounce (channels 0, 2, 3, 9)
 *
 * A bare HAL_ADC_ConfigChannel/Start/PollForConversion/Stop sequence is not
 * safe to interleave: if one task's conversion is pre-empted mid-sequence by
 * another task reconfiguring and restarting the same peripheral, the first
 * task can read back a stale value, a value from the wrong channel, or hang
 * in HAL_ADC_PollForConversion() waiting on a conversion that was never
 * started the way it expects.
 *
 * adc_mutex serializes the whole read (config -> start -> poll -> stop) so
 * only one task ever touches hadc1 at a time. It is created in adc_init(),
 * which is called from main() before the scheduler starts, so the mutex
 * exists before any task that could contend for it.
 */
#include "adc.h"



/* ============================================================================
 * Configuration
 * ============================================================================ */

#define ADC_TIMEOUT_MS      2U   /* HAL_ADC_PollForConversion() budget */
#define ADC_LOCK_TIMEOUT_MS 5U   /* Mutex wait budget -- see note below */

#define ADC_MAX_VALUE       4095U /* 12-bit full scale */
#define ADC_MAX_PERCENT     100U

#define HYSTERESIS_PERCENT  5U

/*
 * ADC_LOCK_TIMEOUT_MS is deliberately longer than ADC_TIMEOUT_MS: the worst
 * case a caller should ever wait is "someone else's whole conversion", which
 * is bounded by ADC_TIMEOUT_MS plus HAL overhead, not an arbitrary stall. If
 * the lock is not acquired in time, the read fails closed (returns 0), the
 * same behavior every other failure path in this file already uses -- a
 * caller cannot tell a lock timeout from a conversion timeout, and does not
 * need to.
 */

/* ============================================================================
 * State
 * ============================================================================ */

static ADC_HandleTypeDef hadc1;
static SemaphoreHandle_t adc_mutex;

static bool adc_initialized;

/* Cached, hysteresis-filtered application values -- see adc_update(). */
static uint8_t fuel_value;
static uint8_t temperature_value;

/* ============================================================================
 * Locking helpers
 * ============================================================================ */

static bool adc_lock(void)
{
	if (adc_mutex == NULL) {
		return false;
	}
	return xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(ADC_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static void adc_unlock(void)
{
	if (adc_mutex != NULL) {
		xSemaphoreGive(adc_mutex);
	}
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

void adc_init(void)
{
	GPIO_InitTypeDef gpio = {0};

	adc_mutex = xSemaphoreCreateMutex();
	if (adc_mutex == NULL) {
		/* Out of heap this early is unrecoverable -- nothing downstream
		 * can safely share the peripheral without it. */
		adc_initialized = false;
		return;
	}

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	/* PA0-PA6: all seven analog inputs on GPIOA (see channel map above). */
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
		   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	/* PB1: high-beam fault sense (ADC_CHANNEL_9), the one channel not on
	 * GPIOA. Reuses the same GPIO_InitTypeDef (still GPIO_MODE_ANALOG). */
	gpio.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &gpio);

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution             = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode           = DISABLE;
	hadc1.Init.ContinuousConvMode     = DISABLE;
	hadc1.Init.DiscontinuousConvMode  = DISABLE;
	hadc1.Init.ExternalTrigConv       = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion        = 1;
	hadc1.Init.DMAContinuousRequests  = DISABLE;
	hadc1.Init.EOCSelection           = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		adc_initialized = false;
		return;
	}

	adc_initialized = true;

	fuel_value = 0U;
	temperature_value = 0U;
}

/* ============================================================================
 * Single-channel read
 * ============================================================================ */

uint16_t adc_read_channel(uint32_t channel)
{
	ADC_ChannelConfTypeDef cfg = {0};
	uint16_t value = 0U;

	if (!adc_initialized) {
		return 0U;
	}
	if (!adc_lock()) {
		return 0U;
	}

	cfg.Channel = channel;
	cfg.Rank = 1;
	cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) {
		goto out;
	}
	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		goto out;
	}
	if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
		HAL_ADC_Stop(&hadc1);
		goto out;
	}

	value = (uint16_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

out:
	adc_unlock();
	return value;
}

uint16_t adc_read_mv(uint32_t channel)
{
	uint32_t raw = adc_read_channel(channel);

	return (uint16_t)((raw * 3300U) / ADC_MAX_VALUE);
}

/* ============================================================================
 * Servo feedback
 * ============================================================================ */

/*
 * PA6 / ADC_CHANNEL_6. Raw range 0..4095 maps linearly to 0..180 degrees:
 *
 *     angle = raw * 180 / 4095
 *
 * Returned as a plain degree value -- intentionally NOT passed through the
 * fuel/temperature hysteresis filter, since servo position is a live control
 * feedback signal, not a slow-moving sender reading.
 */
uint8_t adc_get_servo_angle(void)
{
	uint16_t raw = adc_read_channel(ADC_CHANNEL_6);

	if (raw >= ADC_MAX_VALUE) {
		return 180U;
	}
	return (uint8_t)(((uint32_t)raw * 180U) / ADC_MAX_VALUE);
}

/* ============================================================================
 * Fuel / temperature: raw -> percent, with hysteresis
 * ============================================================================ */

static uint8_t adc_raw_to_percent(uint16_t raw)
{
	if (raw >= ADC_MAX_VALUE) {
		return 100U;
	}
	return (uint8_t)(((uint32_t)raw * 100U) / ADC_MAX_VALUE);
}

/*
 * Schmitt-trigger-style hysteresis on the raw ADC value, evaluated in the
 * percent domain: current_percent only moves once the instantaneous reading
 * crosses HYSTERESIS_PERCENT past the next boundary in that direction. This
 * stops a sender reading that sits exactly on a percent boundary from
 * flickering the reported value up and down every sample.
 */
static uint8_t adc_apply_hysteresis(uint16_t raw, uint8_t current_percent)
{
	uint8_t instantaneous = adc_raw_to_percent(raw);
	uint32_t hysteresis = (HYSTERESIS_PERCENT * ADC_MAX_VALUE) / ADC_MAX_PERCENT;

	if (current_percent >= 100U) {
		uint32_t lower = (ADC_MAX_VALUE > hysteresis) ? (ADC_MAX_VALUE - hysteresis) : 0U;

		return (raw < lower) ? 99U : current_percent;
	}

	if (instantaneous > current_percent) {
		uint32_t next_boundary = ((uint32_t)(current_percent + 1U) * ADC_MAX_VALUE) / ADC_MAX_PERCENT;
		uint32_t upper = next_boundary + hysteresis;

		if (upper > ADC_MAX_VALUE) {
			upper = ADC_MAX_VALUE;
		}
		return (raw >= upper) ? (current_percent + 1U) : current_percent;
	}

	if (instantaneous < current_percent) {
		uint32_t boundary = ((uint32_t)current_percent * ADC_MAX_VALUE) / ADC_MAX_PERCENT;
		uint32_t lower = (boundary > hysteresis) ? (boundary - hysteresis) : 0U;

		return (raw < lower) ? (current_percent - 1U) : current_percent;
	}

	return current_percent;
}

void adc_update(void)
{
	if (!adc_initialized) {
		return;
	}

	uint16_t fuel_raw = adc_read_channel(ADC_CHANNEL_4);
	uint16_t temp_raw = adc_read_channel(ADC_CHANNEL_5);

	fuel_value = adc_apply_hysteresis(fuel_raw, fuel_value);
	temperature_value = adc_apply_hysteresis(temp_raw, temperature_value);
}

uint8_t adc_get_fuel(void)
{
	return fuel_value;
}

uint8_t adc_get_temperature(void)
{
	return temperature_value;
}