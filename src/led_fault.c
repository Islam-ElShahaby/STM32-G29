#include "led_fault.h"
#include "adc.h"


/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

/*
 * Number of normal samples required before deciding a fault.
 *
 * IMPORTANT:
 *
 * led_fault_update() is called every 10 ms, but only one LED is sampled
 * on each call.
 *
 * Therefore each individual LED gets a sample approximately every:
 *
 *     4 * 10 ms = 40 ms
 *
 * Five samples therefore represent approximately:
 *
 *     5 * 40 ms = 200 ms
 */
#define DEBOUNCE_SAMPLES       5U


/*
 * Number of ADC samples used by the startup self-test for each LED.
 */
#define SELFTEST_SAMPLES       5U


/*
 * Number of control ticks to wait after physically turning an LED ON
 * before taking the first ADC sample.
 *
 * At a 10 ms control period:
 *
 *     2 ticks = 20 ms
 */
#define SELFTEST_SETTLE_TICKS  2U


/* ============================================================================
 * STATIC HARDWARE CONFIGURATION
 * ============================================================================ */

typedef struct
{
    uint32_t adc_channel;
    uint16_t ok_min_mv;
    uint16_t ok_max_mv;
    uint16_t margin_mv;

} led_config_t;


/*
 * Hardware mapping for each LED.
 */
static const led_config_t led_cfg[LED_COUNT] =
{
    /*
     * [LED] = { ADC channel, minimum OK, maximum OK, margin }
     */

    [0] = {
        ADC_CHANNEL_0,
        2000U,
        2300U,
        200U
    },

    [1] = {
        ADC_CHANNEL_2,
        2000U,
        2300U,
        200U
    },

    [2] = {
        ADC_CHANNEL_3,
        2700U,
        2800U,
        200U
    },

    [3] = {
        ADC_CHANNEL_9,
        2700U,
        2800U,
        200U
    }
};


/* ============================================================================
 * NORMAL RUNTIME STATE
 * ============================================================================ */

typedef struct
{
    led_fault_state_t state;

    uint16_t voltage;

    uint8_t good_count;
    uint8_t low_count;
    uint8_t high_count;
    uint8_t sample_count;

    bool prev_on;

} led_channel_t;


static led_channel_t leds[LED_COUNT];


/*
 * Round-robin sampling cursor.
 */
static uint8_t next_led;


/* ============================================================================
 * SELF-TEST STATE
 * ============================================================================ */

typedef enum
{
    SELFTEST_IDLE = 0,

    /*
     * Turn the current LED ON.
     */
    SELFTEST_TURN_ON,

    /*
     * Wait for the electrical node to settle.
     */
    SELFTEST_WAIT,

    /*
     * Take ADC samples.
     */
    SELFTEST_SAMPLE,

    /*
     * Turn current LED OFF and move to next LED.
     */
    SELFTEST_TURN_OFF,

    /*
     * All LEDs tested.
     */
    SELFTEST_COMPLETE

} selftest_state_t;


static selftest_state_t selftest_state;

static bool selftest_running;

static uint8_t selftest_led;

static uint8_t selftest_wait_ticks;

static uint8_t selftest_samples;

static uint8_t selftest_good;
static uint8_t selftest_low;
static uint8_t selftest_high;


/*
 * Result for each LED.
 */
static led_selftest_state_t selftest_result[LED_COUNT];


/*
 * Outputs requested by the self-test.
 *
 * These are NOT GPIO registers.
 *
 * main/control_task reads these values and passes them to
 * lights_gpio_set().
 */
static bool selftest_out[LED_COUNT];


/* ============================================================================
 * NORMAL FAULT HELPERS
 * ============================================================================ */

/**
 * @brief Reset normal debounce window.
 */
static void reset_window(uint8_t led)
{
    leds[led].good_count   = 0U;
    leds[led].low_count    = 0U;
    leds[led].high_count   = 0U;
    leds[led].sample_count = 0U;
}


/**
 * @brief Classify one ADC reading.
 *
 * LOW  = short circuit region
 * HIGH = open circuit region
 * GOOD = normal voltage region
 */
static void classify(uint8_t led, uint16_t mv)
{
    const led_config_t *cfg = &led_cfg[led];

    uint16_t trip_low;
    uint16_t trip_high;

    uint16_t gap_low_mid;
    uint16_t gap_high_mid;


    /*
     * Configuration currently uses margin < OK minimum, so these
     * calculations are safe for the present configuration.
     */
    trip_low =
        cfg->ok_min_mv - cfg->margin_mv;

    trip_high =
        cfg->ok_max_mv + cfg->margin_mv;


    gap_low_mid =
        (trip_low + cfg->ok_min_mv) / 2U;

    gap_high_mid =
        (cfg->ok_max_mv + trip_high) / 2U;


    if (mv < gap_low_mid)
    {
        leds[led].low_count++;
    }
    else if (mv > gap_high_mid)
    {
        leds[led].high_count++;
    }
    else
    {
        leds[led].good_count++;
    }
}


/**
 * @brief Majority decision for normal monitoring.
 */
static led_fault_state_t decide(uint8_t led)
{
    uint8_t g = leds[led].good_count;
    uint8_t l = leds[led].low_count;
    uint8_t h = leds[led].high_count;


    /*
     * GOOD wins ties.
     */
    if ((g >= l) && (g >= h))
    {
        return LED_OK;
    }


    /*
     * OPEN wins if it is at least as large as the other two.
     */
    if ((h >= g) && (h >= l))
    {
        return LED_OPEN;
    }


    /*
     * LOW is largest by elimination.
     */
    return LED_SHORT;
}


/* ============================================================================
 * SELF-TEST HELPERS
 * ============================================================================ */

static void reset_selftest_window(void)
{
    selftest_samples = 0U;

    selftest_good = 0U;
    selftest_low  = 0U;
    selftest_high = 0U;
}


/**
 * @brief Clear all self-test requested outputs.
 */
static void selftest_outputs_off(void)
{
    for (uint8_t i = 0U; i < LED_COUNT; i++)
    {
        selftest_out[i] = false;
    }
}


/**
 * @brief Classify one self-test ADC reading.
 *
 * This intentionally uses the same voltage boundaries as the normal
 * fault detector.
 */
static void classify_selftest(uint8_t led, uint16_t mv)
{
    const led_config_t *cfg = &led_cfg[led];

    uint16_t trip_low;
    uint16_t trip_high;

    uint16_t gap_low_mid;
    uint16_t gap_high_mid;


    trip_low =
        cfg->ok_min_mv - cfg->margin_mv;

    trip_high =
        cfg->ok_max_mv + cfg->margin_mv;


    gap_low_mid =
        (trip_low + cfg->ok_min_mv) / 2U;

    gap_high_mid =
        (cfg->ok_max_mv + trip_high) / 2U;


    if (mv < gap_low_mid)
    {
        selftest_low++;
    }
    else if (mv > gap_high_mid)
    {
        selftest_high++;
    }
    else
    {
        selftest_good++;
    }
}


/**
 * @brief Decide result for the current self-test LED.
 */
static led_fault_state_t decide_selftest(void)
{
    if ((selftest_good >= selftest_low) &&
        (selftest_good >= selftest_high))
    {
        return LED_OK;
    }


    if ((selftest_high >= selftest_good) &&
        (selftest_high >= selftest_low))
    {
        return LED_OPEN;
    }


    return LED_SHORT;
}


/* ============================================================================
 * SELF-TEST STATE MACHINE
 * ============================================================================ */

static void selftest_process(void)
{
    uint16_t mv;
    led_fault_state_t result;


    switch (selftest_state)
    {
        /* -----------------------------------------------------------------
         * Turn current LED ON.
         * ----------------------------------------------------------------- */
        case SELFTEST_TURN_ON:

            selftest_result[selftest_led] =
                LED_SELFTEST_RUNNING;


            reset_selftest_window();

            selftest_wait_ticks = 0U;


            /*
             * Request this LED ON.
             *
             * control_task() will physically drive the GPIO.
             */
            selftest_outputs_off();

            selftest_out[selftest_led] = true;


            selftest_state = SELFTEST_WAIT;

            break;


        /* -----------------------------------------------------------------
         * Wait for voltage to settle.
         * ----------------------------------------------------------------- */
        case SELFTEST_WAIT:

            selftest_wait_ticks++;

            if (selftest_wait_ticks >= SELFTEST_SETTLE_TICKS)
            {
                selftest_state = SELFTEST_SAMPLE;
            }

            break;


        /* -----------------------------------------------------------------
         * ADC sampling.
         * ----------------------------------------------------------------- */
        case SELFTEST_SAMPLE:

            mv =
                adc_read_mv(
                    led_cfg[selftest_led].adc_channel
                );


            /*
             * Keep latest voltage available to the normal API.
             */
            leds[selftest_led].voltage = mv;


            classify_selftest(
                selftest_led,
                mv
            );


            selftest_samples++;


            if (selftest_samples >= SELFTEST_SAMPLES)
            {
                result = decide_selftest();


                if (result == LED_OK)
                {
                    selftest_result[selftest_led] =
                        LED_SELFTEST_PASS;

                    /*
                     * A passed self-test means the physical circuit
                     * currently looks healthy.
                     */
                    leds[selftest_led].state = LED_OK;
                }
                else
                {
                    selftest_result[selftest_led] =
                        LED_SELFTEST_FAIL;

                    /*
                     * Preserve the actual fault type for telemetry.
                     */
                    leds[selftest_led].state = result;
                }


                selftest_state =
                    SELFTEST_TURN_OFF;
            }

            break;


        /* -----------------------------------------------------------------
         * Turn current LED OFF.
         * ----------------------------------------------------------------- */
        case SELFTEST_TURN_OFF:

            selftest_outputs_off();


            /*
             * Do not allow the normal debounce state to inherit
             * self-test samples.
             */
            reset_window(selftest_led);

            leds[selftest_led].prev_on = false;


            selftest_led++;


            if (selftest_led >= LED_COUNT)
            {
                /*
                 * Every LED has now been tested.
                 */
                selftest_running = false;

                selftest_state =
                    SELFTEST_COMPLETE;
            }
            else
            {
                reset_selftest_window();

                selftest_wait_ticks = 0U;

                selftest_state =
                    SELFTEST_TURN_ON;
            }

            break;


        case SELFTEST_IDLE:
            break;


        case SELFTEST_COMPLETE:
            break;


        default:
            selftest_state =
                SELFTEST_COMPLETE;

            selftest_running = false;

            selftest_outputs_off();

            break;
    }
}


/* ============================================================================
 * PUBLIC INITIALIZATION
 * ============================================================================ */

void led_fault_init(void)
{
    for (uint8_t i = 0U; i < LED_COUNT; i++)
    {
        leds[i].state   = LED_OK;
        leds[i].voltage = 0U;

        leds[i].prev_on = false;

        reset_window(i);

        selftest_result[i] =
            LED_SELFTEST_NOT_RUN;

        selftest_out[i] = false;
    }


    next_led = 0U;


    selftest_state =
        SELFTEST_IDLE;

    selftest_running = false;

    selftest_led = 0U;

    selftest_wait_ticks = 0U;

    reset_selftest_window();
}


/* ============================================================================
 * NORMAL UPDATE + SELF-TEST
 * ============================================================================ */

void led_fault_update(bool left_on,
                      bool right_on,
                      bool low_on,
                      bool high_on)
{
    /*
     * During startup self-test, the normal fault monitor is suspended.
     */
    if (selftest_running)
    {
        selftest_process();
        return;
    }


    const bool on_state[LED_COUNT] =
    {
        left_on,
        right_on,
        low_on,
        high_on
    };


    uint8_t led =
        next_led;

    bool on =
        on_state[led];


    /*
     * If physical command changed state, discard the partial debounce
     * window.
     */
    if (on != leds[led].prev_on)
    {
        reset_window(led);

        leds[led].prev_on = on;
    }


    /*
     * Only sample when physically commanded ON.
     *
     * When OFF, the current fault state and debounce window are held.
     */
    if (on)
    {
        uint16_t mv =
            adc_read_mv(
                led_cfg[led].adc_channel
            );


        leds[led].voltage =
            mv;


        classify(
            led,
            mv
        );


        leds[led].sample_count++;


        if (leds[led].sample_count >=
            DEBOUNCE_SAMPLES)
        {
            leds[led].state =
                decide(led);

            reset_window(led);
        }
    }


    /*
     * Next LED in round-robin sequence.
     */
    next_led =
        (uint8_t)((next_led + 1U) % LED_COUNT);
}


/* ============================================================================
 * NORMAL STATUS API
 * ============================================================================ */

led_fault_state_t led_fault_get(uint8_t led)
{
    if (led >= LED_COUNT)
    {
        /*
         * Fail-safe result for invalid index.
         */
        return LED_OPEN;
    }


    return leds[led].state;
}


uint16_t led_fault_voltage(uint8_t led)
{
    if (led >= LED_COUNT)
    {
        return 0U;
    }


    return leds[led].voltage;
}


/* ============================================================================
 * SELF-TEST API
 * ============================================================================ */

void led_fault_selftest_start(void)
{
    for (uint8_t i = 0U; i < LED_COUNT; i++)
    {
        selftest_result[i] =
            LED_SELFTEST_NOT_RUN;
    }


    selftest_outputs_off();


    selftest_led = 0U;

    selftest_wait_ticks = 0U;


    reset_selftest_window();


    selftest_running = true;

    selftest_state =
        SELFTEST_TURN_ON;
}


bool led_fault_selftest_running(void)
{
    return selftest_running;
}


bool led_fault_selftest_complete(void)
{
    return
        (selftest_state ==
         SELFTEST_COMPLETE);
}


bool led_fault_selftest_passed(void)
{
    if (!led_fault_selftest_complete())
    {
        return false;
    }


    for (uint8_t i = 0U; i < LED_COUNT; i++)
    {
        if (selftest_result[i] !=
            LED_SELFTEST_PASS)
        {
            return false;
        }
    }


    return true;
}


led_selftest_state_t led_fault_selftest_get(uint8_t led)
{
    if (led >= LED_COUNT)
    {
        return LED_SELFTEST_FAIL;
    }


    return selftest_result[led];
}


led_selftest_state_t led_fault_selftest_state(void)
{
    if (selftest_running)
    {
        return LED_SELFTEST_RUNNING;
    }


    if (selftest_state ==
        SELFTEST_IDLE)
    {
        return LED_SELFTEST_NOT_RUN;
    }


    if (selftest_state !=
        SELFTEST_COMPLETE)
    {
        return LED_SELFTEST_RUNNING;
    }


    if (led_fault_selftest_passed())
    {
        return LED_SELFTEST_PASS;
    }


    return LED_SELFTEST_FAIL;
}


/**
 * @brief Return physical outputs requested by startup self-test.
 */
void led_fault_selftest_outputs(bool *left_on,
                                bool *right_on,
                                bool *low_on,
                                bool *high_on)
{
    if (left_on != NULL)
    {
        *left_on =
            selftest_out[0];
    }


    if (right_on != NULL)
    {
        *right_on =
            selftest_out[1];
    }


    if (low_on != NULL)
    {
        *low_on =
            selftest_out[2];
    }


    if (high_on != NULL)
    {
        *high_on =
            selftest_out[3];
    }
}