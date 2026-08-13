#include "led_fault.h"
#include "adc.h"

/* ============================================================================
 * CONFIGURATION & DEFINITIONS
 * ============================================================================ */

/* Number of samples required before declaring a final fault/OK decision */
#define DEBOUNCE_SAMPLES 5U

/**
 * @brief Static hardware configuration for an LED channel.
 * Holds voltage thresholds and ADC mapping for each LED independently.
 */
typedef struct {
    uint32_t adc_channel;  /* Microcontroller ADC channel mapped to this LED */
    uint16_t ok_min_mv;    /* Minimum normal operating voltage (mV) */
    uint16_t ok_max_mv;    /* Maximum normal operating voltage (mV) */
    uint16_t margin_mv;    /* Buffer margin above/below normal operating range (mV) */
} led_config_t;

/* Array mapping hardware setup per LED channel */
static const led_config_t led_cfg[LED_COUNT] = {
    /* [Channel] = { ADC Channel,   Min OK (mV), Max OK (mV), Margin (mV) } */
    [0]          = { ADC_CHANNEL_0, 2000U,       2300U,       200U }, /* LED 0: Left indicator */
    [1]          = { ADC_CHANNEL_2, 2000U,       2300U,       200U }, /* LED 1: Right indicator */
    [2]          = { ADC_CHANNEL_3, 2700U,       2800U,       200U }, /* LED 2: Low beam */
    [3]          = { ADC_CHANNEL_9, 2700U,       2800U,       200U }, /* LED 3: High beam */
};

/**
 * @brief Dynamic runtime state for an LED channel.
 * Holds active measurements, debounce counters, and output states.
 */
typedef struct {
    led_fault_state_t state;  /* Current fault state (LED_OK, LED_OPEN, LED_SHORT) */
    uint16_t voltage;         /* Latest sampled voltage level in millivolts */
    uint8_t good_count;       /* Counter for samples in the HEALTHY voltage range */
    uint8_t low_count;        /* Counter for samples in the SHORT-CIRCUIT voltage range */
    uint8_t high_count;       /* Counter for samples in the OPEN-CIRCUIT voltage range */
    uint8_t sample_count;     /* Total samples accumulated in current debounce window */
    bool prev_on;             /* Command status (ON/OFF) during the previous tick */
} led_channel_t;

/* Global runtime state table for all channels */
static led_channel_t leds[LED_COUNT];

/* Round-robin cursor indicating which LED will be sampled on the current tick */
static uint8_t next_led;


/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

/**
 * @brief Clears sample accumulators to start a fresh debounce window.
 * 
 * @param led Index of the LED channel to reset.
 */
static void reset_window(uint8_t led) {
    leds[led].good_count   = 0U;
    leds[led].low_count    = 0U;
    leds[led].high_count   = 0U;
    leds[led].sample_count = 0U;
}

/**
 * @brief Categorizes a single ADC reading into LOW, HIGH, or GOOD counts.
 * 
 * Determines trip limits and midpoints dynamically from the LED configuration:
 * - Below Low-Gap Midpoint  -> Classified as SHORT (low_count++)
 * - Above High-Gap Midpoint -> Classified as OPEN  (high_count++)
 * - Anywhere between        -> Classified as OK    (good_count++)
 *
 * @param led Index of the LED being classified.
 * @param mv  Measured voltage in millivolts.
 */
static void classify(uint8_t led, uint16_t mv) {
    const led_config_t *cfg = &led_cfg[led];

    /* Calculate Trip Limits */
    uint16_t trip_low  = cfg->ok_min_mv - cfg->margin_mv;
    uint16_t trip_high = cfg->ok_max_mv + cfg->margin_mv;

    /* Calculate Midpoints for the transitional gap zones */
    uint16_t gap_low_mid  = (trip_low + cfg->ok_min_mv) / 2U;
    uint16_t gap_high_mid = (cfg->ok_max_mv + trip_high) / 2U;

    /* Single classification check */
    if (mv < gap_low_mid) {
        leds[led].low_count++;      /* Voltage is in SHORT range */
    } else if (mv > gap_high_mid) {
        leds[led].high_count++;     /* Voltage is in OPEN range */
    } else {
        leds[led].good_count++;     /* Voltage is in HEALTHY range */
    }
}

/**
 * @brief Evaluates debounce counts using majority vote to dictate final status.
 *
 * FIXED: was a STRICT majority (>), which meant any tied vote (e.g. 2
 * good / 2 low / 1 high out of 5 samples -- not rare with only 3 buckets)
 * fell through to "keep the previous state" and returned early, NEVER
 * updating leds[led].state or leds[led].voltage's associated verdict for
 * that window. Worse, a tie on the very first window a channel is ever
 * sampled (e.g. right at power-up before the node has settled) could
 * latch a wrong verdict that no later window could ever clear, because
 * ties keep recurring indifferently to what the readings actually show
 * afterward -- a channel could sit on a false SHORT/OPEN forever even
 * while every individual sample coming in was perfectly healthy.
 *
 * Now uses a >= cascade in fixed priority order (good, then open, then
 * short by elimination). Among any three counts the largest always
 * satisfies >= against both others, so this ALWAYS resolves to a real
 * verdict every single window -- it can never fall through and silently
 * keep a stale state the way the strict version could.
 *
 * @param led Index of the LED to decide.
 * @return Decided fault state enum (LED_OK, LED_OPEN, or LED_SHORT).
 */
static led_fault_state_t decide(uint8_t led) {
    uint8_t g = leds[led].good_count;
    uint8_t l = leds[led].low_count;
    uint8_t h = leds[led].high_count;

    if (g >= l && g >= h) {
        return LED_OK;
    }
    if (h >= g && h >= l) {
        return LED_OPEN;
    }
    return LED_SHORT;   /* l is the largest of the three by elimination */
}


/* ============================================================================
 * PUBLIC INTERFACE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initializes module state and resets sample buffers.
 * Must be called during system startup before starting the main loop.
 */
void led_fault_init(void) {
    for (uint8_t i = 0U; i < LED_COUNT; i++) {
        leds[i].state   = LED_OK;
        leds[i].voltage = 0U;
        leds[i].prev_on = false;
        reset_window(i);
    }
    next_led = 0U; /* Start round-robin loop at first channel */
}

/**
 * @brief Primary update function. Call periodically (e.g., every 10 ms).
 *
 * Evaluates ONE LED channel per call using round-robin logic.
 *
 * @param left_on  Command state for Left Indicator (LED 0)
 * @param right_on Command state for Right Indicator (LED 1)
 * @param low_on   Command state for Low Beam (LED 2)
 * @param high_on  Command state for High Beam (LED 3)
 */
void led_fault_update(bool left_on, bool right_on, bool low_on, bool high_on) {
    /* Map incoming control arguments to array index corresponding to led_cfg */
    const bool on_state[LED_COUNT] = { left_on, right_on, low_on, high_on };
    
    uint8_t led = next_led;
    bool on = on_state[led];

    /* 1. DISCARD WINDOW ON STATE SWITCH
     * If the LED turned ON or OFF mid-way, discard accumulated samples 
     * because operating conditions changed. */
    if (on != leds[led].prev_on) {
        reset_window(led);
        leds[led].prev_on = on;
    }

    /* 2. SAMPLE & DEBOUNCE (Only active when LED is commanded ON) */
    if (on) {
        /* Read physical voltage for current channel */
        uint16_t mv = adc_read_mv(led_cfg[led].adc_channel);
        leds[led].voltage = mv;

        /* Sort voltage sample into vote accumulators */
        classify(led, mv);
        leds[led].sample_count++;

        /* Check if debounce window completed */
        if (leds[led].sample_count >= DEBOUNCE_SAMPLES) {
            leds[led].state = decide(led); /* Apply majority vote verdict */
            reset_window(led);             /* Prepare for next debounce window */
        }
    }

    /* 3. ADVANCE CURSOR
     * Increment round-robin cursor for next task tick */
    next_led = (next_led + 1U) % LED_COUNT;
}

/**
 * @brief Reads the current processed fault status of an LED.
 * 
 * @param led Index of the LED.
 * @return Current fault state (Defaults to LED_OPEN on invalid index for safety).
 */
led_fault_state_t led_fault_get(uint8_t led) {
    if (led >= LED_COUNT) {
        return LED_OPEN; /* Fail-safe return */
    }
    return leds[led].state;
}

/**
 * @brief Reads the latest measured ADC millivolt value for an LED.
 * 
 * @param led Index of the LED.
 * @return Latest raw reading in mV (0 mV if invalid index).
 */
uint16_t led_fault_voltage(uint8_t led) {
    if (led >= LED_COUNT) {
        return 0U;
    }
    return leds[led].voltage;
}