#ifndef LED_FAULT_H
#define LED_FAULT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "adc.h"
#include "stm32f4xx_hal.h"


#define LED_COUNT 4U


typedef enum
{
    LED_OK = 0,
    LED_OPEN,
    LED_SHORT
} led_fault_state_t;


/*
 * Startup self-test state.
 *
 * NOT_RUN  : self-test has not started
 * RUNNING  : self-test is currently testing the LEDs
 * PASS     : all LEDs passed
 * FAIL     : one or more LEDs failed
 */
typedef enum
{
    LED_SELFTEST_NOT_RUN = 0,
    LED_SELFTEST_RUNNING,
    LED_SELFTEST_PASS,
    LED_SELFTEST_FAIL
} led_selftest_state_t;


/*
 * Normal LED fault monitoring.
 */
void led_fault_init(void);

/**
 * @brief Samples one LED using round-robin scheduling.
 *
 * One LED is sampled per call.
 *
 * When the LED is commanded OFF, its previous fault state is held.
 *
 * When commanded ON, ADC samples are accumulated until the debounce
 * window completes.
 */
void led_fault_update(bool left_on,
                     bool right_on,
                     bool low_on,
                     bool high_on);


/*
 * Normal fault status.
 */
led_fault_state_t led_fault_get(uint8_t led);

uint16_t led_fault_voltage(uint8_t led);


/*
 * --------------------------------------------------------------------------
 * STARTUP SELF-TEST
 * --------------------------------------------------------------------------
 *
 * The self-test is non-blocking.
 *
 * Call:
 *
 *     led_fault_selftest_start();
 *
 * once during startup.
 *
 * Then call:
 *
 *     led_fault_update(...)
 *
 * every control tick as usual.
 *
 * While the self-test is running, led_fault_update() internally executes
 * the self-test instead of normal fault monitoring.
 *
 * The application must obtain the requested physical LED outputs using
 * led_fault_selftest_outputs().
 */


/**
 * @brief Start the startup LED self-test.
 */
void led_fault_selftest_start(void);


/**
 * @brief Returns true while the startup self-test is running.
 */
bool led_fault_selftest_running(void);


/**
 * @brief Returns true once the self-test has finished.
 */
bool led_fault_selftest_complete(void);


/**
 * @brief Returns true only when all LEDs passed.
 */
bool led_fault_selftest_passed(void);


/**
 * @brief Returns the self-test result for one LED.
 *
 * @return
 *     LED_SELFTEST_NOT_RUN
 *     LED_SELFTEST_RUNNING
 *     LED_SELFTEST_PASS
 *     LED_SELFTEST_FAIL
 */
led_selftest_state_t led_fault_selftest_get(uint8_t led);


/**
 * @brief Returns the overall self-test state.
 */
led_selftest_state_t led_fault_selftest_state(void);


/**
 * @brief Gets the physical outputs requested by the self-test.
 *
 * During self-test, the caller should drive the four GPIO outputs
 * according to these values.
 *
 * Outside self-test all outputs are returned false.
 */
void led_fault_selftest_outputs(bool *left_on,
                                bool *right_on,
                                bool *low_on,
                                bool *high_on);


#endif /* LED_FAULT_H */