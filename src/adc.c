#include "adc.h"

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"





#define ADC_TIMEOUT_MS       2U

#define ADC_MAX_VALUE        4095U
#define ADC_MAX_PERCENT      100U

/*
 * Hysteresis in percentage points.
 *
 * Example:
 *
 * Current value = 1%
 *
 * Normal boundary for 2%:
 *
 *     2%
 *
 * With +5% hysteresis:
 *
 *     ADC must reach approximately 7% above
 *     the current state before increasing.
 *
 * Similarly, when decreasing, the ADC must
 * go sufficiently below the current boundary.
 */
#define HYSTERESIS_PERCENT   5U


static ADC_HandleTypeDef hadc1;

static bool adc_initialized = false;


/* ========================================================================== */
/* Cached application values                                                  */
/* ========================================================================== */

static uint8_t fuel_value = 0U;
static uint8_t temperature_value = 0U;


/* ========================================================================== */
/* ADC initialization                                                         */
/* ========================================================================== */

void adc_init(void)
{
    GPIO_InitTypeDef gpio = {0};


    /* ---------------------------------------------------------------------- */
    /* Enable clocks                                                          */
    /* ---------------------------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();


    /* ---------------------------------------------------------------------- */
    /*
     * ADC inputs
     *
     * PA0 = ADC_CHANNEL_0
     * PA1 = ADC_CHANNEL_1
     * PA2 = ADC_CHANNEL_2
     * PA3 = ADC_CHANNEL_3
     * PA4 = ADC_CHANNEL_4 -> Fuel
     * PA5 = ADC_CHANNEL_5 -> Temperature
     */
    /* ---------------------------------------------------------------------- */

    gpio.Pin =
        GPIO_PIN_0 |
        GPIO_PIN_1 |
        GPIO_PIN_2 |
        GPIO_PIN_3 |
        GPIO_PIN_4 |
        GPIO_PIN_5;

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &gpio);


    /* ---------------------------------------------------------------------- */
    /* PB1                                                                    */
    /* ---------------------------------------------------------------------- */

    gpio.Pin = GPIO_PIN_1;

    HAL_GPIO_Init(GPIOB, &gpio);


    /* ---------------------------------------------------------------------- */
    /* ADC1 configuration                                                     */
    /* ---------------------------------------------------------------------- */

    hadc1.Instance = ADC1;

    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;

    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;

    hadc1.Init.ScanConvMode =
        DISABLE;

    hadc1.Init.ContinuousConvMode =
        DISABLE;

    hadc1.Init.DiscontinuousConvMode =
        DISABLE;

    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;

    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    hadc1.Init.NbrOfConversion =
        1;

    hadc1.Init.DMAContinuousRequests =
        DISABLE;

    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;


    /* ---------------------------------------------------------------------- */
    /* Initialize ADC                                                         */
    /* ---------------------------------------------------------------------- */

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {

        adc_initialized = false;

        return;
    }


    adc_initialized = true;


    /* ---------------------------------------------------------------------- */
    /* Reset values                                                            */
    /* ---------------------------------------------------------------------- */

    fuel_value = 0U;
    temperature_value = 0U;
}


/* ========================================================================== */
/* Read one ADC channel                                                       */
/* ========================================================================== */

uint16_t adc_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};


    if (!adc_initialized) {
        return 0U;
    }


    /* Configure channel */
    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;


    if (
        HAL_ADC_ConfigChannel(
            &hadc1,
            &cfg
        ) != HAL_OK
    ) {

        return 0U;
    }


    /* Start conversion */
    if (
        HAL_ADC_Start(&hadc1)
        != HAL_OK
    ) {

        return 0U;
    }


    /* Wait for conversion */
    if (
        HAL_ADC_PollForConversion(
            &hadc1,
            ADC_TIMEOUT_MS
        ) != HAL_OK
    ) {

        HAL_ADC_Stop(&hadc1);

        return 0U;
    }


    /* Read result */
    uint16_t value =
        (uint16_t)HAL_ADC_GetValue(&hadc1);


    HAL_ADC_Stop(&hadc1);


    return value;
}


/* ========================================================================== */
/* ADC -> millivolts                                                          */
/* ========================================================================== */

uint16_t adc_read_mv(uint32_t channel)
{
    uint32_t raw;

    raw = adc_read_channel(channel);

    return (uint16_t)(
        (raw * 3300U) /
        ADC_MAX_VALUE
    );
}


/* ========================================================================== */
/* Convert ADC to 0..100                                                      */
/* ========================================================================== */

static uint8_t adc_raw_to_percent(uint16_t raw)
{
    if (raw >= ADC_MAX_VALUE) {
        return 100U;
    }

    return (uint8_t)(
        ((uint32_t)raw * 100U) /
        ADC_MAX_VALUE
    );
}


/* ========================================================================== */
/*
 * Hysteresis filter
 *
 * The important point:
 *
 * The current output is remembered.
 *
 * If the signal moves upward, we require it to cross
 * the next boundary + hysteresis.
 *
 * If the signal moves downward, we require it to cross
 * the current boundary - hysteresis.
 *
 * This prevents:
 *
 *     1 2 1 2 1 2
 *
 * when the ADC is oscillating around a boundary.
 */
/* ========================================================================== */

static uint8_t adc_apply_hysteresis(
    uint16_t raw,
    uint8_t current_percent
)
{
    uint8_t instantaneous;

    instantaneous =
        adc_raw_to_percent(raw);


    /* ---------------------------------------------------------------------- */
    /* Already at maximum                                                     */
    /* ---------------------------------------------------------------------- */

    if (current_percent >= 100U) {

        /*
         * Only leave 100% when the signal drops
         * below the lower hysteresis threshold.
         */

        uint32_t lower =
            (
                100U *
                ADC_MAX_VALUE
            ) /
            ADC_MAX_PERCENT;

        uint32_t hysteresis =
            (
                HYSTERESIS_PERCENT *
                ADC_MAX_VALUE
            ) /
            ADC_MAX_PERCENT;


        if (lower > hysteresis) {
            lower -= hysteresis;
        }
        else {
            lower = 0U;
        }


        if (raw < lower) {
            current_percent = 99U;
        }

        return current_percent;
    }


    /* ---------------------------------------------------------------------- */
    /* Increase                                                               */
    /* ---------------------------------------------------------------------- */

    if (instantaneous > current_percent) {

        /*
         * Next percentage boundary.
         *
         * Example:
         *
         * current = 1
         *
         * next boundary = 2%
         */

        uint32_t next_boundary =
            (
                ((uint32_t)
                 (current_percent + 1U) *
                 ADC_MAX_VALUE)
                /
                ADC_MAX_PERCENT
            );


        /*
         * Add hysteresis.
         */

        uint32_t hysteresis =
            (
                HYSTERESIS_PERCENT *
                ADC_MAX_VALUE
            )
            /
            ADC_MAX_PERCENT;


        uint32_t upper =
            next_boundary +
            hysteresis;


        if (upper > ADC_MAX_VALUE) {
            upper = ADC_MAX_VALUE;
        }


        if (raw >= upper) {

            current_percent++;
        }


        return current_percent;
    }


    /* ---------------------------------------------------------------------- */
    /* Decrease                                                               */
    /* ---------------------------------------------------------------------- */

    if (instantaneous < current_percent) {

        /*
         * Current percentage boundary.
         *
         * Example:
         *
         * current = 2%
         *
         * normal lower boundary = 2%
         */

        uint32_t boundary =
            (
                (uint32_t)
                current_percent *
                ADC_MAX_VALUE
            )
            /
            ADC_MAX_PERCENT;


        uint32_t hysteresis =
            (
                HYSTERESIS_PERCENT *
                ADC_MAX_VALUE
            )
            /
            ADC_MAX_PERCENT;


        uint32_t lower;


        if (boundary > hysteresis) {
            lower = boundary - hysteresis;
        }
        else {
            lower = 0U;
        }


        if (raw < lower) {

            current_percent--;
        }


        return current_percent;
    }


    return current_percent;
}


/* ========================================================================== */
/* Update ADC application values                                              */
/* ========================================================================== */

void adc_update(void)
{
    uint16_t fuel_raw;
    uint16_t temp_raw;


    if (!adc_initialized) {
        return;
    }


    /* ---------------------------------------------------------------------- */
    /* Fuel                                                                     */
    /* ---------------------------------------------------------------------- */

    fuel_raw =
        adc_read_channel(
            ADC_CHANNEL_4
        );


    /* ---------------------------------------------------------------------- */
    /* Temperature                                                              */
    /* ---------------------------------------------------------------------- */

    temp_raw =
        adc_read_channel(
            ADC_CHANNEL_5
        );


    /* ---------------------------------------------------------------------- */
    /* Apply hysteresis                                                        */
    /* ---------------------------------------------------------------------- */

    fuel_value =
        adc_apply_hysteresis(
            fuel_raw,
            fuel_value
        );


    temperature_value =
        adc_apply_hysteresis(
            temp_raw,
            temperature_value
        );
}


/* ========================================================================== */
/* Get fuel                                                                   */
/* ========================================================================== */

uint8_t adc_get_fuel(void)
{
    return fuel_value;
}


/* ========================================================================== */
/* Get temperature                                                            */
/* ========================================================================== */

uint8_t adc_get_temperature(void)
{
    return temperature_value;
}