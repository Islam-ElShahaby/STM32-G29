#include "telemetry.h"

#include "mcp2515.h"
#include "powertrain.h"
#include "shifter.h"
#include "led_fault.h"

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>


/* ========================================================================= */
/* CAN IDs                                                                   */
/* ========================================================================= */

#define CAN_TELEMETRY_ID       0x0A2U
#define CAN_LIGHTS_ID          0x0A3U


/* ========================================================================= */
/* G29 BUTTON MASKS                                                          */
/* ========================================================================= */

#define BTN_HIGH_BEAM          0x00000400U
#define BTN_LOW_BEAM           0x00000800U

#define BTN_LEFT_INDICATOR     0x00000080U
#define BTN_RIGHT_INDICATOR    0x00000040U

#define BTN_X                  0x00000001U


/* ========================================================================= */
/* LIGHT STATE BIT POSITIONS                                                 */
/* ========================================================================= */

/*
 * CAN 0x0A3 Byte 0
 *
 * Bit 0 = High beam
 * Bit 1 = Low beam
 * Bit 2 = Left indicator
 * Bit 3 = Right indicator
 */

#define LIGHTS_BIT_HIGH_BEAM        0U
#define LIGHTS_BIT_LOW_BEAM         1U
#define LIGHTS_BIT_LEFT_INDICATOR   2U
#define LIGHTS_BIT_RIGHT_INDICATOR  3U


/* ========================================================================= */
/* INDICATOR BLINK                                                           */
/* ========================================================================= */

#define INDICATOR_BLINK_PERIOD_MS   500U


/* ========================================================================= */
/* PHYSICAL LED GPIO                                                         */
/* ========================================================================= */

/*
 * IMPORTANT:
 *
 * PA5 is used by the temperature ADC.
 *
 * Therefore the light outputs are:
 *
 * Left indicator  -> PB2
 * Right indicator -> PA6
 * Low beam        -> PA7
 * High beam       -> PA8
 */

#define LEFT_LED_PORT       GPIOB
#define LEFT_LED_PIN        GPIO_PIN_2

#define RIGHT_LED_PORT      GPIOA
#define RIGHT_LED_PIN       GPIO_PIN_6

#define LOW_LED_PORT        GPIOA
#define LOW_LED_PIN         GPIO_PIN_7

#define HIGH_LED_PORT       GPIOA
#define HIGH_LED_PIN        GPIO_PIN_8


/* ========================================================================= */
/* CAN TRANSMISSION PERIOD                                                   */
/* ========================================================================= */

/*
 * main.c calls:
 *
 *     can_telemetry_update()
 *     can_lights_update()
 *
 * from the CAN task every 10 ms.
 *
 * Therefore the maximum actual CAN transmission rate is 10 ms.
 */

#define LIGHTS_TX_PERIOD_MS       10U
#define TELEMETRY_TX_PERIOD_MS    10U


/* ========================================================================= */
/* 0x0A2 POWERTRAIN TELEMETRY                                                */
/* ========================================================================= */

/*
 * CAN ID: 0x0A2
 * DLC:    8
 *
 * Byte 0-1 : Engine RPM, little endian
 * Byte 2   : Vehicle speed
 * Byte 3   : Gear mode
 *            'P' / 'N' / 'D' / 'R'
 * Byte 4   : Gear number
 * Byte 5   : Hands-on flag
 * Byte 6   : Reserved
 * Byte 7   : Rolling counter
 */

static void send_telemetry_can_msg(
    uint8_t speed,
    uint16_t rpm,
    int8_t gear,
    char gear_mode,
    bool hands_on
)
{
    static uint8_t msg_counter = 0U;

    struct can_frame frame = {0};


    frame.id = CAN_TELEMETRY_ID;
    frame.dlc = 8U;


    /* --------------------------------------------------------------------- */
    /* Byte 0-1: RPM                                                         */
    /* --------------------------------------------------------------------- */

    frame.data[0] =
        (uint8_t)(rpm & 0xFFU);

    frame.data[1] =
        (uint8_t)((rpm >> 8) & 0xFFU);


    /* --------------------------------------------------------------------- */
    /* Byte 2: Speed                                                         */
    /* --------------------------------------------------------------------- */

    frame.data[2] = speed;


    /* --------------------------------------------------------------------- */
    /* Byte 3: Gear mode                                                     */
    /* --------------------------------------------------------------------- */

    frame.data[3] =
        (uint8_t)gear_mode;


    /* --------------------------------------------------------------------- */
    /* Byte 4: Gear number                                                   */
    /* --------------------------------------------------------------------- */

    frame.data[4] =
        (uint8_t)gear;


    /* --------------------------------------------------------------------- */
    /* Byte 5: Hands-on                                                      */
    /* --------------------------------------------------------------------- */

    frame.data[5] =
        hands_on ? 1U : 0U;


    /* --------------------------------------------------------------------- */
    /* Byte 6: Reserved                                                      */
    /* --------------------------------------------------------------------- */

    frame.data[6] = 0x00U;


    /* --------------------------------------------------------------------- */
    /* Byte 7: Rolling counter                                               */
    /* --------------------------------------------------------------------- */

    frame.data[7] =
        msg_counter++;


    /* --------------------------------------------------------------------- */
    /* Send                                                                  */
    /* --------------------------------------------------------------------- */

    (void)mcp2515_send(&frame);
}


/* ========================================================================= */
/* 0x0A3 LIGHTS / FAULT / FUEL / TEMPERATURE                                */
/* ========================================================================= */

/*
 * CAN ID: 0x0A3
 * DLC:    6
 *
 * Byte 0:
 *
 *     Bit 0 = High beam
 *     Bit 1 = Low beam
 *     Bit 2 = Left indicator
 *     Bit 3 = Right indicator
 *
 *
 * Byte 1:
 *
 *     Bits 1:0 = Left indicator fault
 *     Bits 3:2 = Right indicator fault
 *     Bits 5:4 = Low beam fault
 *     Bits 7:6 = High beam fault
 *
 *
 * Fault encoding:
 *
 *     00 = OK
 *     01 = OPEN
 *     10 = SHORT
 *
 *
 * Byte 2 = X button
 * Byte 3 = Fuel       0..100 %
 * Byte 4 = Temperature 0..100 %
 * Byte 5 = Rolling counter
 */

static void send_lights_can_msg(
    uint8_t lights_state,
    uint8_t fault_state,
    bool x_pressed,
    uint8_t fuel,
    uint8_t temperature
)
{
    static uint8_t msg_counter = 0U;

    struct can_frame frame = {0};


    frame.id = CAN_LIGHTS_ID;
    frame.dlc = 6U;


    /* --------------------------------------------------------------------- */
    /* Byte 0: Light states                                                  */
    /* --------------------------------------------------------------------- */

    frame.data[0] =
        lights_state;


    /* --------------------------------------------------------------------- */
    /* Byte 1: LED fault states                                              */
    /* --------------------------------------------------------------------- */

    frame.data[1] =
        fault_state;


    /* --------------------------------------------------------------------- */
    /* Byte 2: X button                                                      */
    /* --------------------------------------------------------------------- */

    frame.data[2] =
        x_pressed ? 1U : 0U;


    /* --------------------------------------------------------------------- */
    /* Byte 3: Fuel 0..100                                                   */
    /* --------------------------------------------------------------------- */

    frame.data[3] =
        (fuel > 100U) ? 100U : fuel;


    /* --------------------------------------------------------------------- */
    /* Byte 4: Temperature 0..100                                            */
    /* --------------------------------------------------------------------- */

    frame.data[4] =
        (temperature > 100U) ? 100U : temperature;


    /* --------------------------------------------------------------------- */
    /* Byte 5: Rolling counter                                               */
    /* --------------------------------------------------------------------- */

    frame.data[5] =
        msg_counter++;


    /* --------------------------------------------------------------------- */
    /* Send                                                                  */
    /* --------------------------------------------------------------------- */

    (void)mcp2515_send(&frame);
}


/* ========================================================================= */
/* BUTTON EDGE DETECTION                                                     */
/* ========================================================================= */

static void toggle_on_press(
    uint32_t buttons,
    uint32_t mask,
    bool *was_pressed,
    bool *state
)
{
    bool pressed =
        ((buttons & mask) != 0U);


    /*
     * Toggle only on the rising edge.
     *
     * This means holding a G29 button does not repeatedly toggle
     * the light.
     */

    if (
        pressed &&
        !(*was_pressed)
    ) {

        *state =
            !(*state);
    }


    *was_pressed =
        pressed;
}


/* ========================================================================= */
/* LIGHT TOGGLE STATE                                                        */
/* ========================================================================= */

uint8_t lights_toggle_update(
    uint32_t buttons,
    bool *left_on,
    bool *right_on,
    bool *low_on,
    bool *high_on
)
{
    /*
     * Persistent logical states.
     */

    static bool high_beam = false;
    static bool low_beam = false;

    static bool left_indicator = false;
    static bool right_indicator = false;


    /*
     * Previous button states.
     */

    static bool high_beam_pressed = false;
    static bool low_beam_pressed = false;

    static bool left_indicator_pressed = false;
    static bool right_indicator_pressed = false;


    /* --------------------------------------------------------------------- */
    /* High beam                                                             */
    /* --------------------------------------------------------------------- */

    toggle_on_press(
        buttons,
        BTN_HIGH_BEAM,
        &high_beam_pressed,
        &high_beam
    );


    /* --------------------------------------------------------------------- */
    /* Low beam                                                              */
    /* --------------------------------------------------------------------- */

    toggle_on_press(
        buttons,
        BTN_LOW_BEAM,
        &low_beam_pressed,
        &low_beam
    );


    /* --------------------------------------------------------------------- */
    /* Left indicator                                                        */
    /* --------------------------------------------------------------------- */

    toggle_on_press(
        buttons,
        BTN_LEFT_INDICATOR,
        &left_indicator_pressed,
        &left_indicator
    );


    /* --------------------------------------------------------------------- */
    /* Right indicator                                                       */
    /* --------------------------------------------------------------------- */

    toggle_on_press(
        buttons,
        BTN_RIGHT_INDICATOR,
        &right_indicator_pressed,
        &right_indicator
    );


    /* --------------------------------------------------------------------- */
    /* Return logical states                                                 */
    /* --------------------------------------------------------------------- */

    if (left_on != NULL) {
        *left_on = left_indicator;
    }

    if (right_on != NULL) {
        *right_on = right_indicator;
    }

    if (low_on != NULL) {
        *low_on = low_beam;
    }

    if (high_on != NULL) {
        *high_on = high_beam;
    }


    /* --------------------------------------------------------------------- */
    /* Pack CAN Byte 0                                                       */
    /* --------------------------------------------------------------------- */

    return (uint8_t)(
        (high_beam ?
            (1U << LIGHTS_BIT_HIGH_BEAM) : 0U) |

        (low_beam ?
            (1U << LIGHTS_BIT_LOW_BEAM) : 0U) |

        (left_indicator ?
            (1U << LIGHTS_BIT_LEFT_INDICATOR) : 0U) |

        (right_indicator ?
            (1U << LIGHTS_BIT_RIGHT_INDICATOR) : 0U)
    );
}


/* ========================================================================= */
/* CAN LIGHTS UPDATE                                                         */
/* ========================================================================= */

void can_lights_update(
    uint8_t lights_state,
    uint8_t fault_state,
    uint32_t buttons,
    uint8_t fuel,
    uint8_t temperature
)
{
    static uint32_t last_tx_time = 0U;

    static uint8_t prev_lights_state = 0xFFU;
    static uint8_t prev_fault_state = 0xFFU;

    static uint8_t prev_fuel = 0xFFU;
    static uint8_t prev_temperature = 0xFFU;

    static bool prev_x_pressed = false;


    uint32_t now =
        HAL_GetTick();


    bool x_pressed =
        ((buttons & BTN_X) != 0U);


    /*
     * Clamp values.
     */

    if (fuel > 100U) {
        fuel = 100U;
    }

    if (temperature > 100U) {
        temperature = 100U;
    }


    /*
     * Transmit:
     *
     * 1. Every 10 ms
     *
     * OR
     *
     * 2. Light state changed
     *
     * OR
     *
     * 3. Fault state changed
     *
     * OR
     *
     * 4. X changed
     *
     * OR
     *
     * 5. Fuel changed
     *
     * OR
     *
     * 6. Temperature changed
     */

    if (
        (now - last_tx_time >= LIGHTS_TX_PERIOD_MS) ||

        (lights_state != prev_lights_state) ||

        (fault_state != prev_fault_state) ||

        (x_pressed != prev_x_pressed) ||

        (fuel != prev_fuel) ||

        (temperature != prev_temperature)
    ) {

        send_lights_can_msg(
            lights_state,
            fault_state,
            x_pressed,
            fuel,
            temperature
        );


        last_tx_time =
            now;


        prev_lights_state =
            lights_state;

        prev_fault_state =
            fault_state;

        prev_x_pressed =
            x_pressed;

        prev_fuel =
            fuel;

        prev_temperature =
            temperature;
    }
}


/* ========================================================================= */
/* LIGHT GPIO INITIALIZATION                                                 */
/* ========================================================================= */

void lights_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};


    /* --------------------------------------------------------------------- */
    /* Enable GPIO clocks                                                    */
    /* --------------------------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* --------------------------------------------------------------------- */
    /* Initial outputs LOW                                                   */
    /* --------------------------------------------------------------------- */

    HAL_GPIO_WritePin(
        LEFT_LED_PORT,
        LEFT_LED_PIN,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOA,
        RIGHT_LED_PIN |
        LOW_LED_PIN |
        HIGH_LED_PIN,
        GPIO_PIN_RESET
    );


    /* --------------------------------------------------------------------- */
    /* PB2 - Left indicator                                                  */
    /* --------------------------------------------------------------------- */

    g.Pin =
        LEFT_LED_PIN;

    g.Mode =
        GPIO_MODE_OUTPUT_PP;

    g.Pull =
        GPIO_NOPULL;

    g.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        LEFT_LED_PORT,
        &g
    );


    /* --------------------------------------------------------------------- */
    /* PA6 - Right indicator                                                 */
    /* PA7 - Low beam                                                        */
    /* PA8 - High beam                                                       */
    /* --------------------------------------------------------------------- */

    g.Pin =
        RIGHT_LED_PIN |
        LOW_LED_PIN |
        HIGH_LED_PIN;


    HAL_GPIO_Init(
        GPIOA,
        &g
    );
}


/* ========================================================================= */
/* LIGHT GPIO OUTPUT                                                         */
/* ========================================================================= */

void lights_gpio_set(
    bool left_on,
    bool right_on,
    bool low_on,
    bool high_on,
    bool *left_out,
    bool *right_out,
    bool *low_out,
    bool *high_out
)
{
    uint32_t now =
        HAL_GetTick();


    /*
     * Indicator blink.
     *
     * 0..499 ms   = ON
     * 500..999 ms = OFF
     */

    bool blink_phase =
        ((now / INDICATOR_BLINK_PERIOD_MS) % 2U) == 0U;


    bool left_output =
        left_on && blink_phase;

    bool right_output =
        right_on && blink_phase;


    /* --------------------------------------------------------------------- */
    /* Left indicator -> PB2                                                 */
    /* --------------------------------------------------------------------- */

    HAL_GPIO_WritePin(
        LEFT_LED_PORT,
        LEFT_LED_PIN,
        left_output ?
            GPIO_PIN_SET :
            GPIO_PIN_RESET
    );


    /* --------------------------------------------------------------------- */
    /* Right indicator -> PA6                                                */
    /* --------------------------------------------------------------------- */

    HAL_GPIO_WritePin(
        RIGHT_LED_PORT,
        RIGHT_LED_PIN,
        right_output ?
            GPIO_PIN_SET :
            GPIO_PIN_RESET
    );


    /* --------------------------------------------------------------------- */
    /* Low beam -> PA7                                                       */
    /* --------------------------------------------------------------------- */

    HAL_GPIO_WritePin(
        LOW_LED_PORT,
        LOW_LED_PIN,
        low_on ?
            GPIO_PIN_SET :
            GPIO_PIN_RESET
    );


    /* --------------------------------------------------------------------- */
    /* High beam -> PA8                                                      */
    /* --------------------------------------------------------------------- */

    HAL_GPIO_WritePin(
        HIGH_LED_PORT,
        HIGH_LED_PIN,
        high_on ?
            GPIO_PIN_SET :
            GPIO_PIN_RESET
    );


    /* --------------------------------------------------------------------- */
    /* Return actual physical states                                         */
    /* --------------------------------------------------------------------- */

    if (left_out != NULL) {
        *left_out =
            left_output;
    }

    if (right_out != NULL) {
        *right_out =
            right_output;
    }

    if (low_out != NULL) {
        *low_out =
            low_on;
    }

    if (high_out != NULL) {
        *high_out =
            high_on;
    }
}


/* ========================================================================= */
/* POWERTRAIN TELEMETRY                                                      */
/* ========================================================================= */

void can_telemetry_update(
    enum shifter_mode shifter_sel,
    bool hands_on
)
{
    static uint32_t last_tx_time = 0U;

    static bool prev_hands_on = false;


    uint32_t now =
        HAL_GetTick();


    struct powertrain_state pt;


    /* --------------------------------------------------------------------- */
    /* Get current powertrain state                                          */
    /* --------------------------------------------------------------------- */

    powertrain_get_state(
        &pt
    );


    /* --------------------------------------------------------------------- */
    /* Speed                                                                  */
    /* --------------------------------------------------------------------- */

    uint8_t speed =
        (pt.speed_kmh > 255U)
        ? 255U
        : (uint8_t)pt.speed_kmh;


    /* --------------------------------------------------------------------- */
    /* RPM                                                                    */
    /* --------------------------------------------------------------------- */

    uint16_t rpm =
        pt.engine_rpm;


    /* --------------------------------------------------------------------- */
    /* Gear number                                                            */
    /* --------------------------------------------------------------------- */

    int8_t gear =
        pt.gear;


    /* --------------------------------------------------------------------- */
    /* Gear mode                                                              */
    /* --------------------------------------------------------------------- */

    char gear_mode;


    if (
        shifter_sel == SHIFTER_N ||
        shifter_sel == SHIFTER_P
    ) {

        gear_mode =
            shifter_letter(
                shifter_sel
            );
    }

    else {

        gear_mode =
            pt.reverse
            ? 'R'
            : 'D';
    }


    /* --------------------------------------------------------------------- */
    /* Hands-on change                                                       */
    /* --------------------------------------------------------------------- */

    bool hod_changed =
        (hands_on != prev_hands_on);


    /* --------------------------------------------------------------------- */
    /* Periodic / event transmission                                         */
    /* --------------------------------------------------------------------- */

    if (
        (now - last_tx_time >=
         TELEMETRY_TX_PERIOD_MS)
        ||
        hod_changed
    ) {

        send_telemetry_can_msg(
            speed,
            rpm,
            gear,
            gear_mode,
            hands_on
        );


        last_tx_time =
            now;

        prev_hands_on =
            hands_on;
    }
}


/* ========================================================================= */
/* LED FAULT -> CAN ENCODING                                                 */
/* ========================================================================= */

/*
 * Encoding:
 *
 *     LED_OK    = 00
 *     LED_OPEN  = 01
 *     LED_SHORT = 10
 */

static uint8_t led_fault_to_bits(
    led_fault_state_t state
)
{
    switch (state) {

    case LED_OPEN:
        return 1U;

    case LED_SHORT:
        return 2U;

    case LED_OK:
    default:
        return 0U;
    }
}


/* ========================================================================= */
/* GET COMBINED LED FAULT STATE                                              */
/* ========================================================================= */

/*
 * Byte layout:
 *
 * Bits 1:0 = LED 0 = Left indicator
 * Bits 3:2 = LED 1 = Right indicator
 * Bits 5:4 = LED 2 = Low beam
 * Bits 7:6 = LED 3 = High beam
 */

uint8_t get_led_fault_state(void)
{
    uint8_t state =
        0U;


    /* --------------------------------------------------------------------- */
    /* LED 0 - Left indicator                                                */
    /* --------------------------------------------------------------------- */

    state |=
        (uint8_t)(
            led_fault_to_bits(
                led_fault_get(0)
            )
            << 0
        );


    /* --------------------------------------------------------------------- */
    /* LED 1 - Right indicator                                               */
    /* --------------------------------------------------------------------- */

    state |=
        (uint8_t)(
            led_fault_to_bits(
                led_fault_get(1)
            )
            << 2
        );


    /* --------------------------------------------------------------------- */
    /* LED 2 - Low beam                                                      */
    /* --------------------------------------------------------------------- */

    state |=
        (uint8_t)(
            led_fault_to_bits(
                led_fault_get(2)
            )
            << 4
        );


    /* --------------------------------------------------------------------- */
    /* LED 3 - High beam                                                     */
    /* --------------------------------------------------------------------- */

    state |=
        (uint8_t)(
            led_fault_to_bits(
                led_fault_get(3)
            )
            << 6
        );


    return state;
}