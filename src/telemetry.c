#include "telemetry.h"
#include "mcp2515.h"
#include "powertrain.h"
#include "shifter.h"
#include "led_fault.h"
#include "stm32f4xx_hal.h"

#define CAN_TELEMETRY_ID  0x0A2U  /* Target CAN Frame ID */
#define CAN_LIGHTS_ID     0x0A3U  /* Lights/indicators toggle-state frame */


/*
 * FIXED: these four values had gotten rotated one slot relative to their
 * lN/rN comments (BTN_HIGH_BEAM was holding BTN_LOW_BEAM's old
 * value, etc.) -- the labels no longer matched what was actually
 * confirmed against the live button-press log, so pressing the low-beam
 * paddle (r2) was toggling high_beam instead. Restored to the values
 * that were actually confirmed.
 */


 
#define BTN_HIGH_BEAM         0x00000400U /* l3 */
#define BTN_LOW_BEAM          0x00000800U/* r2 */
#define BTN_LEFT_INDICATOR    0x00000080U/* l2 */
#define BTN_RIGHT_INDICATOR   0x00000040U/* r3 */
#define BTN_X   0x00000001U/* x */

/* Output bit positions in the CAN_LIGHTS_ID frame's single data byte. */
#define LIGHTS_BIT_HIGH_BEAM       0
#define LIGHTS_BIT_LOW_BEAM        1
#define LIGHTS_BIT_LEFT_INDICATOR  2
#define LIGHTS_BIT_RIGHT_INDICATOR 3

#define INDICATOR_BLINK_PERIOD_MS 500U

/*
 * Physical GPIO outputs that drive the 4 lights. Chosen free of every
 * other pin already claimed in this project: ADC sense (PA0/1/2/3/4, PB1
 * -- see led_fault.c/shifter.c), SPI1 to the MCP2515 (PB3/4/5), its
 * CS/INT (PC13/PB0), USART1 console (PA9/10), USB OTG (PA11/12), and the
 * onboard status LED (PC13). PA5-PA8 are untouched by anything else here.
 */
#define LEFT_LED_PORT   GPIOA
#define LEFT_LED_PIN    GPIO_PIN_5
#define RIGHT_LED_PORT  GPIOA
#define RIGHT_LED_PIN   GPIO_PIN_6
#define LOW_LED_PORT    GPIOA
#define LOW_LED_PIN     GPIO_PIN_7
#define HIGH_LED_PORT   GPIOA
#define HIGH_LED_PIN    GPIO_PIN_8


#define LIGHTS_TX_PERIOD_MS 10U  /* cyclic send interval */
#define TELEMETRY_TX_PERIOD_MS 1U
#define LED_FALUI_TX_PERIOD_MS 10U  

static void send_telemetry_can_msg(uint8_t speed, uint16_t rpm, int8_t gear, char gear_mode, bool hands_on)
{
    static uint8_t msg_counter = 0;
    struct can_frame frame;

    frame.id = CAN_TELEMETRY_ID;
    frame.dlc = 8;

    /* Bytes 0-1: Engine RPM (Little-Endian) */
    frame.data[0] = (uint8_t)(rpm & 0xFFU);
    frame.data[1] = (uint8_t)((rpm >> 8) & 0xFFU);

    /* Byte 2: Vehicle Speed (1 Byte, 0-174 km/h) */
    frame.data[2] = speed;

    /* Byte 3: Gear Mode ('P', 'N', 'D', 'R') */
    frame.data[3] = (uint8_t)gear_mode;

    /* Byte 4: Gear Number (-1 to 6) */
    frame.data[4] = (uint8_t)gear;

    /* Byte 5: Hands-On Flag (0 = OFF, 1 = ON) */
    frame.data[5] = hands_on ? 1U : 0U;

    /* Byte 6: Reserved / Unused */
    frame.data[6] = 0x00;

    /* Byte 7: Rolling Counter (0-255) */
    frame.data[7] = msg_counter++;

    /* Send frame using existing MCP2515 API */
    mcp2515_send(&frame);
}
static void send_lights_can_msg(uint8_t lights_state,
                                uint8_t fault_state,
                                bool x_pressed)
{
    static uint8_t msg_counter = 0;
    struct can_frame frame;

    frame.id = CAN_LIGHTS_ID;
    frame.dlc = 4;

    /* Byte 0: light/indicator ON/OFF state */
    frame.data[0] = lights_state;

    /* Byte 1: LED fault state
     *
     * bits 1:0 = Left Indicator
     * bits 3:2 = Right Indicator
     * bits 5:4 = Low Beam
     * bits 7:6 = High Beam
     */
    frame.data[1] = fault_state;

    /* Byte 2: X button
     * 0 = not pressed
     * 1 = pressed
     *
     * BTN_X used to dismiss notifications.
     */
    frame.data[2] = x_pressed ? 1U : 0U;

    /* Byte 3: rolling counter */
    frame.data[3] = msg_counter++;

    mcp2515_send(&frame);
}
/*
 * Edge-detected press-to-toggle: a light's state only flips on the tick its
 * button bit goes 0 -> 1 in shared_buttons (rising edge), never on release
 * and never for as long as it's held. Without the edge check, holding the
 * button down would toggle it every single call this function is invoked
 * from -- effectively every control_task tick the button stays pressed,
 * which reads as the light flickering rather than switching once per press.
 *
 * *state and *was_pressed are the caller's persistent per-button storage
 * (see the static locals in can_lights_update() below) -- this has no
 * static state of its own so one implementation serves all four buttons.
 */
static void toggle_on_press(uint32_t buttons, uint32_t mask,
                             bool *was_pressed, bool *state)
{
    bool pressed = (buttons & mask) != 0U;

    if (pressed && !*was_pressed) {
        *state = !*state;
    }
    *was_pressed = pressed;
}

/*
 * buttons: shared_buttons, the same raw 32-bit report bitmap main.c already
 *          samples once per control_task tick -- read here rather than
 *          re-touched at the USB layer, for the same split-brain reasons
 *          shifter_sel and hands_on are passed into can_telemetry_update()
 *          rather than re-derived.
 *
 * This is the ONLY place the four toggle_pressed/state statics live now --
 * previously they were static locals inside can_lights_update() itself.
 * Pulling them out here means there is exactly one toggle state machine
 * per light, called once per tick, whose result both the CAN send and
 * led_fault's gating consume -- not two independent state machines reading
 * the same bitmap and hoping to agree.
 */
uint8_t lights_toggle_update(uint32_t buttons, bool *left_on, bool *right_on,
                              bool *low_on, bool *high_on)
{
    static bool high_beam, low_beam, left_indicator, right_indicator;
    static bool high_beam_pressed, low_beam_pressed;
    static bool left_indicator_pressed, right_indicator_pressed;

    toggle_on_press(buttons, BTN_HIGH_BEAM, &high_beam_pressed, &high_beam);
    toggle_on_press(buttons, BTN_LOW_BEAM, &low_beam_pressed, &low_beam);
    toggle_on_press(buttons, BTN_LEFT_INDICATOR, &left_indicator_pressed,
                     &left_indicator);
    toggle_on_press(buttons, BTN_RIGHT_INDICATOR, &right_indicator_pressed,
                     &right_indicator);

    *left_on  = left_indicator;
    *right_on = right_indicator;
    *low_on   = low_beam;
    *high_on  = high_beam;

    return (uint8_t)
        ((high_beam       ? (1U << LIGHTS_BIT_HIGH_BEAM)       : 0U) |
         (low_beam        ? (1U << LIGHTS_BIT_LOW_BEAM)        : 0U) |
         (left_indicator  ? (1U << LIGHTS_BIT_LEFT_INDICATOR)  : 0U) |
         (right_indicator ? (1U << LIGHTS_BIT_RIGHT_INDICATOR) : 0U));
}

/*
 * lights_state: already-packed toggle state from lights_toggle_update(),
 *               computed once per tick by control_task. This function no
 *               longer touches buttons or runs any toggle logic -- it only
 *               decides when to send, exactly as before.
 */
void can_lights_update(uint8_t lights_state,
                       uint8_t fault_state,
                       uint32_t buttons)
{
    static uint32_t last_tx_time = 0;
    static uint8_t prev_lights_state = 0xFFU;
    static uint8_t prev_fault_state = 0xFFU;
    static bool prev_x_pressed = false;

    uint32_t now = HAL_GetTick();
    bool x_pressed = (buttons & BTN_X) != 0U;

    if ((now - last_tx_time >= LIGHTS_TX_PERIOD_MS) ||
        (lights_state != prev_lights_state) ||
        (fault_state != prev_fault_state) ||
        (x_pressed != prev_x_pressed))
    {
        send_lights_can_msg(lights_state, fault_state, x_pressed);

        last_tx_time = now;
        prev_lights_state = lights_state;
        prev_fault_state = fault_state;
        prev_x_pressed = x_pressed;
    }
}
/*
 * Driven low BEFORE the pins are switched to output mode -- same ordering
 * board.c uses for the onboard status LED (HAL_GPIO_WritePin() ahead of
 * HAL_GPIO_Init()) -- so there's no brief glitch-high while the pin is
 * still floating/input during configuration.
 */
void lights_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();   /* idempotent if already enabled elsewhere */

    HAL_GPIO_WritePin(LEFT_LED_PORT, LEFT_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_LED_PORT, RIGHT_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOW_LED_PORT, LOW_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HIGH_LED_PORT, HIGH_LED_PIN, GPIO_PIN_RESET);

    g.Pin   = LEFT_LED_PIN | RIGHT_LED_PIN | LOW_LED_PIN | HIGH_LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;    /* driving an LED, not a bus signal */
    HAL_GPIO_Init(GPIOA, &g);
}

/*
 * Takes the same 4 states led_fault_update() ultimately gates on -- see the
 * comment on this function in telemetry.h for why. Called once per
 * control_task tick, right after lights_toggle_update() produces them.
 */
void lights_gpio_set(bool left_on, bool right_on, bool low_on, bool high_on,
                      bool *left_out, bool *right_out, bool *low_out,
                      bool *high_out)
{
    /*
     * Left/right indicators blink at 1 Hz:
     * 500 ms ON, 500 ms OFF.
     *
     * High beam and low beam remain steady.
     */
    uint32_t now = HAL_GetTick();
    bool blink_phase = ((now / INDICATOR_BLINK_PERIOD_MS) % 2U) == 0U;

    bool left_output  = left_on  && blink_phase;
    bool right_output = right_on && blink_phase;

    HAL_GPIO_WritePin(LEFT_LED_PORT, LEFT_LED_PIN,
                       left_output ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(RIGHT_LED_PORT, RIGHT_LED_PIN,
                       right_output ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LOW_LED_PORT, LOW_LED_PIN,
                       low_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(HIGH_LED_PORT, HIGH_LED_PIN,
                       high_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    *left_out  = left_output;
    *right_out = right_output;
    *low_out   = low_on;
    *high_out  = high_on;
}

/*
 * shifter_sel:  the actual PRND selector reading, owned and sampled once per
 *               tick by main.c (shared_shifter) and passed in here rather
 *               than re-read via shifter_update() -- calling that a second
 *               time per tick would re-poll the ADC and risk a different
 *               reading from the one main.c already used to drive the sim
 *               this tick, which is the kind of split-brain state this whole
 *               project works hard to avoid elsewhere (see the
 *               taskENTER_CRITICAL() state copies in g29_hid.c for the same
 *               reasoning applied to the wheel report).
 *
 * hands_on:     the actual hands-on/off VERDICT from hod_update(), as
 *               already computed once per tick by steer_feel_update() in
 *               main.c and published to shared_hands_on. This used to be
 *               read from steer_feel_get_telemetry()'s p2p field instead --
 *               but p2p is the raw peak-to-peak excursion COUNT from the
 *               last hands-on probe (an arbitrary magnitude like 514 or
 *               771, visible in the console log's p2p= column), not a
 *               boolean. Implicitly converting that nonzero count to bool
 *               meant the CAN Hands byte would latch ON (well, stay
 *               whatever a nonzero p2p implies) the first time any probe
 *               ever produced excursion, and then never track the real
 *               verdict again -- completely disconnected from what
 *               hod_update() actually decided. There is no hands_on field
 *               on struct steer_feel_telemetry to read instead; the verdict
 *               only exists as steer_feel_update()'s return value, so it has
 *               to come in as a parameter the same way shifter_sel does.
 */
void can_telemetry_update(enum shifter_mode shifter_sel, bool hands_on)
{
    static uint32_t last_tx_time = 0;
    static bool prev_hands_on = false;

    uint32_t now = HAL_GetTick();

    /* Read current powertrain state using the existing struct API */
    struct powertrain_state pt;
    powertrain_get_state(&pt);

    /* Clamp speed to 255 max */
    uint8_t  speed    = (pt.speed_kmh > 255U) ? 255U : (uint8_t)pt.speed_kmh;
    uint16_t rpm      = pt.engine_rpm;
    int8_t   gear     = pt.gear;

    /*
     * Gear Mode character. powertrain.c only models D and R -- pt.gear/
     * pt.reverse have no way to say N or P, so deriving gear_mode from
     * pt.gear alone (as this used to) could only ever report 'N' (both
     * neutral AND reverse read pt.gear==0) or 'D', never 'R' or 'P'. That is
     * exactly what showed up on the CAN bus: Mode: N sitting there
     * regardless of what the shifter was actually doing.
     *
     * Take the mode letter from the real shifter selection when it reads N
     * or P, and only fall back to the sim's own D/R distinction when the
     * selector genuinely reads D or R -- that is the one distinction
     * powertrain.c tracks correctly (e.g. the R->D interlock holding 'R'
     * until the car is actually stopped is real state, not a display bug).
     * Mirrors the identical fix already applied to the console printf in
     * main.c.
     */
    char gear_mode = (shifter_sel == SHIFTER_N || shifter_sel == SHIFTER_P)
                    ? shifter_letter(shifter_sel)
                    : (pt.reverse ? 'R' : 'D');

    /* Immediate Event Trigger: Detect state change on Hands-On / Hands-Off */
    bool hod_changed = (hands_on != prev_hands_on);

    /* Transmit every 10 ms OR immediately if HOD status toggles */
    if ((now - last_tx_time >= TELEMETRY_TX_PERIOD_MS) || hod_changed) {
        send_telemetry_can_msg(speed, rpm, gear, gear_mode, hands_on);

        last_tx_time = now;
        prev_hands_on = hands_on;
    }
}
static uint8_t led_fault_to_bits(led_fault_state_t s)
{
    switch (s) {

    default:
    case LED_OK:
        return 0;

    case LED_OPEN:
        return 1;

    case LED_SHORT:
        return 2;
    }
}

uint8_t get_led_fault_state(void)
{
    uint8_t state = 0;

    state |= (led_fault_to_bits(led_fault_get(0)) << 0);
    state |= (led_fault_to_bits(led_fault_get(1)) << 2);
    state |= (led_fault_to_bits(led_fault_get(2)) << 4);
    state |= (led_fault_to_bits(led_fault_get(3)) << 6);

    return state;
}