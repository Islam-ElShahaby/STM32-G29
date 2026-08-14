/**
 * G29 force-feedback over native USB OTG-FS host on STM32 BlackPill.
 *
 * Application layer:
 *
 *   USB G29
 *      |
 *      v
 *   control_task
 *      |
 *      +---- pedals ---> powertrain
 *      |
 *      +---- steering ---> steer_feel ---> G29 FFB
 *      |
 *      +---- lights ---> GPIO + LED fault detection
 *      |
 *      +---- telemetry data ---> CAN
 *
 * CAN:
 *
 *   0x0A2 = powertrain telemetry
 *   0x0A3 = lights + LED faults + X + fuel + temperature
 *   0x0B0 = optional remote FFB command
 */

#include "stm32f4xx_hal.h"

#include "board.h"
#include "log.h"
#include "steer_feel.h"
#include "usb_host.h"
#include "g29_hid.h"
#include "mcp2515.h"
#include "powertrain.h"
#include "shifter.h"
#include "telemetry.h"

#include "adc.h"
#include "led_fault.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"


/* =========================================================================
 * CAN IDs
 * =========================================================================
 */

#define CAN_ID_POWERTRAIN  0x0A2U
#define CAN_ID_LIGHTS      0x0A3U
#define CAN_ID_FFB         0x0B0U


/* =========================================================================
 * FFB configuration
 * =========================================================================
 */

#define FFB_SELFTEST 0

#if FFB_SELFTEST
#define INIT_LAST 5
#else
#define INIT_LAST 2
#endif

/*
 * steer_feel.c owns the normal torque channel.
 *
 * Keep remote FFB disabled unless specifically required.
 */
#define REMOTE_FFB 0


/* =========================================================================
 * FreeRTOS priorities
 * =========================================================================
 */

#define PRIO_CONTROL   4
#define PRIO_USB       3
#define PRIO_CAN       2
#define PRIO_CONSOLE   1


/* =========================================================================
 * Task stack sizes
 * =========================================================================
 */

#define STACK_CONTROL  512
#define STACK_USB      512
#define STACK_CAN      256
#define STACK_CONSOLE  512


/* =========================================================================
 * Wheel setup
 * =========================================================================
 */

#define SETUP_SETTLE_MS       600U
#define SETUP_ACK_TIMEOUT_MS  500U

#define SETUP_REASSERTS       4
#define SETUP_REASSERT_MS     2000U


/* =========================================================================
 * Powertrain
 * =========================================================================
 */

#define PARK_PAWL_KMH 5U


/* =========================================================================
 * FreeRTOS task handles
 * =========================================================================
 */

static TaskHandle_t h_control;
static TaskHandle_t h_usb;
static TaskHandle_t h_can;
static TaskHandle_t h_console;


/* =========================================================================
 * Shared state
 * =========================================================================
 */

static volatile bool shared_ready;
static volatile bool shared_hands_on;

static volatile uint16_t shared_steering;

static volatile uint8_t shared_throttle;
static volatile uint8_t shared_brake;
static volatile uint8_t shared_clutch;

static volatile uint32_t shared_buttons;

static volatile uint8_t shared_lights_state;


/* =========================================================================
 * Powertrain shared state
 * =========================================================================
 */

static volatile uint16_t shared_pt_rpm;
static volatile uint16_t shared_pt_speed;

static volatile uint8_t shared_pt_gear;

static volatile bool shared_pt_reverse;

static volatile uint8_t shared_shifter;


/* =========================================================================
 * Console state
 * =========================================================================
 */

static bool log_quiet;

static volatile bool pt_reverse_request;

static int32_t steer_range = 900;

static volatile bool setup_redo_request;


/* =========================================================================
 * Console RX ring
 * =========================================================================
 */

#define RX_BUF_SZ 64U

static volatile char rx_buf[RX_BUF_SZ];

static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;


/* =========================================================================
 * Console command
 * =========================================================================
 */

static void console_exec(char *line)
{
    long v = 0;
    char c = line[0];


    /*
     * Steering-feel commands.
     */
    if (c && strchr("tvhpiraenymocbCuxRg", c) != NULL) {

        if (sscanf(line + 1, "%ld", &v) != 1) {

            printf(
                "? need a number\r\n"
            );

            return;
        }


        if (steer_feel_console(c, v)) {

            printf("ok ");

            steer_feel_print_values();

            return;
        }


        switch (c) {

        case 'R':

            if (v < 40) {
                steer_range = 40;
            }
            else if (v > 900) {
                steer_range = 900;
            }
            else {
                steer_range = (int32_t)v;
            }


            setup_redo_request = true;


            printf(
                "ok range=%ld deg requested, "
                "re-running setup...\r\n",
                (long)steer_range
            );

            return;


        case 'g':

            pt_reverse_request =
                (v != 0);


            printf(
                "ok gear=%s "
                "(engages once stopped)\r\n",
                pt_reverse_request
                    ? "R"
                    : "D"
            );

            return;


        default:

            printf(
                "? unhandled '%c'\r\n",
                c
            );

            return;
        }
    }


    /* =====================================================================
     * Raw FFB report
     * =====================================================================
     */

    switch (c) {

    case 'f':
    {
        unsigned b[7] = {0};

        int n =
            sscanf(
                line + 1,
                "%x %x %x %x %x %x %x",
                &b[0],
                &b[1],
                &b[2],
                &b[3],
                &b[4],
                &b[5],
                &b[6]
            );


        uint8_t cmd[7];


        if (n < 1) {

            printf(
                "? need hex bytes\r\n"
            );

            return;
        }


        for (int i = 0; i < 7; i++) {

            cmd[i] =
                (uint8_t)b[i];
        }


        printf(
            "raw -> %02X %02X %02X %02X "
            "%02X %02X %02X (%d given)\r\n",

            cmd[0],
            cmd[1],
            cmd[2],
            cmd[3],
            cmd[4],
            cmd[5],
            cmd[6],
            n
        );


        g29_send_raw(cmd);

        break;
    }


    /* =====================================================================
     * Task stack information
     * =====================================================================
     */

    case 'k':
    {
        struct {
            const char *n;
            TaskHandle_t h;
            int size;
        } t[] = {

            {
                "control",
                h_control,
                STACK_CONTROL
            },

            {
                "usb",
                h_usb,
                STACK_USB
            },

            {
                "can",
                h_can,
                STACK_CAN
            },

            {
                "console",
                h_console,
                STACK_CONSOLE
            }
        };


        printf(
            "task      stack  free(words)\r\n"
        );


        for (
            unsigned i = 0;
            i < sizeof(t) / sizeof(t[0]);
            i++
        ) {

            printf(
                "  %-8s %5d  %5u\r\n",

                t[i].n,

                t[i].size,

                (unsigned)
                uxTaskGetStackHighWaterMark(
                    t[i].h
                )
            );
        }


        printf(
            "heap free %u of %u bytes, "
            "log dropped %lu\r\n",

            (unsigned)
            xPortGetFreeHeapSize(),

            (unsigned)
            configTOTAL_HEAP_SIZE,

            (unsigned long)
            log_dropped
        );

        break;
    }


    /* =====================================================================
     * Stop FFB
     * =====================================================================
     */

    case 's':

        steer_feel_stop();

        printf(
            "stopped\r\n"
        );

        break;


    /* =====================================================================
     * System identification
     * =====================================================================
     */

    case 'S':

        log_quiet = true;

        steer_feel_sysid_start();

        printf(
            "sid,begin\r\n"
        );

        break;


    /* =====================================================================
     * Toggle log
     * =====================================================================
     */

    case 'q':

        log_quiet =
            !log_quiet;


        printf(
            "log %s\r\n",
            log_quiet
                ? "off"
                : "on"
        );

        break;


    /* =====================================================================
     * Help
     * =====================================================================
     */

    case '?':

        steer_feel_print_values();

        steer_feel_print_help();

        printf(
            "  R=lock-to-lock range in deg, 40..900\r\n"
            "  g <0|1>=powertrain direction D/R\r\n"
            "  S=step-id\r\n"
            "  k=task stacks\r\n"
            "  f <hex..>=raw FFB\r\n"
            "  s=stop FFB\r\n"
            "  q=toggle log\r\n"
            "  ?=help\r\n"
        );

        break;


    default:

        printf(
            "? unknown '%c'\r\n",
            c
        );

        break;
    }
}


/* =========================================================================
 * USART1 interrupt
 * =========================================================================
 */

void USART1_IRQHandler(void)
{
    USART_TypeDef *u =
        huart1.Instance;


    uint32_t sr =
        u->SR;


    if (
        sr &
        (USART_SR_RXNE | USART_SR_ORE)
    ) {

        char ch =
            (char)
            (u->DR & 0xFFU);


        uint8_t next =
            (uint8_t)
            (
                (rx_head + 1U) &
                (RX_BUF_SZ - 1U)
            );


        if (next != rx_tail) {

            rx_buf[rx_head] =
                ch;

            rx_head =
                next;
        }
    }
}


/* =========================================================================
 * Console RX processing
 * =========================================================================
 */

static void console_poll(void)
{
    static char buf[48];

    static uint8_t len;


    while (rx_tail != rx_head) {

        char ch =
            rx_buf[rx_tail];


        rx_tail =
            (uint8_t)
            (
                (rx_tail + 1U) &
                (RX_BUF_SZ - 1U)
            );


        if (
            ch == '\r' ||
            ch == '\n'
        ) {

            if (len) {

                buf[len] =
                    '\0';

                len = 0;

                console_exec(buf);
            }
        }

        else if (
            len <
            sizeof(buf) - 1
        ) {

            buf[len++] =
                ch;
        }
    }
}


/* =========================================================================
 * Remote FFB command
 * =========================================================================
 */

static void apply_ffb(
    const struct can_frame *f
)
{
    if (f->dlc < 1U) {
        return;
    }


    switch (f->data[0]) {

    case 0x00:

        g29_send_no_effect();

        break;


    case 0x01:

        if (f->dlc >= 3U) {

            int16_t force =
                (int16_t)
                (
                    (uint16_t)
                    f->data[1]
                    |
                    (
                        (uint16_t)
                        f->data[2]
                        << 8
                    )
                );


            g29_send_constant_force(
                force
            );
        }

        break;


    case 0x02:

        if (f->dlc >= 3U) {

            g29_send_autocenter(
                f->data[1],
                f->data[2]
            );
        }

        break;


    case 0x03:

        if (f->dlc >= 3U) {

            uint16_t range =
                (uint16_t)
                f->data[1]
                |
                (
                    (uint16_t)
                    f->data[2]
                    << 8
                );


            g29_send_range(
                range
            );
        }

        break;


    default:

        break;
    }
}


/* =========================================================================
 * USB TASK
 * =========================================================================
 */

static void usb_task(void *arg)
{
    (void)arg;


    for (;;) {

        MX_USB_HOST_Process();


        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }
}


/* =========================================================================
 * CONTROL TASK
 * =========================================================================
 */

static void control_task(void *arg)
{
    TickType_t next =
        xTaskGetTickCount();


    uint32_t last_blink = 0;

    uint32_t t_init = 0;

    uint32_t init_hold =
        SETUP_SETTLE_MS;


    uint8_t init_step = 0;

    uint8_t pt_div = 0;


    uint32_t setup_mark = 0;

    bool setup_waiting = false;

    bool setup_announced = false;


    uint8_t setup_reasserts =
        SETUP_REASSERTS;


    uint32_t t_reassert = 0;


    (void)arg;


    powertrain_init();


    for (;;) {

        struct g29_state st =
            {0};


        bool ready =
            g29_get_state(&st) == 0;


        bool setup_done;

        bool hands_on = false;


        uint32_t ffb_ok;
        uint32_t ffb_nak;
        uint32_t ffb_err;


        g29_ffb_stats(
            &ffb_ok,
            &ffb_nak,
            &ffb_err
        );


        /* =================================================================
         * Wheel connected
         * =================================================================
         */

        if (ready) {

            HAL_GPIO_WritePin(
                GPIOC,
                GPIO_PIN_13,
                GPIO_PIN_RESET
            );


            /* -------------------------------------------------------------
             * Re-run setup request
             * -------------------------------------------------------------
             */

            if (setup_redo_request) {

                setup_redo_request =
                    false;


                init_step = 0;

                init_hold = 0;

                setup_waiting = false;

                setup_announced = false;
            }


            /* -------------------------------------------------------------
             * Wheel setup state machine
             * -------------------------------------------------------------
             */

            if (
                init_step <=
                INIT_LAST
            ) {

                if (setup_waiting) {

                    if (
                        ffb_ok !=
                        setup_mark
                    ) {

                        setup_waiting =
                            false;


                        t_init =
                            HAL_GetTick();


                        init_step++;
                    }

                    else if (
                        HAL_GetTick() -
                        t_init >=
                        SETUP_ACK_TIMEOUT_MS
                    ) {

                        printf(
                            "setup step %u "
                            "NOT acked in %u ms\r\n",

                            init_step,

                            SETUP_ACK_TIMEOUT_MS
                        );


                        setup_waiting =
                            false;


                        t_init =
                            HAL_GetTick();


                        init_step++;
                    }
                }

                else if (
                    HAL_GetTick() -
                    t_init >=
                    init_hold
                ) {

                    bool queued = true;


                    switch (init_step) {

                    case 0:

                        /*
                         * Disable wheel autocenter.
                         */

                        g29_send_autocenter(
                            0,
                            0
                        );

                        break;


                    case 1:

                        /*
                         * Set requested wheel range.
                         */

                        g29_send_range(
                            (uint16_t)
                            steer_range
                        );

                        break;


#if FFB_SELFTEST

                    case 2:

                        printf(
                            "FFB self-test: "
                            "hard LEFT\r\n"
                        );


                        g29_send_constant_force(
                            -32767
                        );

                        break;


                    case 3:

                        printf(
                            "FFB self-test: "
                            "hard RIGHT\r\n"
                        );


                        g29_send_constant_force(
                            32767
                        );

                        break;


                    case 4:

                        printf(
                            "FFB self-test: "
                            "done\r\n"
                        );


                        g29_send_constant_force(
                            0
                        );

                        break;

#endif


                    default:

                        queued = false;

                        break;
                    }


#if FFB_SELFTEST

                    init_hold =
                        (
                            init_step == 2U ||
                            init_step == 3U
                        )
                        ? 700U
                        : 50U;

#else

                    init_hold =
                        50U;

#endif


                    t_init =
                        HAL_GetTick();


                    if (queued) {

                        setup_mark =
                            ffb_ok;


                        setup_waiting =
                            true;
                    }

                    else {

                        init_step++;
                    }
                }
            }
        }


        /* =================================================================
         * Wheel disconnected
         * =================================================================
         */

        else {

            init_step = 0;

            init_hold =
                SETUP_SETTLE_MS;

            setup_waiting =
                false;


            setup_reasserts =
                SETUP_REASSERTS;


            t_reassert =
                HAL_GetTick();


            t_init =
                HAL_GetTick();


            if (
                HAL_GetTick() -
                last_blink >=
                200U
            ) {

                HAL_GPIO_TogglePin(
                    GPIOC,
                    GPIO_PIN_13
                );


                last_blink =
                    HAL_GetTick();
            }
        }


        setup_done =
            init_step >
            INIT_LAST;


        /* =================================================================
         * Reassert wheel setup
         * =================================================================
         */

        if (
            ready &&
            setup_done &&
            setup_reasserts > 0 &&
            HAL_GetTick() -
            t_reassert >=
            SETUP_REASSERT_MS
        ) {

            printf(
                "re-asserting range=%ld "
                "(%u left)\r\n",

                (long)steer_range,

                setup_reasserts - 1U
            );


            setup_reasserts--;

            t_reassert =
                HAL_GetTick();


            init_step = 0;

            init_hold = 0;

            setup_waiting = false;
        }


        /* =================================================================
         * Setup announcement
         * =================================================================
         */

        if (
            ready &&
            setup_done &&
            !setup_announced
        ) {

            setup_announced =
                true;


            printf(
                "wheel setup done: range=%ld deg "
                "delivered to wheel\r\n",

                (long)steer_range
            );
        }


        if (!ready) {

            setup_announced =
                false;
        }


        /* =================================================================
         * Steering / FFB
         * =================================================================
         */

        if (
            ready &&
            setup_done
        ) {

            hands_on =
                steer_feel_update(
                    st.steering
                );
        }


        /* =================================================================
         * Publish G29 state
         * =================================================================
         */

        shared_steering =
            st.steering;


        shared_throttle =
            st.throttle;


        shared_brake =
            st.brake;


        shared_clutch =
            st.clutch;


        shared_buttons =
            st.buttons;


        shared_hands_on =
            hands_on;


        shared_ready =
            ready;


        /* =================================================================
         * LIGHTS
         * =================================================================
         */

        {
            bool left_on = false;
            bool right_on = false;
            bool low_on = false;
            bool high_on = false;


            bool left_out = false;
            bool right_out = false;
            bool low_out = false;
            bool high_out = false;


            /* -------------------------------------------------------------
             * LED startup self-test
             * -------------------------------------------------------------
             */

            if (
                led_fault_selftest_running()
            ) {

                led_fault_update(
                    false,
                    false,
                    false,
                    false
                );


                led_fault_selftest_outputs(
                    &left_out,
                    &right_out,
                    &low_out,
                    &high_out
                );


                lights_gpio_set(
                    left_out,
                    right_out,
                    low_out,
                    high_out,

                    NULL,
                    NULL,
                    NULL,
                    NULL
                );
            }


            /* -------------------------------------------------------------
             * Normal light operation
             * -------------------------------------------------------------
             */

            else {

                shared_lights_state =
                    lights_toggle_update(
                        st.buttons,

                        &left_on,
                        &right_on,
                        &low_on,
                        &high_on
                    );


                lights_gpio_set(
                    left_on,
                    right_on,
                    low_on,
                    high_on,

                    &left_out,
                    &right_out,
                    &low_out,
                    &high_out
                );


                /*
                 * LED fault detection receives the
                 * actual GPIO state.
                 */

                led_fault_update(
                    left_out,
                    right_out,
                    low_out,
                    high_out
                );
            }
        }


        /* =================================================================
         * POWERTRAIN
         * =================================================================
         */

        if (
            ++pt_div >=
            (uint8_t)
            (
                PT_TICK_MS /
                FEEL_MS
            )
        ) {

            struct powertrain_state pt;

            enum shifter_mode sel;


            uint8_t thr =
                (uint8_t)
                (
                    (uint32_t)
                    st.throttle *
                    100U /
                    255U
                );


            uint8_t brk =
                (uint8_t)
                (
                    (uint32_t)
                    st.brake *
                    100U /
                    255U
                );


            pt_div = 0;


            if (
                !shifter_update(&sel)
            ) {

                sel =
                    pt_reverse_request
                    ? SHIFTER_R
                    : SHIFTER_D;
            }


            shared_shifter =
                (uint8_t)sel;


            bool neutral =
                (
                    sel == SHIFTER_N ||
                    sel == SHIFTER_P
                );


            /*
             * Parking pawl only engages
             * when stopped.
             */

            if (
                sel == SHIFTER_P &&
                shared_pt_speed <=
                PARK_PAWL_KMH
            ) {

                brk = 100;
            }


            powertrain_tick(
                thr,
                brk,
                sel == SHIFTER_R,
                neutral
            );


            powertrain_get_state(
                &pt
            );


            shared_pt_rpm =
                pt.engine_rpm;


            shared_pt_speed =
                pt.speed_kmh;


            shared_pt_gear =
                pt.gear;


            shared_pt_reverse =
                pt.reverse;
        }


        /* =================================================================
         * Fixed control period
         * =================================================================
         */

        vTaskDelayUntil(
            &next,
            pdMS_TO_TICKS(FEEL_MS)
        );
    }
}


/* =========================================================================
 * CAN TASK
 *
 * ADC:
 *
 *   PA4 -> Fuel
 *   PA5 -> Temperature
 *
 * adc.c owns:
 *
 *   - ADC hardware
 *   - ADC sampling
 *   - ADC -> percentage conversion
 *   - hysteresis
 *
 * telemetry.c owns:
 *
 *   - CAN 0x0A2
 *   - CAN 0x0A3
 * =========================================================================
 */

static void can_task(void *arg)
{
    TickType_t next =
        xTaskGetTickCount();


    (void)arg;


    for (;;) {

        /* ================================================================
         * Update ADC
         * ================================================================
         */

        adc_update();


        uint8_t fuel =
            adc_get_fuel();


        uint8_t temperature =
            adc_get_temperature();


        /* ================================================================
         * Powertrain telemetry
         * ================================================================
         */

        can_telemetry_update(
            (enum shifter_mode)
            shared_shifter,

            shared_hands_on
        );


        /* ================================================================
         * Lights / faults / fuel / temperature
         * ================================================================
         */

        can_lights_update(
            shared_lights_state,

            get_led_fault_state(),

            shared_buttons,

            fuel,

            temperature
        );


        /* ================================================================
         * Optional remote FFB
         * ================================================================
         */

#if REMOTE_FFB

        {
            struct can_frame rx;


            if (
                shared_ready &&
                mcp2515_recv(&rx) == 0 &&
                rx.id == CAN_ID_FFB
            ) {

                apply_ffb(&rx);
            }
        }

#endif


        /* ================================================================
         * CAN period = 10 ms
         * ================================================================
         */

        vTaskDelayUntil(
            &next,
            pdMS_TO_TICKS(10)
        );
    }
}


/* =========================================================================
 * CPU LOAD
 * =========================================================================
 */

static unsigned cpu_load_permille(void)
{
    static uint32_t last_idle;
    static uint32_t last_cyc;


    uint32_t idle =
        ulTaskGetIdleRunTimeCounter();


    uint32_t cyc =
        portGET_RUN_TIME_COUNTER_VALUE();


    uint32_t d_idle =
        idle - last_idle;


    uint32_t d_cyc =
        cyc - last_cyc;


    uint32_t per_mille =
        d_cyc / 1000U;


    last_idle =
        idle;


    last_cyc =
        cyc;


    if (
        per_mille == 0U ||
        d_idle >= d_cyc
    ) {

        return 0;
    }


    return
        (unsigned)
        (
            (d_cyc - d_idle) /
            per_mille
        );
}


/* =========================================================================
 * LED fault string
 * =========================================================================
 */

static const char *led_fault_str(
    led_fault_state_t s
)
{
    switch (s) {

    case LED_OK:
        return "OK   ";


    case LED_OPEN:
        return "OPEN ";


    case LED_SHORT:
        return "SHORT";


    default:
        return "?    ";
    }
}


/* =========================================================================
 * CONSOLE TASK
 * =========================================================================
 */

static void console_task(void *arg)
{
    uint32_t last_log = 0;


    (void)arg;


    for (;;) {

        console_poll();

        log_flush();


        /*
         * Keep CPU load baseline fresh.
         */

        if (
            log_quiet ||
            !shared_ready
        ) {

            (void)
            cpu_load_permille();
        }


        if (
            shared_ready &&
            !log_quiet &&
            HAL_GetTick() -
            last_log >=
            1000U
        ) {

            uint32_t sent;
            uint32_t nak;
            uint32_t err;


            struct steer_feel_telemetry sf;


            steer_feel_get_telemetry(
                &sf
            );


            /*
             * Convert wheel position
             * to degrees.
             */

            int32_t deg =
                (
                    (
                        (int32_t)
                        shared_steering -
                        32768
                    ) *
                    (steer_range / 2)
                ) /
                32768;


            /*
             * Current shifter.
             */

            enum shifter_mode sel_now =
                (enum shifter_mode)
                shared_shifter;


            /*
             * Gear display.
             */

            char gear_mode_c;


            if (
                sel_now == SHIFTER_N ||
                sel_now == SHIFTER_P
            ) {

                gear_mode_c =
                    shifter_letter(
                        sel_now
                    );
            }

            else {

                gear_mode_c =
                    shared_pt_reverse
                    ? 'R'
                    : 'D';
            }


            unsigned load =
                cpu_load_permille();


            g29_ffb_stats(
                &sent,
                &nak,
                &err
            );


            /*
             * Fuel and temperature
             * are already processed
             * by adc.c.
             */

            uint8_t fuel =
                adc_get_fuel();


            uint8_t temperature =
                adc_get_temperature();


            printf(
                "steer=%+4ld "
                "thr=%3u "
                "brk=%3u "
                "btn=%08lX "
                "vel=%+5d "
                "force=%+6d "
                "ctr=%+6d "
                "p2p=%5u "
                "hands=%d "
                "sel=%c "
                "shf=%4u "
                "gear=%c%u "
                "rpm=%4u "
                "spd=%3u "
                "fuel=%3u "
                "temp=%3u "
                "cpu=%u.%u%% "
                "ffb[ok=%lu nak=%lu err=%lu]%s\r\n",

                (long)deg,

                shared_throttle,

                shared_brake,

                (unsigned long)
                shared_buttons,

                sf.vel,

                sf.force,

                sf.centre,

                sf.p2p,

                (int)
                shared_hands_on,

                shifter_letter(
                    sel_now
                ),

                shifter_raw(),

                gear_mode_c,

                shared_pt_gear,

                shared_pt_rpm,

                shared_pt_speed,

                (unsigned)fuel,

                (unsigned)temperature,

                load / 10U,

                load % 10U,

                (unsigned long)sent,

                (unsigned long)nak,

                (unsigned long)err,

                log_dropped
                    ? "  [log overflow]"
                    : ""
            );


            /*
             * LED fault information.
             */

            printf(
                "led: "
                "L=%s(%4umV) "
                "R=%s(%4umV) "
                "LO=%s(%4umV) "
                "HI=%s(%4umV)\r\n",

                led_fault_str(
                    led_fault_get(0)
                ),

                led_fault_voltage(0),

                led_fault_str(
                    led_fault_get(1)
                ),

                led_fault_voltage(1),

                led_fault_str(
                    led_fault_get(2)
                ),

                led_fault_voltage(2),

                led_fault_str(
                    led_fault_get(3)
                ),

                led_fault_voltage(3)
            );


            last_log =
                HAL_GetTick();
        }


        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }
}


/* =========================================================================
 * MAIN
 * =========================================================================
 */

int main(void)
{
    /* =====================================================================
     * HAL
     * =====================================================================
     */

    HAL_Init();


    /* =====================================================================
     * Board
     *
     * Clock
     * UART
     * LED
     * etc.
     * =====================================================================
     */

    board_init();


    printf(
        "\r\n"
        "============================================\r\n"
        " G29 USB Host - STM32 BlackPill F401\r\n"
        " FreeRTOS + CAN + LED Fault Detection\r\n"
        "============================================\r\n"
    );


    /* =====================================================================
     * USB HOST
     * =====================================================================
     */

    MX_USB_HOST_Init();


    /* =====================================================================
     * MCP2515
     * =====================================================================
     */

    int can_status =
        mcp2515_init();


    printf(
        "MCP2515: %s\r\n",

        can_status == 0
            ? "ok"
            : "not responding"
    );


    /* =====================================================================
     * ADC
     *
     * PA4 = ADC channel 4 = fuel
     * PA5 = ADC channel 5 = temperature
     *
     * adc.c owns all ADC configuration.
     * =====================================================================
     */

    adc_init();


    /* =====================================================================
     * LIGHT GPIO
     *
     * Left  -> PB2
     * Right -> PB7
     * Low   -> PA7
     * High  -> PA8
     * =====================================================================
     */

    lights_gpio_init();


    /* =====================================================================
     * LED fault monitor
     * =====================================================================
     */

    led_fault_init();


    /* =====================================================================
     * LED startup electrical self-test
     * =====================================================================
     */

    led_fault_selftest_start();


    /* =====================================================================
     * Flush startup messages before scheduler
     * =====================================================================
     */

    while (
        log_flush() != 0
    ) {
        /* wait */
    }


    /* =====================================================================
     * CREATE CONTROL TASK
     * =====================================================================
     */

    if (
        xTaskCreate(
            control_task,
            "control",
            STACK_CONTROL,
            NULL,
            PRIO_CONTROL,
            &h_control
        ) != pdPASS
    ) {

        printf(
            "ERROR: control task creation failed\r\n"
        );
    }


    /* =====================================================================
     * CREATE USB TASK
     * =====================================================================
     */

    if (
        xTaskCreate(
            usb_task,
            "usb",
            STACK_USB,
            NULL,
            PRIO_USB,
            &h_usb
        ) != pdPASS
    ) {

        printf(
            "ERROR: USB task creation failed\r\n"
        );
    }


    /* =====================================================================
     * CREATE CAN TASK
     * =====================================================================
     */

    if (
        xTaskCreate(
            can_task,
            "can",
            STACK_CAN,
            NULL,
            PRIO_CAN,
            &h_can
        ) != pdPASS
    ) {

        printf(
            "ERROR: CAN task creation failed\r\n"
        );
    }


    /* =====================================================================
     * CREATE CONSOLE TASK
     * =====================================================================
     */

    if (
        xTaskCreate(
            console_task,
            "console",
            STACK_CONSOLE,
            NULL,
            PRIO_CONSOLE,
            &h_console
        ) != pdPASS
    ) {

        printf(
            "ERROR: console task creation failed\r\n"
        );
    }


    /* =====================================================================
     * START SCHEDULER
     * =====================================================================
     */

    vTaskStartScheduler();


    /* =====================================================================
     * SHOULD NEVER REACH HERE
     * =====================================================================
     */

    while (1) {
    }
}