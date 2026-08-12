/**
 * G29 force-feedback over native USB OTG-FS host on the STM32 BlackPill.
 *
 * This file is the APPLICATION LAYER only — tasks, shared state, the CAN
 * frames, the wheel setup sequence and the console dispatch. The substance
 * lives in modules:
 *
 *   board.c       clock (84 MHz core / 48 MHz USB), LED, console UART
 *   log.c         non-blocking printf ring (a blocking printf ate a whole
 *                 control period, see log.h)
 *   g29_hid.c     custom USB-host HID class + force-feedback transport
 *   powertrain.c  engine + 8-speed automatic + reverse, and the speed-dependent
 *                 steering-feel physics. Pure integer maths, host-testable.
 *   steer_feel.c  the torque model: tyre scrub, damping, self-centring,
 *                 rumble, hands-on detection. Owns the wheel's single FFB
 *                 channel and arbitrates who drives it.
 *   bringup.c     FFB effect sweep + step-response identification ('S').
 *                 Measurement tools, not part of the control path.
 *   mcp2515.c     SPI-to-CAN (the F401 has no bxCAN peripheral)
 *
 * Data flow, once per 10 ms control tick:
 *
 *   wheel --USB--> control_task --pedals--> powertrain --speed--> steer_feel
 *                       |                       |                    |
 *                       |                       +-- rpm/gear ---+    +-- torque
 *                       +-------------------------------------- | ------> wheel
 *                                                              v
 *                                                  can_task --> 0x0A2/0A3
 *
 * Board: WeAct BlackPill STM32F401CEU6 (84 MHz max — NOT 96 MHz like the F411).
 * Console: USART1 on PA9 (TX) / PA10 (RX) at 115200, 8N1. Type '?' for knobs.
 * LED: PC13 (active low) blinks while idle, solid while the wheel is ready.
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
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "telemetry.h"
#include "adc.h"
#include "led_fault.h"

/* CAN frame IDs — see README. Powertrain state out, force feedback in. */
#define CAN_ID_POWERTRAIN  0x0A2U /* simulated engine/gearbox state, cyclic 10 ms */
#define CAN_ID_HANDS       0x0A3U /* hands-on/off, on change + 100 ms heartbeat */
#define CAN_ID_FFB         0x0B0U /* force-feedback command in */

/*
 * Startup FFB self-test: full-scale torque hard left, then hard right, ~700 ms
 * each. The car feel is deliberately subtle, so when nothing seems to happen
 * this answers "is constant force reaching the wheel at all?" without any
 * squinting. Set 0 once force feedback is known good.
 */
#define FFB_SELFTEST 0

/* Last index of the wheel setup sequence; one past it is the settle step. */
#if FFB_SELFTEST
#define INIT_LAST 5
#else
#define INIT_LAST 2
#endif

/*
 * Honour force-feedback commands arriving on CAN 0x0B0.
 *
 * 0 because steer_feel.c owns the torque: two writers on the wheel's single
 * constant-force channel makes both useless. Kept as a flag rather than
 * deleted so the frame layout and apply_ffb() stay live and documented.
 */
#define REMOTE_FFB 0

#define PRIO_CONTROL  4
#define PRIO_USB      3
#define PRIO_CAN      2
#define PRIO_CONSOLE  1

/* Words, not bytes. Console is largest because sscanf() is stack-hungry. */
#define STACK_CONTROL 512
#define STACK_USB     512
#define STACK_CAN     256
#define STACK_CONSOLE 512

/*
 * Published by the control task, consumed by the CAN task. Both are 32-bit or
 * smaller and naturally aligned, so reads and writes are atomic on Cortex-M4 —
 * no lock needed, but they must be volatile or the compiler will cache them.
 */
static TaskHandle_t h_control, h_usb, h_can, h_console;



/* ── Serial console (PA10 RX) ───────────────────────────────────────────── */
/*
 * Line-based, polled, no interrupts — tuning force feedback needs dozens of
 * iterations and reflashing for each one is the slow way. Commands:
 *
 *   t <n>            dither torque       (0..32767)
 *   d <n>            damping gain
 *   v <n>            velocity deadband
 *   h <n>            hands-on threshold
 *   f <7 hex bytes>  send a raw FFB report, e.g. "f 11 00 ff 80 00 00 00"
 *   s                stop all effects
 *   ?                show current values
 */
static bool log_quiet;

/* 'g' console command: requested powertrain direction. Actually engaging
 * reverse still waits for the vehicle to stop -- see the interlock in
 * powertrain_tick().
 *
 * FALLBACK ONLY once an analog lever is wired to PA1: the lever is absolute
 * and wins whenever it has ever read a real detent. This is what a bench with
 * no pot fitted uses. */
static volatile bool pt_reverse_request;

/*
 * Physical lock-to-lock travel, degrees. ONE variable drives both the range
 * command sent to the wheel AND the degree conversion in the log, because
 * those two disagreeing is a silent lie: the firmware used to ask the wheel
 * for 900 while the log divided by 180, so a wheel that HAD unlocked to 900
 * would still have printed +-180 and looked locked.
 *
 * Whether the wheel honours it is a separate question — this unit runs in
 * C294 compat mode (see README) and `f8 81` is a G25+ native command. Set it
 * live with 'R' and turn the wheel to the stops to find out what it actually
 * gives you: the steer= reading saturates at the true mechanical limit
 * regardless of what we asked for.
 */
static int32_t steer_range = 900;

/*
 * Wheel setup timing. The FFB queue is a single overwriting slot, so setup is
 * gated on delivery confirmation rather than on a guessed delay — see the
 * ack-gate comment in control_task().
 *
 * SETTLE lets the interrupt-OUT pipe become serviceable before the first
 * command is queued; ACK_TIMEOUT bounds how long a single step may wait for
 * its URB to complete before setup gives up on it and moves on.
 */
#define SETUP_SETTLE_MS      600U
#define SETUP_ACK_TIMEOUT_MS 500U

/*
 * Re-assert the setup a few times after start-up.
 *
 * The ack gate proved delivery is no longer the problem: at boot the range
 * command completes with USBH_URB_DONE and the wheel STILL keeps its narrow
 * travel. So it is acked and ignored — the wheel's own FFB engine evidently
 * is not ready to act on a range command that arrives moments after
 * enumeration, even though the endpoint accepts it.
 *
 * There is no way to read the range back, so the robust answer is to say it
 * again once the wheel has definitely finished waking up. Cheap: each re-send
 * costs one 10 ms torque update.
 */
#define SETUP_REASSERTS   4
#define SETUP_REASSERT_MS 2000U

/*
 * Set by the 'R' console command to re-run the ack-gated setup sequence.
 *
 * 'R' must NOT call g29_send_range() directly: that drops the report into the
 * same single overwriting slot the 100 Hz torque loop is already writing every
 * tick, so it would be clobbered before delivery just as often as the original
 * init-time race was. Routing it through the setup state machine is what makes
 * a live range change actually stick.
 */
static volatile bool setup_redo_request;

/*
 * Command dispatch. Steering-feel knobs are NOT handled here — they belong to
 * steer_feel.c and stay private to it; this tries them first via
 * steer_feel_console() and only then falls through to the app-level commands
 * (range, gear, task stacks, log). Adding a feel knob therefore touches one
 * file, not two.
 */
static void console_exec(char *line)
{
	long v = 0;
	char c = line[0];

	/* Single-letter knobs that take a number. */
	if (c && strchr("tvhpiraenymocbCuxRg", c) != NULL) {
		if (sscanf(line + 1, "%ld", &v) != 1) {
			printf("? need a number\r\n");
			return;
		}

		if (steer_feel_console(c, v)) {
			printf("ok ");
			steer_feel_print_values();
			return;
		}

		switch (c) {
		case 'R':
			/* Same 40..900 clamp the wheel API enforces, so the degree
			 * conversion can never disagree with what was actually
			 * sent. Delivery is handed to the ack-gated setup sequence,
			 * NOT sent directly — see setup_redo_request. */
			steer_range = (v < 40) ? 40 : ((v > 900) ? 900 : v);
			setup_redo_request = true;
			printf("ok range=%ld deg requested, re-running setup...\r\n",
			       (long)steer_range);
			return;

		case 'g':
			pt_reverse_request = (v != 0);
			printf("ok gear=%s (engages once stopped)\r\n",
			       pt_reverse_request ? "R" : "D");
			return;

		default:
			printf("? unhandled '%c'\r\n", c);
			return;
		}
	}

	switch (c) {
	case 'f': {
		unsigned b[7] = {0};
		int n = sscanf(line + 1, "%x %x %x %x %x %x %x",
			       &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6]);
		uint8_t cmd[7];

		if (n < 1) {
			printf("? need hex bytes\r\n");
			return;
		}
		for (int i = 0; i < 7; i++) {
			cmd[i] = (uint8_t)b[i];
		}
		printf("raw -> %02X %02X %02X %02X %02X %02X %02X (%d given)\r\n",
		       cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], n);
		g29_send_raw(cmd);
		break;
	}

	case 'k': {
		/* Words still unused at each task's deepest point so far, plus free
		 * heap. Check this before adding tasks — the numbers are what say
		 * whether there is room, rather than guessing at stack sizes. */
		struct { const char *n; TaskHandle_t h; int size; } t[] = {
			{ "control", h_control, STACK_CONTROL },
			{ "usb",     h_usb,     STACK_USB     },
			{ "can",     h_can,     STACK_CAN     },
			{ "console", h_console, STACK_CONSOLE },
		};

		printf("task      stack  free(words)\r\n");
		for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
			printf("  %-8s %5d  %5u\r\n", t[i].n, t[i].size,
			       (unsigned)uxTaskGetStackHighWaterMark(t[i].h));
		}
		printf("heap free %u of %u bytes, log dropped %lu\r\n",
		       (unsigned)xPortGetFreeHeapSize(),
		       (unsigned)configTOTAL_HEAP_SIZE,
		       (unsigned long)log_dropped);
		break;
	}

	case 's':
		steer_feel_stop();
		printf("stopped\r\n");
		break;

	case 'S':
		log_quiet = true;          /* the periodic log would corrupt the CSV */
		steer_feel_sysid_start();
		printf("sid,begin\r\n");
		break;

	case 'q':
		log_quiet = !log_quiet;
		printf("log %s\r\n", log_quiet ? "off" : "on");
		break;

	case '?':
		steer_feel_print_values();
		steer_feel_print_help();
		printf("  R=lock-to-lock range in deg, 40..900 (also fixes the steer= scale)\r\n"
		       "  g <0|1>=powertrain direction request, D or R (engages once stopped)\r\n"
		       "  S=step-id  k=task stacks  f <hex..>  s  q  ?\r\n");
		break;

	default:
		printf("? unknown '%c'\r\n", c);
		break;
	}
}

/*
 * RX is interrupt-driven, not polled.
 *
 * The F4 USART has a ONE-BYTE receive register and no FIFO. At 115200 a
 * character lands every 87 us, so a task polling RXNE every few milliseconds
 * loses all but the last byte of a command and latches the overrun flag. The
 * old kHz main loop got away with polling; a scheduled task cannot.
 */
#define RX_BUF_SZ 64U             /* power of two: the mask below depends on it */

static volatile char rx_buf[RX_BUF_SZ];
static volatile uint8_t rx_head, rx_tail;

void USART1_IRQHandler(void)
{
	USART_TypeDef *u = huart1.Instance;
	uint32_t sr = u->SR;

	if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
		/* Reading DR after SR clears RXNE and, importantly, ORE — leaving
		 * ORE set would wedge reception permanently. */
		char ch = (char)(u->DR & 0xFFU);
		uint8_t next = (uint8_t)((rx_head + 1U) & (RX_BUF_SZ - 1U));

		if (next != rx_tail) {         /* drop when full, never block */
			rx_buf[rx_head] = ch;
			rx_head = next;
		}
	}
}

/* Drain the receive ring; dispatch on newline. */
static void console_poll(void)
{
	static char buf[48];
	static uint8_t len;

	while (rx_tail != rx_head) {
		char ch = rx_buf[rx_tail];

		rx_tail = (uint8_t)((rx_tail + 1U) & (RX_BUF_SZ - 1U));

		if (ch == '\r' || ch == '\n') {
			if (len) {
				buf[len] = '\0';
				len = 0;
				console_exec(buf);
			}
		} else if (len < sizeof(buf) - 1) {
			buf[len++] = ch;
		}
	}
}

/* ── CAN ────────────────────────────────────────────────────────────────── */
/* Dispatch a 0x0B0 command frame onto the existing FFB API in g29_hid.h. */
static void apply_ffb(const struct can_frame *f)
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
			g29_send_constant_force(
				(int16_t)((uint16_t)f->data[1] |
					  ((uint16_t)f->data[2] << 8)));
		}
		break;
	case 0x02:
		if (f->dlc >= 3U) {
			g29_send_autocenter(f->data[1], f->data[2]);
		}
		break;
	case 0x03:
		if (f->dlc >= 3U) {
			g29_send_range((uint16_t)f->data[1] |
				       ((uint16_t)f->data[2] << 8));
		}
		break;
	default:
		break;   /* unknown command: ignore, next frame is 10 ms away */
	}
}

/* ── Tasks ──────────────────────────────────────────────────────────────── */
/*
 * Priorities (higher number wins). Every task blocks, so nothing starves.
 *
 *   4 control  10 ms periodic — torque, car feel, hands-on. Highest because a
 *              late torque update is the one thing that is actually felt.
 *   3 usb      services the host stack. It cannot block on an event: the G29
 *              class polls URB state (see g29_hid.c), so USBH_USE_OS stays 0
 *              and this runs on a 1 ms tick instead. That is ample against a
 *              10 ms report interval, and it yields so the rest can run.
 *   2 can      10 ms periodic — cyclic frames out, force-feedback frames in.
 *   1 console  drains the log ring and parses commands. Lowest on purpose:
 *              serial output must never delay torque.
 */
static volatile bool     shared_ready;
static volatile bool     shared_hands_on;
static volatile uint16_t shared_steering;
static volatile uint8_t  shared_throttle, shared_brake, shared_clutch;
static volatile uint32_t shared_buttons;
static volatile uint8_t  shared_lights_state;   /* packed, CAN-frame bit order */

/*
 * Powertrain sim output, published by control_task (which already reads the
 * pedals every tick) and consumed by can_task/console_task. One writer, and
 * each field is naturally aligned and <=32 bits, so — same reasoning as the
 * pedal/steering shares above — no lock is needed, just volatile.
 */
static volatile uint16_t shared_pt_rpm, shared_pt_speed;
static volatile uint8_t  shared_pt_gear;
static volatile bool     shared_pt_reverse;
static volatile uint8_t  shared_shifter;	/* enum shifter_mode */

/*
 * Road speed at or below which selecting P engages the pawl. Above it P coasts
 * like N — see the powertrain tick in control_task().
 */
#define PARK_PAWL_KMH 5

static void usb_task(void *arg)
{
	(void)arg;
	for (;;) {
		MX_USB_HOST_Process();
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

static void control_task(void *arg)
{
	TickType_t next = xTaskGetTickCount();
	uint32_t last_blink = 0;
	uint32_t t_init = 0;
	uint32_t init_hold = SETUP_SETTLE_MS;
	uint8_t  init_step = 0;
	uint8_t  pt_div = 0;
	uint32_t setup_mark = 0;	/* ffb_sent when the step was queued */
	bool     setup_waiting = false;	/* queued, not yet confirmed on the wire */
	bool     setup_announced = false;
	uint8_t  setup_reasserts = SETUP_REASSERTS;
	uint32_t t_reassert = 0;

	(void)arg;

	powertrain_init();

	for (;;) {
		struct g29_state st = {0};
		bool ready = g29_get_state(&st) == 0;
		bool setup_done;
		bool hands_on = false;
		uint32_t ffb_ok, ffb_nak, ffb_err;

		g29_ffb_stats(&ffb_ok, &ffb_nak, &ffb_err);

		if (ready) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* solid */

			/* 'R' asked for a different range: replay setup so the new
			 * value is delivered under the same ack gate. */
			if (setup_redo_request) {
				setup_redo_request = false;
				init_step = 0;
				init_hold = 0;   /* pipe is already up; no settle needed */
				setup_waiting = false;
				setup_announced = false;
			}

			/*
			 * One-shot wheel setup — ACK-GATED, not timer-gated.
			 *
			 * queue_ffb() in g29_hid.c is a SINGLE slot and overwrites
			 * unconditionally. Advancing on a fixed 50 ms delay meant that
			 * whenever the range command's OUT transfer had not completed
			 * in time, the next step overwrote and destroyed it — and once
			 * the 100 Hz torque loop starts it owns the queue, so the range
			 * never got another chance. The wheel then kept its default
			 * narrow travel. That race is exactly why full lock-to-lock
			 * came up only SOMETIMES.
			 *
			 * So a step is only retired once g29_ffb_stats()'s sent counter
			 * actually advances, which happens on USBH_URB_DONE — proof the
			 * report reached the wheel. SETUP_ACK_TIMEOUT_MS stops a wedged
			 * endpoint from stalling setup for ever.
			 */
			if (init_step <= INIT_LAST) {
			    if (setup_waiting) {
				if (ffb_ok != setup_mark) {
					setup_waiting = false;
					t_init = HAL_GetTick();
					init_step++;
				} else if (HAL_GetTick() - t_init >= SETUP_ACK_TIMEOUT_MS) {
					printf("setup step %u NOT acked in %u ms — "
					       "wheel may keep default range\r\n",
					       init_step, SETUP_ACK_TIMEOUT_MS);
					setup_waiting = false;
					t_init = HAL_GetTick();
					init_step++;
				}
			    } else if (HAL_GetTick() - t_init >= init_hold) {
				/* Not every step queues a report — the trailing
				 * settle step sends nothing, and gating that on an
				 * ack that can never arrive just burns the timeout
				 * and prints a bogus warning. */
				bool queued = true;

				switch (init_step) {
				case 0:
					/* Kill the default centring spring. It is ON at
					 * power-up and is what drags the wheel back to
					 * centre — nothing in this file does that. */
					g29_send_autocenter(0, 0);
					break;
				case 1:
					/* Unlock the full travel. Deliberately sent LAST:
					 * if any earlier command resets the range as a
					 * side effect, ordering it after them means the
					 * range is what survives. Re-sent on every
					 * re-enumeration, since the wheel forgets it, and
					 * re-asserted a few times after boot because this
					 * wheel acks-but-ignores an early one. */
					g29_send_range((uint16_t)steer_range);
					break;
#if FFB_SELFTEST
				case 2:
					printf("FFB self-test: hard LEFT\r\n");
					g29_send_constant_force(-32767);
					break;
				case 3:
					printf("FFB self-test: hard RIGHT\r\n");
					g29_send_constant_force(32767);
					break;
				case 4:
					printf("FFB self-test: done\r\n");
					g29_send_constant_force(0);
					break;
#endif
				default:
					queued = false;   /* pure settle, nothing sent */
					break;
				}
				/* Self-test steps have to be held long enough to *feel*;
				 * setup steps only need the single-slot queue to drain. */
				init_hold = (init_step == 2U || init_step == 3U) ? 700U : 50U;
				t_init = HAL_GetTick();
				if (queued) {
					setup_mark = ffb_ok;
					setup_waiting = true;
				} else {
					init_step++;
				}
			    }
			}
		} else {
			init_step = 0;        /* wheel unplugged: re-run setup on return */
			/* Settle before the first command: the OUT pipe is not
			 * necessarily serviceable the instant reports start arriving,
			 * and a range command queued too early is one the ack gate
			 * above would only discover had been lost. */
			init_hold = SETUP_SETTLE_MS;
			setup_waiting = false;
			/* A replugged wheel is a freshly woken wheel: it needs the
			 * same re-assertion as a cold boot. */
			setup_reasserts = SETUP_REASSERTS;
			t_reassert = HAL_GetTick();
			t_init = HAL_GetTick();
			if (HAL_GetTick() - last_blink >= 200) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				last_blink = HAL_GetTick();
			}
		}

		setup_done = init_step > INIT_LAST;

		/*
		 * Say the range again a few times after start-up. The ack gate
		 * already proves the first one was delivered, so this is aimed at
		 * the wheel ACTING on it: an early range command is acked and
		 * ignored, and one sent a couple of seconds later sticks.
		 * Re-running the whole sequence also re-asserts autocenter-off.
		 */
		if (ready && setup_done && setup_reasserts > 0 &&
		    HAL_GetTick() - t_reassert >= SETUP_REASSERT_MS) {
			printf("re-asserting range=%ld (%u left)\r\n",
			       (long)steer_range, setup_reasserts - 1U);
			setup_reasserts--;
			t_reassert = HAL_GetTick();
			init_step = 0;
			init_hold = 0;      /* pipe is long since up */
			setup_waiting = false;
			/* setup_announced left set: this is a re-assert, not news. */
		}

		/* Announce once, edge-triggered: this is the line that says whether
		 * the range command was actually delivered, which is the difference
		 * between full travel and the wheel's default narrow lock. */
		if (ready && setup_done && !setup_announced) {
			setup_announced = true;
			printf("wheel setup done: range=%ld deg delivered to the wheel "
			       "(turn to the stops; steer= reaches +-%ld only if the "
			       "wheel honours it)\r\n",
			       (long)steer_range, (long)(steer_range / 2));
		}
		if (!ready) {
			setup_announced = false;
		}

		/* One call: steer_feel owns the torque and arbitrates internally
		 * between the bring-up sweep, the identification and the normal
		 * feel loop. Gated on setup_done so the 100 Hz torque stream never
		 * starts before the range/autocenter commands have been delivered
		 * — they share the wheel's single FFB slot. */
		if (ready && setup_done) {
			hands_on = steer_feel_update(st.steering);
		}

		shared_steering = st.steering;
		shared_throttle = st.throttle;
		shared_brake    = st.brake;
		shared_clutch   = st.clutch;
		shared_buttons  = st.buttons;
		shared_hands_on = hands_on;
		shared_ready    = ready;

		/*
		 * Single source of truth for the 4 light toggle states -- see
		 * lights_toggle_update()'s comment in telemetry.h. Computed
		 * once here, consumed two ways: packed for the CAN lights
		 * frame (shared_lights_state, sent from can_task), and
		 * directly for led_fault_update()'s gating below, in the
		 * same tick, same task -- no re-derivation, no drift between
		 * the two consumers.
		 */
		{
			bool left_on, right_on, low_on, high_on;
			bool left_out, right_out, low_out, high_out;

			shared_lights_state = lights_toggle_update(
				st.buttons, &left_on, &right_on, &low_on, &high_on);

			/* Drive the physical pins first -- see lights_gpio_set()'s
			 * comment in telemetry.h. left_out/right_out come back
			 * gated by the indicator blink phase (500 ms on/off);
			 * low_out/high_out always equal low_on/high_on. */
			lights_gpio_set(left_on, right_on, low_on, high_on,
					 &left_out, &right_out, &low_out, &high_out);

			/*
			 * LED fault sampling: one round-robin ADC reading per
			 * tick, non-blocking (see led_fault.c) -- cheap enough
			 * to run every tick unconditionally. Deliberately kept
			 * in control_task, the sole owner of ADC1:
			 * shifter_update() below is the only other caller, and
			 * it's in this same task, so the two can never race
			 * each other's HAL_ADC_* calls. can_task only ever
			 * reads the fault verdict via led_fault_get() (see
			 * telemetry.c), never the ADC itself.
			 *
			 * Fed the actual driven pin states (left_out/right_out),
			 * not the raw toggle-enable states (left_on/right_on):
			 * the indicators blink at 1 Hz while toggled on, so
			 * left_on/right_on stay true through the 500 ms dark
			 * half of every blink cycle -- sampling against that
			 * would read the node as SHORT every time the blinker
			 * is legitimately dark.
			 */
			led_fault_update(left_out, right_out, low_out, high_out);
		}

		/*
		 * Powertrain sim: FEEL_MS (10 ms) x2 = PT_TICK_MS (20 ms), the
		 * period its drag/shift-timing constants are tuned against.
		 * Cheap integer math (one array lookup, a handful of
		 * multiplies) — negligible next to steer_feel_update() above, so it
		 * doesn't threaten the torque loop this task exists for.
		 */
		if (++pt_div >= (uint8_t)(PT_TICK_MS / FEEL_MS)) {
			struct powertrain_state pt;
			enum shifter_mode sel;
			uint8_t thr = (uint8_t)((uint32_t)st.throttle * 100U / 255U);
			uint8_t brk = (uint8_t)((uint32_t)st.brake * 100U / 255U);

			pt_div = 0;

			/*
			 * The lever owns the gear whenever one is wired. 'g'
			 * stays as the fallback for a bench with nothing on PA1
			 * — see shifter_update() for why that is decided by
			 * having seen a real detent rather than by a #define.
			 */
			if (!shifter_update(&sel)) {
				sel = pt_reverse_request ? SHIFTER_R : SHIFTER_D;
			}
			shared_shifter = (uint8_t)sel;

			/*
			 * powertrain.c now models N/P directly via the neutral
			 * flag to powertrain_tick() -- see neutral_tick() there.
			 * Throttle is passed through UNCHANGED (previously
			 * forced to 0 here, which is why RPM used to just decay
			 * in N/P regardless of the pedal instead of free-revving
			 * the way a disconnected gearbox actually behaves).
			 *
			 * P: a parking pawl, not a brake. It only holds once
			 * the car is already stopped — forcing full brake at
			 * any speed would turn selecting P into an emergency
			 * stop, which is not what the lever does in a real car
			 * (the pawl simply refuses to engage above walking
			 * pace). Above that it behaves as N.
			 */
			bool neutral = (sel == SHIFTER_N || sel == SHIFTER_P);

			if (sel == SHIFTER_P && shared_pt_speed <= PARK_PAWL_KMH) {
				brk = 100;
			}

			powertrain_tick(thr, brk, sel == SHIFTER_R, neutral);
			powertrain_get_state(&pt);
			shared_pt_rpm     = pt.engine_rpm;
			shared_pt_speed   = pt.speed_kmh;
			shared_pt_gear    = pt.gear;
			shared_pt_reverse = pt.reverse;
		}

		/*
		 * Absolute deadline, not a relative delay: vTaskDelayUntil keeps the
		 * period fixed even when an iteration runs long, so the velocity
		 * estimate stays valid. It is a difference over a FIXED interval, and
		 * a drifting interval would silently scale every gain in the car-feel
		 * model.
		 */
		vTaskDelayUntil(&next, pdMS_TO_TICKS(FEEL_MS));
	}
}


static void can_task(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    (void)arg;

    for (;;) {
        /*
         * Execute CAN telemetry routine (10 ms periodic + event-triggered on hands state change).
         * This handles packing RPM, Speed, Gear, Hands-On/Off status, and the rolling counter.
         *
         * shared_shifter is published by control_task() every FEEL_MS tick;
         * telemetry needs the real PRND selection (not just what
         * powertrain.c's D/R-only state can express) so the CAN Mode byte
         * can report N and P correctly -- see the fix in telemetry.c.
         */
        /*
         * shared_hands_on is the real hod_update() verdict, published by
         * control_task() every FEEL_MS tick -- see the parameter comment in
         * telemetry.h for why this has to be passed in rather than
         * re-derived inside can_telemetry_update() itself.
         */
        can_telemetry_update((enum shifter_mode)shared_shifter, shared_hands_on);

		can_lights_update(shared_lights_state, get_led_fault_state());



        /*
         * Force feedback frame drain from MCP2515 buffer (to prevent hardware RX buffer overflow)
         */
        struct can_frame rx;
        if (shared_ready && mcp2515_recv(&rx) == 0 && rx.id == CAN_ID_FFB && REMOTE_FFB) {
            apply_ffb(&rx);
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(10));
    }
}


/*
 * CPU load over the interval between two console lines, in permille.
 *
 * Both counters run on the DWT cycle counter, so this is the idle task's real
 * share of the elapsed time, not a guess from loop counts. Unsigned differences
 * survive the ~51 s CYCCNT wrap; the kernel drops the single slice that spans
 * the wrap (tasks.c only accumulates when the counter moved forward), so once
 * every 51 s the figure can read a hair high. Not worth a 64-bit counter.
 */
static unsigned cpu_load_permille(void)
{
	static uint32_t last_idle, last_cyc;
	uint32_t idle = ulTaskGetIdleRunTimeCounter();
	uint32_t cyc  = portGET_RUN_TIME_COUNTER_VALUE();
	uint32_t d_idle = idle - last_idle;
	uint32_t d_cyc  = cyc - last_cyc;
	/* d_cyc is ~84e6 for a 1 s window, so busy*1000 would overflow 32 bits —
	 * scale the divisor instead of the dividend. */
	uint32_t per_mille = d_cyc / 1000U;

	last_idle = idle;
	last_cyc  = cyc;

	if (per_mille == 0U || d_idle >= d_cyc) {
		return 0;
	}
	return (unsigned)((d_cyc - d_idle) / per_mille);
}

/*
 * "OK"/"OPEN"/"SHORT" for the console line below. Fixed width (5 chars,
 * space-padded) so the four LED fields line up column-to-column across
 * prints instead of the line length jittering with the verdict.
 */
static const char *led_fault_str(led_fault_state_t s)
{
	switch (s) {
	case LED_OK:    return "OK   ";
	case LED_OPEN:  return "OPEN ";
	case LED_SHORT: return "SHORT";
	default:        return "?    ";
	}
}

static void console_task(void *arg)
{
	uint32_t last_log = 0;

	(void)arg;

	for (;;) {
		console_poll();
		log_flush();

		/* Keep the load baseline fresh while the line is not being printed
		 * ('q', or no wheel yet): a silence longer than the ~51 s CYCCNT
		 * wrap would otherwise make the first line after it read nonsense. */
		if (log_quiet || !shared_ready) {
			(void)cpu_load_permille();
		}

		if (shared_ready && !log_quiet &&
		    HAL_GetTick() - last_log >= 1000) {
			uint32_t sent, nak, err;
			struct steer_feel_telemetry sf;

			steer_feel_get_telemetry(&sf);
			/* 0..0xFFFF -> +-(steer_range/2) deg. Driven by the same
			 * variable as the range command, so the reading means
			 * what it says whatever the wheel is set to. */
			int32_t deg = ((int32_t)shared_steering - 32768) *
				      (steer_range / 2) / 32768;

			/*
			 * gear= mode letter. powertrain.c only models D and R —
			 * shared_pt_reverse can never say N or P — so in those two
			 * modes it was reporting whatever forward/reverse gear the
			 * sim last held (e.g. "gear=D1" while the shifter sat in
			 * P, or a stale "gear=R0" for several ticks after leaving
			 * R). Take the mode letter from the actual shifter
			 * selection instead; only fall back to the sim's own D/R
			 * call when the selector really is in D or R, since that
			 * is the one distinction the sim tracks correctly (e.g.
			 * the R->D interlock holding "R0" until speed_accum hits
			 * zero is real state, not a display bug).
			 */
			enum shifter_mode sel_now = (enum shifter_mode)shared_shifter;
			char gear_mode_c = (sel_now == SHIFTER_N || sel_now == SHIFTER_P)
					   ? shifter_letter(sel_now)
					   : (shared_pt_reverse ? 'R' : 'D');

			unsigned load = cpu_load_permille();

			g29_ffb_stats(&sent, &nak, &err);
			printf("steer=%+4ld thr=%3u brk=%3u btn=%08lX "
			       "vel=%+5d force=%+6d ctr=%+6d p2p=%5u hands=%d "
			       "sel=%c shf=%4u gear=%c%u rpm=%4u spd=%3u "
			       "cpu=%u.%u%% "
			       "ffb[ok=%lu nak=%lu err=%lu]%s\r\n",
			       (long)deg, shared_throttle, shared_brake,
			       (unsigned long)shared_buttons, sf.vel, sf.force,
			       sf.centre, sf.p2p, (int)shared_hands_on,
			       shifter_letter(sel_now),
			       shifter_raw(),
			       gear_mode_c, shared_pt_gear,
			       shared_pt_rpm, shared_pt_speed,
			       load / 10U, load % 10U,
			       (unsigned long)sent, (unsigned long)nak,
			       (unsigned long)err,
			       log_dropped ? "  [log overflow]" : "");

			/*
			 * LED fault line: state + raw mV for all 4 lights, same
			 * 1 s cadence as the line above. led_fault_get()/
			 * led_fault_voltage() are plain reads of state
			 * control_task() computed via led_fault_update() -- safe
			 * to call from console_task() too, same as can_task()
			 * already does for the CAN fault frame (see
			 * can_led_fault_update() in telemetry.c). LED index
			 * order matches led_fault.c: 0=left, 1=right, 2=low
			 * beam, 3=high beam.
			 */
			printf("led: L=%s(%4umV) R=%s(%4umV) "
			       "LO=%s(%4umV) HI=%s(%4umV)\r\n",
			       led_fault_str(led_fault_get(0)), led_fault_voltage(0),
			       led_fault_str(led_fault_get(1)), led_fault_voltage(1),
			       led_fault_str(led_fault_get(2)), led_fault_voltage(2),
			       led_fault_str(led_fault_get(3)), led_fault_voltage(3));

			last_log = HAL_GetTick();
		}

		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

int main(void)
{
	HAL_Init();
	board_init();

	printf("\r\n=== G29 USB host (STM32 BlackPill, FreeRTOS) ===\r\n");

	MX_USB_HOST_Init();

	printf("MCP2515: %s\r\n", mcp2515_init() == 0 ? "ok" : "not responding");

	adc_init();
	/* shifter no longer needs its own init -- adc_init() above already
	 * configures PA1 as analog and brings up the shared ADC1 peripheral. */
	led_fault_init();
	lights_gpio_init();

	/* Nothing has drained the ring yet — push the banner out synchronously so
	 * a failure before the scheduler starts is still visible. */
	while (log_flush() != 0) {
	}

	xTaskCreate(control_task, "control", STACK_CONTROL, NULL, PRIO_CONTROL, &h_control);
	xTaskCreate(usb_task,     "usb",     STACK_USB,     NULL, PRIO_USB,     &h_usb);
	xTaskCreate(can_task,     "can",     STACK_CAN,     NULL, PRIO_CAN,     &h_can);
	xTaskCreate(console_task, "console", STACK_CONSOLE, NULL, PRIO_CONSOLE, &h_console);

	vTaskStartScheduler();

	/* Only reached if the heap was too small for the tasks above. */
	for (;;) {}
}