/*
 * Logitech G29 Driving Force — custom USB-host class for STM32Cube / ST USBH.
 *
 * Protocol reference (same as the Zephyr port):
 *   berarma/new-lg4ff  — mode-switch, device IDs, FFB byte format
 *   misarb/g29rs       — FFB report examples (PS3 mode)
 *   Linux kernel       — drivers/hid/hid-lg4ff.c
 *
 * Put the wheel's base switch in PS3 mode. It enumerates as 046D:C24F and
 * accepts the classic Logitech FFB reports used here.
 *
 * Force feedback is sent on the interrupt-OUT endpoint (EP type INTR), which
 * is how lg4ff drives Logitech wheels — simpler and more responsive than a
 * SET_REPORT control transfer.
 */
#include "g29_hid.h"
#include "usbh_core.h"
#include "usbh_pipes.h"
#include "usbh_ioreq.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#define LOGITECH_VID   0x046DU
#define G29_PID        0xC24FU

/* ── FFB command templates ──────────────────────────────────────────────── */
/*
 * Command byte is (slot_mask << 4) | command. 0xF = all slots.
 *   cmd 3 = stop         -> 0xF3
 *   cmd 4 = default spring on
 *   cmd 5 = default spring off
 * The wheel powers up with its default centring spring ON, so anything that
 * wants the wheel to hold position has to send 0xF5 explicitly.
 */
static const uint8_t cmd_no_effect[7]   = { 0xf3, 0, 0, 0, 0, 0, 0 };
static const uint8_t cmd_spring_off[7]  = { 0xf5, 0, 0, 0, 0, 0, 0 };

/*
 * Native-mode switch (lg4ff / g29rs). A G29 that enumerates as 046D:C294 is in
 * Logitech's restricted "Driving Force" compatibility descriptor: input offsets
 * differ and FFB is unavailable. These two commands make it detach and
 * re-enumerate as C24F (PS3 mode), where the parser + FFB below are correct.
 * The `switched` guard in g29_ClassRequest survives re-enumeration so it fires
 * exactly once (no detach loop).
 */
#define G29_DO_MODE_SWITCH 1
/*
 * Byte-for-byte from lg4ff's mode-switch table, lg4ff_mode_switch_ext09_g29:
 *
 *     {0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,
 *      0xf8, 0x09, 0x05, 0x01, 0x01, 0x00, 0x00}
 *
 *   f8 0a — the ext09 prologue, sent ahead of EVERY ext09 wheel's switch
 *   f8 09 05 01 01 — select G29 native mode (05), detaching to re-enumerate
 *
 * A previous revision of this file recorded these exact bytes as an "earlier
 * guess" that "was not the G29 sequence", and replaced them with
 * f8 09 00 01 00 + f8 13. That is backwards: the f8 13 pair is the generic
 * DFEX/ext09 pattern, and it is why every attempt re-enumerated as C294
 * again. Byte 3 selects the wheel model (00 DFEX, 01 DFP, 02 G25, 03 DFGT,
 * 04 G27, 05 G29) — sending 00 asks a G29 to become a Driving Force, which is
 * the mode it is already in. Do not "fix" this back without re-reading
 * drivers/hid/hid-lg4ff.c.
 *
 * Also recorded there, and disproven by an entire session of moving this
 * wheel around under a probe: "in C294 the FFB motor is not connected to the
 * effect engine — nothing moves". Constant force works fine in C294. Native
 * mode is worth having for the 16-bit STEERING (C294 reports 8 bits, so one
 * count is 3.5 deg at 900 deg lock-to-lock, and the hands-on probe cannot
 * swing less than ~7 deg and still mean anything), not for the motor.
 *
 * The arm may be one-shot: the reattach's own USB reset can consume it, so
 * the first attempt switches and immediately reverts. Hence MODESW_TRIES —
 * retry across re-enumerations instead of giving up after one detach.
 */
#if G29_DO_MODE_SWITCH
#define MODESW_TRIES  3
#define MODESW_WAIT   40U   /* ms between commands; URB polling raced here */

static const uint8_t modesw_cmds[2][7] = {
	{ 0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0xf8, 0x09, 0x05, 0x01, 0x01, 0x00, 0x00 },
};

static uint8_t modesw_tries;   /* survives re-enumeration */
static int     modesw_step;    /* reset per enumeration by InterfaceInit */
#endif

/* ── Driver state ───────────────────────────────────────────────────────── */
enum in_state  { IN_GET, IN_POLL, IN_WAIT };
enum out_state { OUT_IDLE, OUT_BUSY };

static struct {
	USBH_HandleTypeDef *phost;
	uint8_t  iface;

	uint8_t  in_pipe,  in_ep,  in_len;
	uint8_t  out_pipe, out_ep, out_mps;
	uint16_t poll_ms;
	bool     native;      /* true = C24F native layout, false = C294 compat */

	enum in_state  in_st;
	enum out_state out_st;
	uint32_t in_timer;

	volatile bool ready;
	volatile bool ffb_pending;
	volatile uint32_t ffb_sent, ffb_nak, ffb_err;   /* OUT endpoint health */
	uint8_t  ffb_buf[7];
	uint8_t  in_buf[64] __attribute__((aligned(4)));

	struct g29_state state;
} g29;

/* ── Report parsing ─────────────────────────────────────────────────────── */
/*
 * G29 native-mode input layout (PS3 switch). Offsets are best-effort and MUST
 * be verified with usbmon on a PC — capture the wheel talking to lg4ff and
 * line up the bytes. See README.
 */
/*
 * ponytail: raw dump to map report offsets; set 0 once mapped.
 *
 * Both layouts are mapped and verified against live dumps, so this stays off.
 * It earned its keep once more finding the D-pad up+right steering corruption
 * (see the `off` comment in parse_report()) -- that dump is the reason the
 * `g29.native` gate it used to carry is gone: the gate suppressed the dump in
 * whichever mode was not being debugged, which is never what you want from a
 * tool you reach for precisely when you do not yet know what is happening.
 */
#define G29_DUMP_RAW 0

/*
 * How many leading bytes the dump compares and prints.
 *
 * Every input field (buttons, hat, steering, pedals) lives in the first few
 * bytes; the TAIL of the report is the wheel echoing back the last FFB command
 * we sent it, which changes every single tick. Comparing the whole report
 * would therefore make the "only print on change" test always true and turn
 * this into a firehose that buries the very transition being chased.
 */
#define G29_DUMP_BYTES 8

/*
 * Publish a decoded report as one indivisible update. Built into a local first
 * so the critical section covers the whole struct: a reader preempting a
 * partial write would get steering from one report and pedals from the next,
 * and the velocity estimator reads that as a large phantom step.
 */
static void commit_state(const struct g29_state *src)
{
	taskENTER_CRITICAL();
	g29.state = *src;
	taskEXIT_CRITICAL();
}

static void parse_report(const uint8_t *d, uint8_t len)
{
	/*
	 * Neither layout carries a Report ID, so there is nothing to skip.
	 *
	 * This used to be `(d[0] == 0x01) ? 1 : 0` -- a guess made from the
	 * VALUE of a byte that carries live input data. In native C24F, d0's low
	 * nibble is the D-pad hat, and the HID hat encoding puts NORTH-EAST at
	 * exactly 1. So pressing up+right together made d0 == 0x01, the parser
	 * skipped a byte that was never there, and steering was read from
	 * d[5]|d[6]<<8 instead of d[4]|d[5]<<8. d6 is the throttle byte, 0xFF at
	 * rest, so steering became 0xFF7D = 65405 -- FULL RIGHT LOCK -- and
	 * self-centring instantly slammed the wheel left. Confirmed on a live
	 * dump, with right-alone (hat=2) harmless and up+right (hat=1) not:
	 *
	 *   raw len=12 off=0:*02 00 00 00 B2 7F FF FF   right alone: fine
	 *   raw len=12 off=1:*01 00 00 00 E9 7D FF FF   up+right:    corrupt
	 *     -> steer=+448 vel=+6134 force=-18869 ctr=-6162
	 *
	 * Report-ID presence is a property of the DEVICE, never of the data --
	 * a value test on an input byte can always be spoofed by input. Measured
	 * ID-less at len=12 (native) and len=27 (C294); if a device ever does
	 * carry one, decide it from the report LENGTH, which is constant.
	 */
	const int off = 0;
	struct g29_state st = {0};

#if G29_DUMP_RAW
	/*
	 * Print only when the INPUT bytes actually change, so idle is silent and
	 * moving one control shows exactly which offsets are live. Deliberately
	 * NOT gated on g29.native any more -- see G29_DUMP_RAW.
	 *
	 * `len` and `off` are printed alongside the bytes because they are the
	 * whole point of this dump: `off` is what the Report-ID heuristic below
	 * decided, and `len` is the constant that should have decided it.
	 * Watch off flip 0 -> 1 on a D-pad NE press with the bytes unmoved.
	 */
	static uint8_t prev[G29_DUMP_BYTES];
	static uint32_t last_dump;
	uint8_t ncmp = (len < G29_DUMP_BYTES) ? len : G29_DUMP_BYTES;

	if (memcmp(prev, d, ncmp) != 0 && HAL_GetTick() - last_dump >= 60) {
		last_dump = HAL_GetTick();
		printf("raw len=%2u off=%d:", len, off);
		for (uint8_t i = 0; i < ncmp; i++) {
			/* mark the bytes that moved since the last print */
			printf("%c%02X", (prev[i] != d[i]) ? '*' : ' ', d[i]);
		}
		printf("\r\n");
		memcpy(prev, d, ncmp);
	}
#endif

	if (len < (uint8_t)(off + 9)) {
		return;
	}

	if (g29.native) {
		/*
		 * C24F native, read off a live 12-byte dump:
		 *   d0     low nibble = hat (8 = centred), high nibble = buttons
		 *   d1..d3 buttons
		 *   d4,d5  steering, 16-bit LITTLE-endian, full scale over the
		 *          configured range — 0x0000 at full left, and 0x373C
		 *          decoded to -255 deg at 900 deg, which is the check
		 *          that this is a true 16 bits and not a widened byte.
		 *          0.0137 deg per LSB, against 3.5 in C294.
		 *   d6,d7,d8 throttle, brake, clutch — INVERTED, 0xFF released
		 *   d9,dA  sit at 0x80, no signal seen
		 *   dB     changes constantly, not identified, unused
		 *
		 * The hat is masked out of the buttons: it rests at 8, not 0, so
		 * folding d0 in whole reported a permanently-pressed button.
		 */
		st.buttons  = (uint32_t)(d[off] >> 4) |
			      ((uint32_t)d[off + 1] << 4) |
			      ((uint32_t)d[off + 2] << 12) |
			      ((uint32_t)d[off + 3] << 20);
		st.steering = (uint16_t)d[off + 4] | ((uint16_t)d[off + 5] << 8);
		/* Released is 0xFF, pressed is 0x00. Taken raw, an untouched
		 * pedal set read as full throttle AND full brake together. */
		st.throttle = (uint8_t)(255U - d[off + 6]);
		st.brake    = (uint8_t)(255U - d[off + 7]);
		st.clutch   = (uint8_t)(255U - d[off + 8]);
		commit_state(&st);
		return;
	}

	/*
	 * C294 "Driving Force" compat layout, read off a live 27-byte dump:
	 *   d0,d1  buttons
	 *   d2     low nibble = hat (8 = centred), high nibble = more buttons
	 *   d3     steering — 8 bits only, so ~1.4° per count over 360°
	 *   d4     COMBINED pedal axis (see below)
	 *   d5,d6  sit at 0x7F permanently — no signal, not brake/clutch
	 *   d16..  the wheel echoing back the last FFB report we sent it
	 */
	st.buttons  = (uint32_t)d[off] | ((uint32_t)d[off + 1] << 8) |
		      ((uint32_t)(d[off + 2] >> 4) << 16);
	/* Widen 8-bit to the 0..0xFFFF the rest of the code expects. Repeating
	 * the byte (not shifting) keeps 0x00->0x0000 and 0xFF->0xFFFF exact. */
	st.steering = (uint16_t)((uint16_t)d[off + 3] << 8 | d[off + 3]);

	/*
	 * Combined pedals. C294 reports throttle and brake summed onto ONE axis:
	 * 0x7F with nothing pressed, falling as throttle goes down and rising as
	 * brake goes down. Measured over a live capture: d4 spans 0..255 with 62
	 * distinct values while d5 and d6 never leave 0x7F.
	 *
	 * Each half is only 7 bits, so scale back up to the 0..255 the API
	 * promises. Clutch has no signal at all here and reads 0.
	 *
	 * LIMITATION, and it is the descriptor's not ours: because the two pedals
	 * share an axis they cancel. Pressing both together reads as neither, so
	 * brake-and-throttle overlap cannot be detected in this mode. Separate
	 * pedal axes only exist in native C24F.
	 */
	{
		int32_t p = (int32_t)d[off + 4] - 127;

		st.throttle = (p < 0) ? (uint8_t)((-p) * 255 / 127) : 0U;
		st.brake    = (p > 0) ? (uint8_t)(( p) * 255 / 128) : 0U;
		st.clutch   = 0U;
	}
	commit_state(&st);
}

/* ── Endpoint discovery ─────────────────────────────────────────────────── */
static void find_endpoints(USBH_HandleTypeDef *phost, uint8_t iface)
{
	USBH_InterfaceDescTypeDef *itf = &phost->device.CfgDesc.Itf_Desc[iface];

	g29.in_ep = g29.out_ep = 0;

	for (uint8_t i = 0; i < itf->bNumEndpoints && i < USBH_MAX_NUM_ENDPOINTS; i++) {
		USBH_EpDescTypeDef *ep = &itf->Ep_Desc[i];

		if ((ep->bmAttributes & 0x03U) != USB_EP_TYPE_INTR) {
			continue;
		}
		if (ep->bEndpointAddress & 0x80U) {
			g29.in_ep  = ep->bEndpointAddress;
			g29.in_len = (uint8_t)MIN(ep->wMaxPacketSize, sizeof(g29.in_buf));
			g29.poll_ms = ep->bInterval ? ep->bInterval : 4U;
		} else {
			g29.out_ep  = ep->bEndpointAddress;
			g29.out_mps = (uint8_t)MIN(ep->wMaxPacketSize, 64U);
		}
	}
}

/* ── USBH class callbacks ───────────────────────────────────────────────── */
static USBH_StatusTypeDef g29_InterfaceInit(USBH_HandleTypeDef *phost)
{
	uint8_t iface;

	iface = USBH_FindInterface(phost, 0x03U /* HID */, 0xFFU, 0xFFU);
	if (iface == 0xFFU) {
		USBH_ErrLog("G29: no HID interface");
		return USBH_FAIL;
	}

	USBH_SelectInterface(phost, iface);
	g29.iface = iface;
	g29.phost = phost;

	find_endpoints(phost, iface);
	if (g29.in_ep == 0U) {
		USBH_ErrLog("G29: no interrupt-IN endpoint");
		return USBH_FAIL;
	}

	/* IN pipe */
	g29.in_pipe = USBH_AllocPipe(phost, g29.in_ep);
	USBH_OpenPipe(phost, g29.in_pipe, g29.in_ep, phost->device.address,
		      phost->device.speed, USB_EP_TYPE_INTR, g29.in_len);
	USBH_LL_SetToggle(phost, g29.in_pipe, 0U);

	/* OUT pipe (force feedback) — optional but present on the G29 */
	if (g29.out_ep != 0U) {
		g29.out_pipe = USBH_AllocPipe(phost, g29.out_ep);
		USBH_OpenPipe(phost, g29.out_pipe, g29.out_ep, phost->device.address,
			      phost->device.speed, USB_EP_TYPE_INTR, g29.out_mps);
		USBH_LL_SetToggle(phost, g29.out_pipe, 0U);
	}

	g29.in_st  = IN_GET;
	g29.out_st = OUT_IDLE;
	g29.ffb_pending = false;
	g29.ready = true;
	g29.native = (phost->device.DevDesc.idProduct == 0xC24FU);
#if G29_DO_MODE_SWITCH
	modesw_step = 0;   /* re-run the mode switch on each fresh enumeration */
#endif

	USBH_UsrLog("G29 ready: VID=%04X PID=%04X in_ep=0x%02X out_ep=0x%02X poll=%ums",
		    phost->device.DevDesc.idVendor, phost->device.DevDesc.idProduct,
		    g29.in_ep, g29.out_ep, g29.poll_ms);
	return USBH_OK;
}

static USBH_StatusTypeDef g29_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
	g29.ready = false;

	if (g29.in_pipe) {
		USBH_ClosePipe(phost, g29.in_pipe);
		USBH_FreePipe(phost, g29.in_pipe);
		g29.in_pipe = 0;
	}
	if (g29.out_pipe) {
		USBH_ClosePipe(phost, g29.out_pipe);
		USBH_FreePipe(phost, g29.out_pipe);
		g29.out_pipe = 0;
	}
	return USBH_OK;
}

static USBH_StatusTypeDef g29_ClassRequest(USBH_HandleTypeDef *phost)
{
#if G29_DO_MODE_SWITCH
	static uint32_t t0;

	/* Already native (C24F), no OUT pipe, or out of retries: proceed. */
	if (g29.out_pipe == 0U || modesw_tries >= MODESW_TRIES ||
	    phost->device.DevDesc.idProduct == 0xC24FU) {
		static uint16_t told_pid;
		if (told_pid != phost->device.DevDesc.idProduct) {
			told_pid = phost->device.DevDesc.idProduct;
			printf("modesw: SKIP (tries=%u out_pipe=%u pid=%04X)\r\n",
			       modesw_tries, g29.out_pipe, told_pid);
		}
		return USBH_OK;
	}

	/* Even steps send a command, odd steps hold it for MODESW_WAIT ms. */
	if ((modesw_step & 1) == 0) {
		uint8_t i = (uint8_t)(modesw_step / 2);

		printf("modesw[%u]: send %02X %02X %02X %02X %02X\r\n", modesw_tries,
		       modesw_cmds[i][0], modesw_cmds[i][1], modesw_cmds[i][2],
		       modesw_cmds[i][3], modesw_cmds[i][4]);
		USBH_InterruptSendData(phost, (uint8_t *)modesw_cmds[i],
				       sizeof(modesw_cmds[i]), g29.out_pipe);
		t0 = HAL_GetTick();
		modesw_step++;
	} else if (HAL_GetTick() - t0 >= MODESW_WAIT) {
		modesw_step++;
		if (modesw_step / 2 >=
		    (int)(sizeof(modesw_cmds) / sizeof(modesw_cmds[0]))) {
			modesw_tries++;
			printf("modesw: attempt %u sent — expect detach\r\n",
			       modesw_tries);
		}
	}
	return USBH_BUSY;
#else
	(void)phost;
	return USBH_OK;  /* ready for normal processing */
#endif
}

static USBH_StatusTypeDef g29_Process(USBH_HandleTypeDef *phost)
{
	/* ---- interrupt IN: poll the wheel ---- */
	switch (g29.in_st) {
	case IN_GET:
		USBH_InterruptReceiveData(phost, g29.in_buf, g29.in_len, g29.in_pipe);
		g29.in_st = IN_POLL;
		g29.in_timer = HAL_GetTick();
		break;

	case IN_POLL: {
		USBH_URBStateTypeDef us = USBH_LL_GetURBState(phost, g29.in_pipe);
		/* ponytail: URB-state spam drowns the raw dump; re-enable if the IN
		 * pipe itself is ever suspect. 0=IDLE 1=DONE 2=NOTREADY 4=ERROR 5=STALL */
		if (us == USBH_URB_DONE) {
			uint32_t n = USBH_LL_GetLastXferSize(phost, g29.in_pipe);
			parse_report(g29.in_buf, (uint8_t)n);
			g29.in_st = IN_WAIT;
			g29.in_timer = HAL_GetTick();
		} else if (us == USBH_URB_NOTREADY) {
			/* device NAK'd (no new report yet): re-submit now, don't wait */
			g29.in_st = IN_GET;
		} else if (us == USBH_URB_STALL || us == USBH_URB_ERROR) {
			g29.in_st = IN_WAIT;
			g29.in_timer = HAL_GetTick();
		}
		break;
	}

	case IN_WAIT:
		if ((HAL_GetTick() - g29.in_timer) >= g29.poll_ms) {
			g29.in_st = IN_GET;
		}
		break;
	}

	/* ---- interrupt OUT: send queued force feedback ---- */
	if (g29.out_pipe) {
		switch (g29.out_st) {
		case OUT_IDLE:
			if (g29.ffb_pending) {
				USBH_InterruptSendData(phost, g29.ffb_buf,
						       sizeof(g29.ffb_buf), g29.out_pipe);
				g29.ffb_pending = false;
				g29.out_st = OUT_BUSY;
			}
			break;

		case OUT_BUSY: {
			USBH_URBStateTypeDef s = USBH_LL_GetURBState(phost, g29.out_pipe);

			if (s == USBH_URB_DONE) {
				g29.ffb_sent++;
				g29.out_st = OUT_IDLE;
			} else if (s == USBH_URB_NOTREADY) {
				/*
				 * Device NAK'd. Re-submit the same report, exactly as the
				 * IN path re-arms on NAK — ffb_buf still holds it. Without
				 * this the first NAK wedges OUT_BUSY forever and force
				 * feedback silently stops for good.
				 */
				USBH_InterruptSendData(phost, g29.ffb_buf,
						       sizeof(g29.ffb_buf), g29.out_pipe);
				g29.ffb_nak++;
			} else if (s == USBH_URB_STALL || s == USBH_URB_ERROR) {
				g29.ffb_err++;
				g29.out_st = OUT_IDLE;
			}
			break;
		}
		}
	}
	return USBH_OK;
}

static USBH_StatusTypeDef g29_SOFProcess(USBH_HandleTypeDef *phost)
{
	(void)phost;
	return USBH_OK;
}

USBH_ClassTypeDef G29_HID_Class = {
	"G29",
	0x03U,             /* HID class code */
	g29_InterfaceInit,
	g29_InterfaceDeInit,
	g29_ClassRequest,
	g29_Process,
	g29_SOFProcess,
	NULL,
};

/* ── Public API ─────────────────────────────────────────────────────────── */
bool g29_is_ready(void)
{
	return g29.ready;
}

int g29_get_state(struct g29_state *state)
{
	if (!g29.ready) {
		return -1;
	}
	/*
	 * parse_report() writes this from the USB task while the control task
	 * reads it here. The struct is 12 bytes, so an unguarded copy can be
	 * preempted half way and hand back a steering value from one report with
	 * pedals from the next — which the velocity estimator would see as a
	 * large phantom step. A few hundred nanoseconds of lockout is cheaper
	 * than reasoning about that.
	 */
	taskENTER_CRITICAL();
	*state = g29.state;
	taskEXIT_CRITICAL();
	return 0;
}

void g29_ffb_stats(uint32_t *sent, uint32_t *nak, uint32_t *err)
{
	*sent = g29.ffb_sent;
	*nak  = g29.ffb_nak;
	*err  = g29.ffb_err;
}

/* Queue a 7-byte FFB report (sent on the next host tick). */
static int queue_ffb(const uint8_t *cmd)
{
	if (!g29.ready || g29.out_pipe == 0U) {
		return -1;
	}
	memcpy(g29.ffb_buf, cmd, sizeof(g29.ffb_buf));
	g29.ffb_pending = true;
	return 0;
}

/*
 * Force type in byte 1: 0x00 = "constant", understood by every Logitech wheel
 * including the C294 compat descriptor; 0x08 = "variable", what lg4ff uses on
 * G25-and-later in native mode. In C294 the 0x08 form is accepted and echoed
 * back but drives no motor, so default to the classic one here.
 */
#define FFB_FORCE_TYPE 0x00

int g29_send_constant_force(int16_t value)
{
	/*
	 * The wheel's torque axis runs OPPOSITE to its steering-report axis:
	 * measured by step response, a positive command drove the reported
	 * steering count down, every time, at every force level.
	 *
	 * Inverting here makes the whole API mean one thing — "positive force
	 * pushes the steering reading positive". Without it `-vel * gain` is
	 * positive feedback rather than damping, which is what made the damper
	 * self-oscillate at higher gains.
	 */
	value = (int16_t)-value;

	/* lg4ff: byte2 is an 8-bit level, 0x80 = no force, 0x00/0xFF = full. */
	int level = (value >> 8) + 0x80;
	if (level < 0)    { level = 0; }
	if (level > 0xFF) { level = 0xFF; }
	uint8_t cmd[7] = { 0x11, FFB_FORCE_TYPE, (uint8_t)level,
			   0x80, 0x00, 0x00, 0x00 };
	return queue_ffb(cmd);
}

int g29_send_autocenter(uint8_t strength, uint8_t rate)
{
	/* lg4ff semantics: strength 0 means de-activate, not "coefficient 0".
	 * Setting the coefficient to zero leaves the default spring engaged and
	 * the wheel still creeps back to centre — it has to be switched off. */
	if (strength == 0U) {
		return queue_ffb(cmd_spring_off);
	}

	strength &= 0x0FU;
	uint8_t cmd[7] = { 0xfe, 0x0d, strength, strength, rate, 0x00, 0x00 };
	return queue_ffb(cmd);
}

int g29_send_range(uint16_t degrees)
{
	if (degrees < 40U)  { degrees = 40U; }
	if (degrees > 900U) { degrees = 900U; }
	uint8_t cmd[7] = { 0xf8, 0x81, (uint8_t)(degrees & 0xFF),
			   (uint8_t)(degrees >> 8), 0x00, 0x00, 0x00 };
	return queue_ffb(cmd);
}

int g29_send_no_effect(void)
{
	return queue_ffb(cmd_no_effect);
}

int g29_send_raw(const uint8_t cmd[7])
{
	return queue_ffb(cmd);
}
