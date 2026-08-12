#ifndef G29_HID_H
#define G29_HID_H

#include <stdbool.h>
#include <stdint.h>
#include "usbh_core.h"

/*
 * Custom USB-host class for the Logitech G29 Driving Force wheel.
 *
 * ST's stock HID class (usbh_hid.c) refuses any HID interface that is not a
 * boot-protocol mouse/keyboard, so a wheel never initializes through it.
 * This class accepts the generic HID interface, polls the interrupt-IN
 * endpoint for input reports, and sends force-feedback via the interrupt-OUT
 * endpoint.
 *
 * Register it from MX_USB_HOST_Init():
 *     USBH_RegisterClass(&hUsbHostFS, &G29_HID_Class);
 */
extern USBH_ClassTypeDef G29_HID_Class;

struct g29_state {
	uint16_t steering;  /* 0x0000 full-left … 0xFFFF full-right, center ≈ 0x7FFF */
	uint8_t  throttle;  /* 0 released … 0xFF fully pressed */
	uint8_t  brake;
	uint8_t  clutch;
	uint32_t buttons;
};

/* True once the wheel is enumerated and reports are flowing. */
bool g29_is_ready(void);

/* Copy the most recent decoded report. Returns -1 if not ready. */
int g29_get_state(struct g29_state *state);

/*
 * Health of the interrupt-OUT (force feedback) endpoint. If `sent` never rises
 * the wheel is not receiving FFB at all, and no amount of tuning the effect
 * will change that — look at the endpoint or the wheel's mode instead.
 */
void g29_ffb_stats(uint32_t *sent, uint32_t *nak, uint32_t *err);

/*
 * Force-feedback commands. They queue a 7-byte report that is sent on the
 * next USB host process tick. Return -1 if the wheel is not ready.
 *
 *   constant:    value -32767 (full-left torque) … +32767 (full-right torque)
 *   autocenter:  strength 0 … 15, rate 0 … 255. Strength 0 de-activates the
 *                wheel's default centring spring, which is ON at power-up —
 *                send it once if you want the wheel to hold position.
 *   range:       degrees 40 … 900 (physical lock-to-lock)
 */
int g29_send_constant_force(int16_t value);
int g29_send_autocenter(uint8_t strength, uint8_t rate);
int g29_send_range(uint16_t degrees);
int g29_send_no_effect(void);

/* Queue an arbitrary 7-byte Logitech FFB report. For protocol bring-up. */
int g29_send_raw(const uint8_t cmd[7]);

#endif /* G29_HID_H */
