#ifndef MCP2515_H
#define MCP2515_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
/*
 * Minimal MCP2515 SPI-to-CAN driver for the STM32 BlackPill.
 *
 * The STM32F401 has no bxCAN peripheral, so this is the only way to get CAN
 * off this chip.  Standard 11-bit IDs only, one TX buffer, one filtered RX
 * buffer — enough for cyclic broadcast of small fixed-layout frames, which is
 * how a real automotive sensor node behaves.
 *
 * Wiring: PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS, PB0 INT (active low).
 */

struct can_frame {
	uint16_t id;        /* 11-bit standard identifier */
	uint8_t  dlc;       /* 0..8 */
	uint8_t  data[8];
};

/* Reset + configure the controller. 0 = ok, -1 = chip not responding. */
int mcp2515_init(void);

/* Queue a frame into TXB0. Returns -1 if TXB0 is still busy (frame dropped). */
int mcp2515_send(const struct can_frame *f);

/* True when the INT pin is asserted, i.e. a filtered frame is waiting. */
bool mcp2515_rx_pending(void);

/* Pop a frame from RXB0. Returns 0 on success, -1 if nothing is waiting. */
int mcp2515_recv(struct can_frame *f);

#endif /* MCP2515_H */
