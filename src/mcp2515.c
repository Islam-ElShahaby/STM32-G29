/*
 * MCP2515 SPI-to-CAN driver — standard 11-bit IDs, 500 kbps.
 *
 * Reference: Microchip MCP2515 datasheet (DS20001801), SPI instruction set in
 * section 12 and register map in section 11.
 *
 * Scope is deliberately small: one TX buffer, one hardware-filtered RX buffer,
 * blocking SPI.  That is all a cyclic-broadcast sensor node needs.
 */
#include "mcp2515.h"


/* ── Build-time configuration ───────────────────────────────────────────── */

/*
 * Crystal on the MCP2515 module — read the marking on the metal can.
 * Getting this wrong makes every frame come out at exactly half or double the
 * intended bitrate, which is the easiest fault in this file to spot on a
 * logic analyzer.
 */
#define MCP_XTAL_8MHZ  1        

/*
 * Loopback: the controller receives its own transmissions and self-ACKs, so the
 * RX path can be proven with no second node on the bus.  Set 1 for bring-up.
 */
#define MCP_LOOPBACK   0

/* ── SPI instructions ───────────────────────────────────────────────────── */
#define CMD_RESET      0xC0U
#define CMD_READ       0x03U
#define CMD_WRITE      0x02U
#define CMD_LOAD_TX0   0x40U    /* load TXB0 starting at TXB0SIDH */
#define CMD_RTS_TX0    0x81U    /* request-to-send, TXB0 */
#define CMD_READ_RX0   0x90U    /* read RXB0 from SIDH, auto-clears RX0IF */

/* ── Registers ──────────────────────────────────────────────────────────── */
#define REG_RXF0SIDH   0x00U
#define REG_CANSTAT    0x0EU
#define REG_CANCTRL    0x0FU
#define REG_RXM0SIDH   0x20U
#define REG_CNF3       0x28U    /* CNF3, CNF2, CNF1 are consecutive ascending */
#define REG_CNF1       0x2AU
#define REG_CANINTE    0x2BU
#define REG_CANINTF    0x2CU
#define REG_TXB0CTRL   0x30U
#define REG_RXB0CTRL   0x60U

#define TXB0CTRL_TXREQ 0x08U
#define CANSTAT_OPMOD  0xE0U
#define OPMOD_CONFIG   0x80U

/* Normal mode + one-shot. One-shot matters when no other node ACKs: without it
 * an unACKed frame retransmits forever and blocks everything behind it. */
#define CANCTRL_RUN    (MCP_LOOPBACK ? 0x48U : 0x08U)

#if MCP_XTAL_8MHZ
#define CNF1_500K      0x00U
#define CNF2_500K      0x90U
#define CNF3_500K      0x02U
#else
#define CNF1_500K      0x00U
#define CNF2_500K      0xF0U
#define CNF3_500K      0x86U
#endif

/* ── Standard-ID packing ────────────────────────────────────────────────── */
#define SIDH(id)       ((uint8_t)((id) >> 3))
#define SIDL(id)       ((uint8_t)(((id) & 0x07U) << 5))
#define UNPACK_ID(h, l) ((uint16_t)(((uint16_t)(h) << 3) | ((l) >> 5)))

/* The only non-trivial pure logic here; constant-folds, so a break fails the
 * build rather than showing up as garbled IDs on the analyzer. */
_Static_assert(UNPACK_ID(SIDH(0x7FFU), SIDL(0x7FFU)) == 0x7FFU, "ID pack broken");
_Static_assert(UNPACK_ID(SIDH(0x0A0U), SIDL(0x0A0U)) == 0x0A0U, "ID pack broken");
_Static_assert(UNPACK_ID(SIDH(0x0B0U), SIDL(0x0B0U)) == 0x0B0U, "ID pack broken");

/* ── Pins ───────────────────────────────────────────────────────────────── */
#define CS_PORT        GPIOB
#define CS_PIN         GPIO_PIN_6
#define INT_PORT       GPIOB
#define INT_PIN        GPIO_PIN_0

static SPI_HandleTypeDef hspi1;

static inline void cs_low(void)  { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET); }

/* ── SPI primitives ─────────────────────────────────────────────────────── */
static void xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	cs_low();
	if (rx) {
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100);
	} else {
		HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, 100);
	}
	cs_high();
}

static uint8_t reg_read(uint8_t addr)
{
	uint8_t tx[3] = { CMD_READ, addr, 0x00 };
	uint8_t rx[3] = { 0 };

	xfer(tx, rx, sizeof(tx));
	return rx[2];
}

static void reg_write(uint8_t addr, uint8_t val)
{
	uint8_t tx[3] = { CMD_WRITE, addr, val };

	xfer(tx, NULL, sizeof(tx));
}

/* Burst write to consecutive registers starting at addr. */
static void reg_write_n(uint8_t addr, const uint8_t *vals, uint8_t n)
{
	uint8_t tx[8];

	tx[0] = CMD_WRITE;
	tx[1] = addr;
	for (uint8_t i = 0; i < n; i++) {
		tx[2 + i] = vals[i];
	}
	xfer(tx, NULL, (uint16_t)(2 + n));
}

/* ── Hardware bring-up ──────────────────────────────────────────────────── */
static void spi_gpio_init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();

	
	/* PB3 SCK, PB4 MISO, PB5 MOSI */
	g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOB, &g);

	/* PA4 CS — idle high, driven before the peripheral is enabled */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	g.Pin = CS_PIN;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Alternate = 0;
	HAL_GPIO_Init(CS_PORT, &g);

	/* PB0 INT — open-drain on the MCP2515, active low */
	g.Pin = INT_PIN;
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(INT_PORT, &g);

	/* Mode 0 (CPOL=0, CPHA=0). PCLK2 84 MHz / 16 = 5.25 MHz, MCP2515 max 10. */
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	HAL_SPI_Init(&hspi1);
}

int mcp2515_init(void)
{
	uint8_t reset = CMD_RESET;
	const uint8_t cnf[3] = { CNF3_500K, CNF2_500K, CNF1_500K };
	const uint8_t mask[2] = { SIDH(0x7FFU), SIDL(0x7FFU) };   /* exact match */
	const uint8_t filt[2] = { SIDH(0x0B0U), SIDL(0x0B0U) };   /* FFB frame */

	spi_gpio_init();

	xfer(&reset, NULL, 1);
	HAL_Delay(10);

	/* A reset MCP2515 must land in config mode. Anything else means the chip
	 * is not answering — bad wiring, no power, or CS on the wrong pin. */
	if ((reg_read(REG_CANSTAT) & CANSTAT_OPMOD) != OPMOD_CONFIG) {
		return -1;
	}

	reg_write_n(REG_CNF3, cnf, sizeof(cnf));

	/* Prove MOSI and MISO both work before trusting anything downstream. */
	if (reg_read(REG_CNF1) != CNF1_500K) {
		return -1;
	}

	/*
	 * Hardware acceptance filter. The MCP2515 has only two RX buffers; left
	 * unfiltered on a real vehicle bus (~1500 frames/s) they overflow within
	 * milliseconds. Accept exactly one ID and let the silicon drop the rest.
	 */
	reg_write_n(REG_RXM0SIDH, mask, sizeof(mask));
	reg_write_n(REG_RXF0SIDH, filt, sizeof(filt));
	reg_write(REG_RXB0CTRL, 0x00U);   /* filters on, no rollover to RXB1 */

	reg_write(REG_CANINTF, 0x00U);
	reg_write(REG_CANINTE, 0x01U);    /* RX0 only, so INT means one thing */

	reg_write(REG_CANCTRL, CANCTRL_RUN);
	return 0;
}

/* ── Traffic ────────────────────────────────────────────────────────────── */
/*
 * Bound on how long mcp2515_send() will wait for TXB0 to free up before
 * giving up. At 500 kbps a worst-case 8-byte standard frame (with maximal
 * bit-stuffing) is well under 1 ms on the wire; 2 ms is generous margin
 * without risking a real stall — see the note on mcp2515_send() below for
 * why this exists at all.
 */
#define MCP_SEND_BUSY_TIMEOUT_MS 2U

/*
 * The MCP2515 has exactly one TX buffer (TXB0) -- this driver was written
 * for a single cyclic sender and dropped on first sight of TXREQ still set,
 * which was correct when there was only one caller. With a second cyclic
 * sender now calling this back-to-back with the first in the same tick
 * (can_telemetry_update() then can_lights_update(), no gap between them),
 * the second call was seeing TXREQ still set from the first transmission
 * and silently dropping EVERY time -- not occasionally, every single tick,
 * because there is genuinely zero time between the two calls for TXB0 to
 * clear. Confirmed two ways: disabling the first sender let the second
 * transmit cleanly, and renaming the second frame's CAN ID changed nothing
 * (ruling out anything ID-specific and confirming it's purely this race).
 *
 * Fix: retry for a short bounded window instead of dropping on the first
 * busy check. Still drops (same -1 return, same "next one is coming
 * anyway" reasoning) if TXB0 is STILL busy after the timeout -- that means
 * something is actually wrong (a wedged bus, no ACKing peer under normal
 * one-shot mode), not just two senders racing a few microseconds apart,
 * and the driver should fail closed exactly as it always has rather than
 * block can_task() indefinitely.
 */
int mcp2515_send(const struct can_frame *f)
{
	uint8_t tx[14];
	uint8_t rts = CMD_RTS_TX0;
	uint32_t t0 = HAL_GetTick();

	while (reg_read(REG_TXB0CTRL) & TXB0CTRL_TXREQ) {
		if (HAL_GetTick() - t0 >= MCP_SEND_BUSY_TIMEOUT_MS) {
			/* Still sending after the whole timeout -- something is
			 * actually wrong, not just two cyclic senders racing.
			 * Dropping is still correct for cyclic data: the next
			 * one is coming shortly and carries fresher truth. */
			return -1;
		}
	}

	tx[0] = CMD_LOAD_TX0;
	tx[1] = SIDH(f->id);
	tx[2] = SIDL(f->id);
	tx[3] = 0x00;            /* EID8 — standard ID, unused */
	tx[4] = 0x00;            /* EID0 */
	tx[5] = f->dlc;
	for (uint8_t i = 0; i < f->dlc; i++) {
		tx[6 + i] = f->data[i];
	}
	xfer(tx, NULL, (uint16_t)(6 + f->dlc));

	xfer(&rts, NULL, 1);
	return 0;
}

bool mcp2515_rx_pending(void)
{
	return HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == GPIO_PIN_RESET;
}

int mcp2515_recv(struct can_frame *f)
{
	uint8_t tx[14] = { CMD_READ_RX0 };
	uint8_t rx[14] = { 0 };

	if (!mcp2515_rx_pending()) {
		return -1;
	}

	/* READ RX BUFFER streams SIDH, SIDL, EID8, EID0, DLC, D0..D7 and clears
	 * RX0IF on CS rising — no separate interrupt-flag write needed. */
	xfer(tx, rx, sizeof(tx));

	f->id = UNPACK_ID(rx[1], rx[2]);
	f->dlc = rx[5] & 0x0FU;
	if (f->dlc > 8U) {
		f->dlc = 8U;
	}
	for (uint8_t i = 0; i < f->dlc; i++) {
		f->data[i] = rx[6 + i];
	}
	return 0;
}
