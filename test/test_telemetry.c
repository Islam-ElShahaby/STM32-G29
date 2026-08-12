/*
 * Host-buildable self-check for the 0x0A3 lights frame in src/telemetry.c.
 * Not part of the firmware build (PlatformIO only compiles src/); run directly:
 *
 *   gcc -I include -I test/stub src/telemetry.c test/test_telemetry.c \
 *       -o /tmp/tel_test && /tmp/tel_test
 */
#include <assert.h>
#include <stdio.h>
#include "telemetry.h"
#include "mcp2515.h"
#include "powertrain.h"

#define BTN_HIGH  0x00000400U
#define BTN_LOW   0x00000800U
#define BTN_LEFT  0x00000040U
#define BTN_RIGHT 0x00000080U

/* --- stubs for everything telemetry.c links against --- */
static uint32_t fake_tick;
uint32_t HAL_GetTick(void) { return fake_tick; }
void powertrain_get_state(struct powertrain_state *s) { (void)s; }
char shifter_letter(enum shifter_mode m) { (void)m; return 'D'; }

static struct can_frame last_frame;
static unsigned tx_count;
int mcp2515_send(const struct can_frame *f) { last_frame = *f; tx_count++; return 0; }

/* One poll tick of can_task: 10 ms apart, same as main.c. */
static void poll(uint32_t buttons)
{
	fake_tick += 10;
	can_lights_update(buttons);
}

int main(void)
{
	unsigned before;

	/* First call always transmits, so the bus has a known state. */
	poll(0);
	assert(tx_count == 1);
	assert(last_frame.id == 0x0A3U && last_frame.dlc == 4);

	/* A press latches the lamp ON and sends immediately. */
	before = tx_count;
	poll(BTN_HIGH);
	assert(tx_count == before + 1);
	assert(last_frame.data[0] == 1);

	/* Holding it does NOT re-toggle: no rising edge, no extra frame until
	 * the 100 ms heartbeat comes round. This is the whole point of the
	 * edge detection -- level mapping would flip the lamp every poll. */
	before = tx_count;
	for (int i = 0; i < 5; i++) {   /* 50 ms of hold */
		poll(BTN_HIGH);
	}
	assert(tx_count == before);
	assert(last_frame.data[0] == 1);

	/* Release: still no toggle, lamp stays latched. */
	poll(0);
	assert(last_frame.data[0] == 1);

	/* Second press clears it. */
	poll(BTN_HIGH);
	assert(last_frame.data[0] == 0);
	poll(0);

	/* Each lamp is independent and lands in its own byte. */
	poll(BTN_LOW);   assert(last_frame.data[1] == 1);
	poll(0);
	poll(BTN_LEFT);  assert(last_frame.data[2] == 1);
	poll(0);
	poll(BTN_RIGHT); assert(last_frame.data[3] == 1);
	poll(0);
	assert(last_frame.data[0] == 0 && last_frame.data[1] == 1 &&
	       last_frame.data[2] == 1 && last_frame.data[3] == 1);

	/* Two buttons pressed in the same report both toggle. */
	poll(BTN_LEFT | BTN_RIGHT);
	assert(last_frame.data[2] == 0 && last_frame.data[3] == 0);
	poll(0);

	/* Heartbeat: idle, it still transmits at 100 ms and no faster. */
	before = tx_count;
	for (int i = 0; i < 10; i++) {  /* 100 ms of idle polls */
		poll(0);
	}
	assert(tx_count == before + 1);

	printf("test_telemetry: all lights-frame checks passed\n");
	return 0;
}
