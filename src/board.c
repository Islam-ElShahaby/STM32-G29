/*
 * Board bring-up — see include/board.h.
 *
 * Split out of main.c: this is fixed, board-specific setup that is read once
 * and then never touched, unlike the control logic it used to sit above.
 */
#include "board.h"

UART_HandleTypeDef huart1;

/* ── Clock: 96 MHz core, 48 MHz USB ─────────────────────────────────────── */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef osc = {0};
	RCC_ClkInitTypeDef clk = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	/* F401 max 84 MHz -> Scale 2 (Scale 1 does not exist on F401) */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc.HSEState = RCC_HSE_ON;
	osc.PLL.PLLState = RCC_PLL_ON;
	osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	osc.PLL.PLLM = 25;            /* 25 MHz / 25 = 1 MHz PLL input */
	osc.PLL.PLLN = 336;           /* 1 MHz * 336 = 336 MHz VCO */
	osc.PLL.PLLP = RCC_PLLP_DIV4; /* 336 / 4 = 84 MHz SYSCLK (F401 max) */
	osc.PLL.PLLQ = 7;             /* 336 / 7 = 48 MHz USB clock */
	if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
		while (1) {}
	}

	clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* 84 MHz */
	clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* 42 MHz (APB1 max 42) */
	clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* 84 MHz (APB2 max 84) */
	if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
		while (1) {}
	}
}

/* ── GPIO: LED on PC13 ──────────────────────────────────────────────────── */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	g.Pin = GPIO_PIN_13;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &g);
}

/* ── USART1 (PA9 TX) for printf ─────────────────────────────────────────── */
static void MX_USART1_UART_Init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* PA9 = USART1_TX, PA10 = USART1_RX. Init.Mode was already TX_RX but PA10
	 * was never muxed, so receive silently did nothing. */
	g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &g);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	/* Interrupt-driven receive — see the console section for why polling the
	 * one-byte RX register from a scheduled task cannot work. Priority 6 is
	 * below the FreeRTOS syscall ceiling; the handler touches no kernel API
	 * anyway, it only fills a ring. */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}


/* ── DWT cycle counter: run-time-stats clock for the CPU load figure ─────── */
/*
 * Free timebase — no timer peripheral spent, and one register read per context
 * switch. It wraps every ~51 s at 84 MHz, which is nothing against the 1 s
 * window console_task() measures over (unsigned subtraction crosses the wrap).
 */
void cpu_cyccnt_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_init(void)
{
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART1_UART_Init();
}
