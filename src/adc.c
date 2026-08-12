#include "adc.h"

#define ADC_TIMEOUT_MS 2U

static ADC_HandleTypeDef hadc1;

void adc_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* PA0, PA1, PA2, PA3 */
    gpio.Pin = GPIO_PIN_0 |
               GPIO_PIN_1 |
               GPIO_PIN_2 |
               GPIO_PIN_3 |
               GPIO_PIN_4 ;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB1 */
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
     return;
    }
}

uint16_t adc_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}



    uint16_t adc_read_mv(uint32_t channel)
{
    uint32_t raw = adc_read_channel(channel);

    return (uint16_t)((raw * 3300U) / 4095U);
}
