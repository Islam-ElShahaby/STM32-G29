/**
 * Low-level USB Host glue: binds the ST USBH core to the STM32 HAL HCD driver
 * for the OTG-FS peripheral.  Equivalent to CubeMX-generated usbh_conf.c.
 *
 * Pins on the BlackPill (fixed by the OTG-FS peripheral):
 *   PA11 = USB D-      PA12 = USB D+      (AF10)
 * Wire these to a USB-A socket; power the socket's VBUS from an external 5 V
 * supply that can source >=500 mA (the G29 pulls ~400 mA).  Tie grounds.
 */
#include "usbh_core.h"

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* ===== HAL MSP (clocks, GPIO, NVIC) ====================================== */

void HAL_HCD_MspInit(HCD_HandleTypeDef *hcdHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (hcdHandle->Instance == USB_OTG_FS) {
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* PA11 (D-) and PA12 (D+) as alternate-function OTG-FS */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		__HAL_RCC_USB_OTG_FS_CLK_ENABLE();

		HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	}
}

void HAL_HCD_MspDeInit(HCD_HandleTypeDef *hcdHandle)
{
	if (hcdHandle->Instance == USB_OTG_FS) {
		__HAL_RCC_USB_OTG_FS_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
	}
}

/* ===== HAL HCD callbacks -> USBH core =================================== */

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
	USBH_LL_IncTimer((USBH_HandleTypeDef *)hhcd->pData);
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
	USBH_LL_Connect((USBH_HandleTypeDef *)hhcd->pData);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
	USBH_LL_Disconnect((USBH_HandleTypeDef *)hhcd->pData);
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
	USBH_LL_PortEnabled((USBH_HandleTypeDef *)hhcd->pData);
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
	USBH_LL_PortDisabled((USBH_HandleTypeDef *)hhcd->pData);
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
					 HCD_URBStateTypeDef urb_state)
{
	/* Polled via USBH_LL_GetURBState — nothing to do here */
	(void)hhcd;
	(void)chnum;
	(void)urb_state;
}

/* ===== USBH core low-level driver (USBH_LL_*) =========================== */

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef *phost)
{
	hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hhcd_USB_OTG_FS.Init.Host_channels = 8;
	hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
	hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
	hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hhcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hhcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hhcd_USB_OTG_FS.Init.use_external_vbus = DISABLE;

	/* Cross-link core <-> HAL handles */
	hhcd_USB_OTG_FS.pData = phost;
	phost->pData = &hhcd_USB_OTG_FS;

	if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK) {
		return USBH_FAIL;
	}

	USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hhcd_USB_OTG_FS));
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_DeInit(USBH_HandleTypeDef *phost)
{
	HAL_HCD_DeInit(phost->pData);
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_Start(USBH_HandleTypeDef *phost)
{
	HAL_HCD_Start(phost->pData);
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_Stop(USBH_HandleTypeDef *phost)
{
	HAL_HCD_Stop(phost->pData);
	return USBH_OK;
}

USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef *phost)
{
	switch (HAL_HCD_GetCurrentSpeed(phost->pData)) {
	case 0:  return USBH_SPEED_HIGH;
	case 1:  return USBH_SPEED_FULL;
	case 2:  return USBH_SPEED_LOW;
	default: return USBH_SPEED_FULL;
	}
}

USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef *phost)
{
	HAL_HCD_ResetPort(phost->pData);
	return USBH_OK;
}

uint32_t USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	return HAL_HCD_HC_GetXferCount(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_OpenPipe(USBH_HandleTypeDef *phost, uint8_t pipe_num,
				    uint8_t epnum, uint8_t dev_address,
				    uint8_t speed, uint8_t ep_type, uint16_t mps)
{
	HAL_HCD_HC_Init(phost->pData, pipe_num, epnum, dev_address,
			speed, ep_type, mps);
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	HAL_HCD_HC_Halt(phost->pData, pipe);
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_SubmitURB(USBH_HandleTypeDef *phost, uint8_t pipe,
				     uint8_t direction, uint8_t ep_type,
				     uint8_t token, uint8_t *pbuff,
				     uint16_t length, uint8_t do_ping)
{
	HAL_HCD_HC_SubmitRequest(phost->pData, pipe, direction, ep_type,
				 token, pbuff, length, do_ping);
	return USBH_OK;
}

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	return (USBH_URBStateTypeDef)HAL_HCD_HC_GetURBState(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state)
{
	/* VBUS is supplied externally on the BlackPill — nothing to switch.
	 * If you add a load-switch on a GPIO, drive it here.
	 */
	(void)phost;
	(void)state;
	HAL_Delay(200);
	return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost, uint8_t pipe,
				     uint8_t toggle)
{
	HCD_HandleTypeDef *hhcd = phost->pData;

	if (hhcd->hc[pipe].ep_is_in) {
		hhcd->hc[pipe].toggle_in = toggle;
	} else {
		hhcd->hc[pipe].toggle_out = toggle;
	}
	return USBH_OK;
}

uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe)
{
	HCD_HandleTypeDef *hhcd = phost->pData;

	return hhcd->hc[pipe].ep_is_in ? hhcd->hc[pipe].toggle_in
				       : hhcd->hc[pipe].toggle_out;
}

void USBH_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}
