/**
 * USB host application glue — equivalent to CubeMX-generated usb_host.c, but
 * it registers our custom G29 class instead of the stock HID class.
 */
#include "usb_host.h"


USBH_HandleTypeDef hUsbHostFS;

static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
	switch (id) {
	case HOST_USER_CONNECTION:
		USBH_UsrLog("USB: device connected");
		break;
	case HOST_USER_DISCONNECTION:
		USBH_UsrLog("USB: device disconnected");
		break;
	case HOST_USER_CLASS_ACTIVE:
		USBH_UsrLog("USB: class active (G29 enumerated)");
		break;
	case HOST_USER_CLASS_SELECTED:
		USBH_UsrLog("USB: class selected");
		break;
	case HOST_USER_UNRECOVERED_ERROR:
		USBH_UsrLog("USB: unrecovered error");
		break;
	default:
		break;
	}
}

void MX_USB_HOST_Init(void)
{
	if (USBH_Init(&hUsbHostFS, USBH_UserProcess, 0U) != USBH_OK) {
		USBH_ErrLog("USBH_Init failed");
		return;
	}
	if (USBH_RegisterClass(&hUsbHostFS, &G29_HID_Class) != USBH_OK) {
		USBH_ErrLog("RegisterClass failed");
		return;
	}
	if (USBH_Start(&hUsbHostFS) != USBH_OK) {
		USBH_ErrLog("USBH_Start failed");
	}
}

void MX_USB_HOST_Process(void)
{
	USBH_Process(&hUsbHostFS);
}
