#ifndef USB_HOST_H
#define USB_HOST_H

#include "usbh_core.h"
#include "g29_hid.h"
extern USBH_HandleTypeDef hUsbHostFS;

void MX_USB_HOST_Init(void);
void MX_USB_HOST_Process(void);

#endif /* USB_HOST_H */
