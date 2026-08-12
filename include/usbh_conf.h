/**
 * USB Host library configuration.  Equivalent to the CubeMX-generated
 * usbh_conf.h, hand-written here.
 */
#ifndef USBH_CONF_H
#define USBH_CONF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* ##### Library sizing ##### */
#define USBH_MAX_NUM_ENDPOINTS         3U
#define USBH_MAX_NUM_INTERFACES        3U
#define USBH_MAX_NUM_CONFIGURATION     1U
#define USBH_MAX_NUM_SUPPORTED_CLASS   1U
#define USBH_KEEP_CFG_DESCRIPTOR       1U
#define USBH_MAX_SIZE_CONFIGURATION    0x200U
#define USBH_MAX_DATA_BUFFER           0x200U
#define USBH_DEBUG_LEVEL               2U
#define USBH_USE_OS                    0U

/* ##### Memory & string ops ##### */
#define USBH_malloc        malloc
#define USBH_free          free
#define USBH_memset        memset
#define USBH_memcpy        memcpy

/* ##### Logging — routed to printf (retargeted to USART1 in main.c) ##### */
#if (USBH_DEBUG_LEVEL > 0U)
#define USBH_UsrLog(...)   do { printf(__VA_ARGS__); printf("\r\n"); } while (0)
#else
#define USBH_UsrLog(...)
#endif

#if (USBH_DEBUG_LEVEL > 1U)
#define USBH_ErrLog(...)   do { printf("ERR: "); printf(__VA_ARGS__); printf("\r\n"); } while (0)
#else
#define USBH_ErrLog(...)
#endif

#if (USBH_DEBUG_LEVEL > 2U)
#define USBH_DbgLog(...)   do { printf("DBG: "); printf(__VA_ARGS__); printf("\r\n"); } while (0)
#else
#define USBH_DbgLog(...)
#endif

#endif /* USBH_CONF_H */
