#ifndef PRINTING_H
#define PRINTING_H

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

// Unified print functions (USB CDC)
void Debug_SendString(const char* message);
void Debug_Printf(const char *fmt, ...);

// Legacy alias used by message_center.c etc.
void USB_CDC_SendString(const char* message);
void USB_CDC_Printf(const char *fmt, ...);

#endif
