#include "printing.h"

void Debug_SendString(const char* message)
{
    if (message == NULL) return;
    CDC_Transmit_FS((uint8_t*)message, strlen(message));
}

void Debug_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
}

void USB_CDC_SendString(const char* message)
{
    Debug_SendString(message);
}

void USB_CDC_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
}
