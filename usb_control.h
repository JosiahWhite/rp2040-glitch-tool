/* SPDX-License-Identifier: MIT */
#ifndef GLITCH_USB_CONTROL_H
#define GLITCH_USB_CONTROL_H

#include <stdbool.h>

/* Call once on core 0 after timing_init(), before servicing USB. */
void usb_control_init(bool clock_valid);
/* Core-0 session service; the caller runs tud_task() first. */
void usb_control_task(void);

#endif
