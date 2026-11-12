/* SPDX-License-Identifier: MIT */
/* Startup establishes safe GPIO inputs and the configured clock before USB. */
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "timing.h"
#include "tusb.h"
#include "usb_control.h"

int main(void) {
    /* board_init enables board-default LED/UART pins; intentionally do not call it. */
    for (unsigned pin = 0; pin < NUM_BANK0_GPIOS; ++pin) {
        gpio_init(pin);
        gpio_disable_pulls(pin);
    }
    bool clock_valid = GLITCH_SYS_CLOCK_KHZ > 0 && GLITCH_SYS_CLOCK_KHZ <= UINT32_MAX / 1000u &&
                       set_sys_clock_khz(GLITCH_SYS_CLOCK_KHZ, false);
    timing_init();
    usb_control_init(clock_valid);
    tusb_init();
    for (;;) {
        tud_task();
        usb_control_task();
    }
}
