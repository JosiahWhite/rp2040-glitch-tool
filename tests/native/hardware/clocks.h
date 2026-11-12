#pragma once
#include <stdbool.h>
#include <stdint.h>
enum { clk_sys };
uint32_t clock_get_hz(int clock);
bool set_sys_clock_khz(uint32_t khz, bool required);
