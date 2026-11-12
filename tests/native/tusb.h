#pragma once
#include <stdbool.h>
#include <stdint.h>
bool tusb_init(void);
void tud_task(void);
void tud_cdc_read_flush(void);
uint32_t tud_cdc_write(const void *data, uint32_t length);
uint32_t tud_cdc_write_flush(void);
uint32_t tud_cdc_available(void);
uint32_t tud_cdc_read(void *data, uint32_t length);
