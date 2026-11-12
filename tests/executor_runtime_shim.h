#ifndef EXECUTOR_RUNTIME_SHIM_H
#define EXECUTOR_RUNTIME_SHIM_H

#include <stdint.h>

#define __not_in_flash_func(name) name
void multicore_launch_core1(void (*entry)(void));
void tight_loop_contents(void);
uint64_t time_us_64(void);

#endif
