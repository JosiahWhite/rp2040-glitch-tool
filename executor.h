#ifndef GLITCH_EXECUTOR_H
#define GLITCH_EXECUTOR_H

#include "sequence.h"
#include "timing.h"

typedef struct {
    sequence_status_t sequence;
    timing_pwm_t      pwm;
} executor_status_t;

/* Core0 is the only caller. Initialize after timing_init()/sequence_clear(). */
void executor_init(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]);
/* A successful acquire parks core1 until release, transferring exclusive access
 * to staging/configuration and direct-transfer buffers. Active work refuses it.
 * Do not request status/abort or acquire again while holding this ownership. */
bool executor_acquire(void);
void executor_release(void);
/* Advisory only; acquire is the authoritative ownership check. */
bool executor_busy(void);
/* Copy at a worker service boundary; run_elapsed is frozen even while running.
 * Core1 resumes immediately, without waiting for core0 to format the copy. */
void executor_status(executor_status_t *out);
/* Returns only after the worker has stopped PIO/DMA/controller use of buffers. */
void executor_abort(void);

#endif
