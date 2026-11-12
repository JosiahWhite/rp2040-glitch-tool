#ifndef GLITCH_SEQUENCE_H
#define GLITCH_SEQUENCE_H

#include "i2c_bus.h"
#include "protocol.h"
#include "timing.h"

#define SEQUENCE_STEP_LIMIT    64u
#define SEQUENCE_PAYLOAD_LIMIT 8192u
#define SEQUENCE_ROUTINE_BYTES 256u

typedef enum {
    SEQUENCE_IDLE,
    SEQUENCE_RUNNING,
    SEQUENCE_DONE,
    SEQUENCE_ERROR,
    SEQUENCE_ABORTED
} sequence_state_t;

/* Worker-owned run metadata; core0 reads only executor snapshots while active. */
typedef struct {
    sequence_state_t state;
    unsigned         current_step; /* Foreground cursor; failing WriteAt index on group errors. */
    const char      *run_error;
    uint32_t         error_detail;
    uint64_t         run_start, run_elapsed;
    uint8_t          poll_data[SEQUENCE_ROUTINE_BYTES];
    size_t           poll_length;
    uint32_t         poll_attempts;
    const char      *last_poll_error;
    bool             matched, hold_final;
    timing_trigger_t matched_trigger;
    unsigned         compiled_tokens, compiled_pwm_words;
    uint64_t         programmed_cycles, inserted_cycles;
} sequence_status_t;

bool                     sequence_running(void);
const sequence_status_t *sequence_status(void);
const char              *sequence_state_name(sequence_state_t state);

/* Core0 stages/starts only while executor_acquire() holds the worker parked.
 * Core1 exclusively services/aborts active execution. Clear preserves held pins.
 * Scratch and receive storage are shared with direct transfers; ownership stays
 * with the worker until completion or acknowledged cancellation.
 * Consecutive WriteAt declarations share the preceding command's completion.
 * SyncWrites, native Write/Poll, and run completion join the pending group. */
void        sequence_clear(void);
const char *sequence_stage(const request_t *request, uint8_t scratch[I2C_TRANSFER_LIMIT]);
const char *sequence_start(uint32_t timeout_us, bool hold_final, uint32_t occupied_pins);
void        sequence_service(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]);
void        sequence_abort(void);

#endif
