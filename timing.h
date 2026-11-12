#ifndef GLITCH_TIMING_H
#define GLITCH_TIMING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TIMING_MAX_TOKENS          1024u
#define TIMING_MAX_PWM_SEGMENTS    64u
#define TIMING_PWM_SEGMENT_WORDS   16u
#define TIMING_MAX_PWM_WORDS       (TIMING_MAX_PWM_SEGMENTS * TIMING_PWM_SEGMENT_WORDS)
#define TIMING_MIN_INTERVAL_CYCLES 8u
#define TIMING_MAX_COUNTDOWN       0x0fffffffu
#define TIMING_WRITE_LIMIT         2u
#define TIMING_WRITE_MIN_CYCLES    8u
#define TIMING_WRITE_READY_LEAD    4u

/* Absolute instruction addresses used by the pure PWM encoder. The hardware
 * runtime checks these against the assembled program before it can build. */
enum {
    TIMING_PWM_COMMAND_PC = 12u,
    TIMING_PWM_HOLD_PC    = 13u
};
/* Recognized synchronized SCL/SCK sampling edge to push-pull OUT.
 * Open-drain/open-source direction edges occur one cycle later.
 * An asynchronous edge also incurs synchronizer/sampling uncertainty. */
#define TIMING_I2C_MATCH_LATENCY_CYCLES 11u
#define TIMING_SPI_MATCH_LATENCY_CYCLES 11u
/* Synchronized GPIO edge to push-pull OUT, excluding input sampling. */
#define TIMING_GPIO_MATCH_LATENCY_CYCLES 7u
#define TIMING_INPUT_UNCERTAINTY_CYCLES  2u
#define TIMING_SPI_CS_SETUP_CYCLES       8u
#define TIMING_SPI_CS_HOLD_CYCLES        8u
#define TIMING_SPI_CS_GAP_CYCLES         64u

typedef enum {
    TIMING_PUSH_PULL,
    TIMING_OPEN_DRAIN,
    TIMING_OPEN_SOURCE
} timing_drive_t;

typedef enum {
    TIMING_PIN_LOW      = 0,
    TIMING_PIN_HIGH     = 1,
    TIMING_PIN_FLOATING = 2
} timing_pin_state_t;

typedef struct {
    uint32_t configured_mask, open_drain_mask, open_source_mask, idle_high_mask;
    uint8_t  out_base;
    uint32_t current_levels, current_dirs; /* Physical, unrotated, last completed state. */
} timing_gpio_t;

typedef struct {
    bool     configured;
    uint8_t  cs, sck, data, mode;
    bool     lsb_first, cs_active_low;
    uint32_t max_hz;
} timing_spi_t;

typedef enum {
    TIMING_PWM_CARRIER  = 0,
    TIMING_PWM_LOW      = 1,
    TIMING_PWM_HIGH     = 2,
    TIMING_PWM_FLOATING = 3
} timing_pwm_mode_t;

typedef struct {
    bool              configured, active;
    uint8_t           pin;
    uint32_t          hz; /* Requested frequency; actual frequency is clk_sys / period_cycles. */
    uint32_t          period_cycles;
    uint32_t          final_duty_basis_points; /* Target final duty, not instantaneous duty. */
    timing_pwm_mode_t final_mode;
} timing_pwm_t;

typedef enum {
    TIMING_DELAY,
    TIMING_PULSE,
    TIMING_WAIT,
    TIMING_PWM,
    TIMING_SET
} timing_op_t;

typedef struct {
    timing_op_t        op;
    uint32_t           cycles;
    uint32_t           duty_basis_points;
    uint8_t            pin;
    timing_pin_state_t state;
} timing_instruction_t;

/* Storage must be 32-byte aligned; the 64-byte stride preserves DMA read rings.
 * Only words[0..word_count) are streamed. periods * word_count must fit uint32_t. */
typedef struct {
    uint32_t          words[8], periods, word_count;
    timing_pwm_mode_t mode;
    uint32_t          duty_basis_points;
    uint32_t          padding[4];
} timing_pwm_segment_t;

typedef struct {
    const uint32_t *tokens; /* Three-word frames; immutable until block completes. */
    size_t          count;
    bool            waits;
    uint64_t        programmed_cycles; /* Requested plus inserted cycles; excludes variable wait. */
    const timing_pwm_segment_t *pwm_segments; /* Immutable until block completes. */
    size_t                      pwm_count;    /* Segments, not FIFO words. */
    uint32_t                    pwm_final_duty;
    uint32_t          wave_start_countdown; /* Zero normally; 1/2 for synchronized PWM warmup. */
    uint64_t          inserted_cycles;
    uint32_t          final_levels, final_dirs;
    timing_pwm_mode_t pwm_final_mode;
    /* Isolated Delay/Set/Pwm command boundary from synchronous SM enable;
     * zero for blocks with no fixed scheduling origin. Cycle zero is first NOP. */
    uint64_t completion_cycles;
    /* Prepared once per run; no descriptor construction on the execution path. */
    uint16_t pwm_descriptor, pwm_tail_descriptor;
    uint32_t pwm_prefill[8];
} timing_block_t;

typedef enum {
    TIMING_TRIGGER_NONE,
    TIMING_TRIGGER_I2C,
    TIMING_TRIGGER_SPI,
    TIMING_TRIGGER_GPIO
} timing_trigger_kind_t;

typedef struct {
    timing_trigger_kind_t kind;
    uint8_t               sda, scl, address;
    bool                  read;
    uint8_t               prefix[4];
    size_t                prefix_length;
    uint8_t               pin;
    bool                  rising;
} timing_trigger_t;

typedef struct {
    bool        active, done, matched, waiting;
    const char *error;
} timing_status_t;

/* Hardware-independent compilation/encoding in timing.c; no Pico SDK needed.
 * Caller-owned output buffers are bounded by capacity and the limits above. */
const char *timing_compile(const timing_gpio_t *gpio, const timing_instruction_t *instructions,
                           size_t count, uint32_t *tokens, size_t capacity, timing_block_t *block);
/* PWM changes are on integer period boundaries. A PWM-containing block cannot
 * contain WAIT; starting each such block restarts the carrier phase. */
const char *timing_compile_pwm(const timing_gpio_t *gpio, const timing_pwm_t *pwm,
                               const timing_instruction_t *instructions, size_t count,
                               uint32_t initial_duty, uint32_t *tokens, size_t capacity,
                               timing_pwm_segment_t *pwm_segments, size_t pwm_capacity,
                               timing_block_t *block);
const char *timing_pwm_parameters(uint32_t sys_hz, uint32_t hz, uint32_t *period_cycles);
/* Nearest-half-up duty rounding; unsupported short high/low phases are rejected.
 * 0/100% are constant levels, encoded with an internal valid phase split. */
const char *timing_pwm_encode(uint32_t period_cycles, uint32_t duty_basis_points, uint32_t words[8],
                              uint32_t *word_count, uint32_t *high_cycles);
const char *timing_pack_i2c(uint8_t address, bool read, const uint8_t *prefix, size_t length,
                            uint32_t *expected, uint32_t *bit_count);
const char *timing_pack_spi(const uint8_t *prefix, size_t length, bool lsb_first,
                            uint32_t *expected, uint32_t *bit_count);

/* RP2040 runtime in timing_hw.c; TIMING_RUNTIME_TEST selects its register shim.
 * Init touches no pins. The caller owns global pin collision/busy checks. */
void                 timing_init(void);
const char          *timing_configure_gpio(unsigned pin, timing_drive_t drive, bool idle);
const char          *timing_configure_sync(unsigned pin);
const char          *timing_configure_spi(const timing_spi_t *spi);
const timing_gpio_t *timing_gpio(void);
int                  timing_sync_pin(void);
const timing_spi_t  *timing_spi(void);
const timing_pwm_t  *timing_pwm(void);
/* Configure leaves the carrier off; changing configuration while active is BUSY. */
const char *timing_configure_pwm(unsigned pin, uint32_t hz);
/* Stop low, abort source DMA, clear FIFO and the latched runtime error. */
void        timing_pwm_stop(void);
const char *timing_pwm_error(void);
/* After compilation, reserve the maximum simultaneous scheduled events and
 * prepare every PWM stream. Sparse count-zero blocks are skipped; all live
 * blocks and their source buffers remain immutable until completion/release. */
const char     *timing_prepare_run(timing_block_t *blocks, size_t count, unsigned write_slots);
/* Releases run reservations, preserving an independently owned held PWM tail. */
void            timing_release_run(void);
/* Start a prepared block. Bounds boundary DMA by remaining run duration
 * (1..60,000,000 us). A wait's latch is armed before the whole block, including
 * pulses before that wait. Capture integrity monitoring belongs to the caller. */
const char     *timing_start(const timing_block_t *block, const timing_trigger_t *trigger,
                             uint32_t remaining_run_us);
void            timing_service(void);
timing_status_t timing_status(void);
/* Two input-only, one-shot PIO0 SM0/1 events; no CPU release path.
 * For pin 0..29, arm injects WAIT opposite before the resident reference WAIT.
 * at_cycles is the RX PUSH instruction cycle relative to that synchronized
 * reference level. Arming on the internal sync pin clears its stale latch first.
 * GPIO synchronization/sampling and DMA-to-IC_ENABLE latency are extra;
 * this is not a guarantee of the wire START timestamp.
 * For pin -1 and waveform_cycles=0, reference() enables both counters in sync.
 * Nonzero waveform_cycles (1..2^32) starts them with the next waveform block,
 * using its compiler-provided hardware completion offset, without CPU polling.
 * Readiness may be published before reference; it must reach TX before PULL at
 * at_cycles - READY_LEAD. An empty FIFO halts permanently with MISSED_DEADLINE.
 * Arm accepts 1..LIMIT strictly increasing timestamps,
 * each >= MIN_CYCLES, and aligned non-null ENABLE destinations. The caller owns
 * destination validity and must stop these engines before resetting controllers.
 * Active PWM and subsequent waveform/wait blocks coexist with these events.
 * Ordinary waveform completion keeps events armed. Stop quiesces DMA, clears
 * permits and errors, retaining the run reservation. Arm requires no active
 * waveform and count <= the slots reserved by timing_prepare_run(). */
const char *timing_write_arm(int pin, bool rising, const uint32_t *at_cycles,
                              volatile uint32_t *const *enable_registers, size_t count,
                              uint64_t waveform_cycles);
/* Starts a deferred reference once; hardware, unarmed or repeated calls fail. */
const char *timing_write_reference(void);
const char *timing_write_ready(unsigned index);
/* Actual completed release results survive fault teardown until stop/new arm. */
bool        timing_write_released(unsigned index);
const char *timing_write_error(void);
void        timing_write_stop(void);
/* Explicit stop restores configured GPIO idle and held metadata. Successful
 * service completion preserves final GPIO/PWM state; PWM stop is independent. */
void timing_stop(void);

#endif
