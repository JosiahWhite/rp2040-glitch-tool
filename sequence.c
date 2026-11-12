/* Core 0 stages and compiles under acknowledged executor ownership; core 1
 * owns run state and execution until completion or acknowledged cancellation. */
#include "sequence.h"

#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"

typedef enum {
    DELAY,
    PULSE,
    WRITE,
    POLL,
    WAIT_I2C,
    WAIT_SPI,
    WAIT_GPIO,
    PWM_DUTY,
    SET_PIN,
    WRITE_AT,
    SYNC_WRITES
} step_kind_t;

typedef struct {
    step_kind_t        kind;
    uint32_t           cycles, timeout, transfer_timeout, interval;
    uint32_t           duty_basis_points;
    uint16_t           offset, length, read_length;
    uint16_t           command_offset;
    uint8_t            bus, address;
    bool               read;
    uint8_t            pin;
    bool               rising;
    timing_pin_state_t pin_state;
} step_t;

/* One staged program; compiled blocks borrow this storage until completion. */
static step_t   steps[SEQUENCE_STEP_LIMIT];
static unsigned step_count, payload_length;
static uint8_t  payload[SEQUENCE_PAYLOAD_LIMIT];
static uint32_t tokens[TIMING_MAX_TOKENS];
static _Alignas(32) timing_pwm_segment_t pwm_segments[TIMING_MAX_PWM_SEGMENTS];
static timing_block_t blocks[SEQUENCE_STEP_LIMIT];
static unsigned       block_end[SEQUENCE_STEP_LIMIT];
static int            block_wait[SEQUENCE_STEP_LIMIT];
static timing_trigger_t block_trigger[SEQUENCE_STEP_LIMIT];
static uint16_t scheduled_commands[SEQUENCE_STEP_LIMIT * I2C_PREPARED_LIMIT];
static struct {
    unsigned count;
    int pin;
    bool rising;
    uint32_t offsets[TIMING_WRITE_LIMIT];
    volatile uint32_t *registers[TIMING_WRITE_LIMIT];
    uint64_t waveform_cycles;
} schedules[SEQUENCE_STEP_LIMIT];

/* Execution cursors are separate from the status exposed to command replies. */
static sequence_status_t run;
static uint64_t          run_deadline, step_deadline, next_attempt;
static bool              step_started, attempt_active, wait_clock_started;
static unsigned scheduled_begin, scheduled_count, scheduled_index, scheduled_reference;
static bool scheduled_preparing, scheduled_ready;

bool __not_in_flash_func(sequence_running)(void) {
    return run.state == SEQUENCE_RUNNING;
}

const sequence_status_t *__not_in_flash_func(sequence_status)(void) {
    return &run;
}

const char *sequence_state_name(sequence_state_t state) {
    static const char *const names[] = {"idle", "running", "done", "error", "aborted"};
    return names[state];
}

static uint64_t __not_in_flash_func(minimum)(uint64_t a, uint64_t b) {
    return a < b ? a : b;
}

static bool deadline_valid(uint32_t value) {
    return value && value <= I2C_DEADLINE_LIMIT_US;
}

static bool address_valid(uint32_t value) {
    return value >= 8 && value <= 0x77;
}
static void __not_in_flash_func(finish)(sequence_state_t terminal, const char *error, uint32_t detail) {
    /* Release DMA must be quiescent before any controller reset, even on success. */
    timing_write_stop();
    timing_release_run();
    /* Successful runs may hold explicit states; every fault restores safe idle. */
    if (terminal != SEQUENCE_DONE || !run.hold_final) {
        timing_stop();
        timing_pwm_stop();
    }
    i2c_bus_cancel();
    run.run_elapsed  = run.run_start ? time_us_64() - run.run_start : 0;
    run.run_error    = error;
    run.error_detail = detail;
    run.state        = terminal;
}

void __not_in_flash_func(sequence_abort)(void) {
    if (sequence_running()) {
        finish(SEQUENCE_ABORTED, "ABORTED", 0);
    } else {
        timing_release_run();
        timing_stop();
        timing_pwm_stop();
        i2c_bus_cancel();
    }
}
const char *sequence_stage(const request_t *request, uint8_t scratch[I2C_TRANSFER_LIMIT]) {
    if (request->argc < 3)
        return "BAD_REQUEST";
    if (step_count == SEQUENCE_STEP_LIMIT)
        return "RANGE";

    step_t      step          = {0};
    const char *operation     = request->argv[2];
    size_t      payload_bytes = 0;

    if (!strcmp(operation, "sync_writes")) {
        if (request->argc != 3)
            return "BAD_REQUEST";
        step.kind = SYNC_WRITES;
    } else if (!strcmp(operation, "delay")) {
        uint32_t cycles;
        if (request->argc != 4 || !request_u32(request, 3, &cycles))
            return "BAD_REQUEST";
        if (cycles && cycles < TIMING_MIN_INTERVAL_CYCLES)
            return "RANGE";
        step.kind   = DELAY;
        step.cycles = cycles;
    } else if (!strcmp(operation, "pulse")) {
        uint32_t pin, cycles;
        if (request->argc != 5 || !request_u32(request, 3, &pin) ||
            !request_u32(request, 4, &cycles))
            return "BAD_REQUEST";
        if (pin > 29 || cycles < TIMING_MIN_INTERVAL_CYCLES)
            return "RANGE";
        step.kind   = PULSE;
        step.pin    = pin;
        step.cycles = cycles;
    } else if (!strcmp(operation, "write") || !strcmp(operation, "write_at")) {
        bool scheduled = !strcmp(operation, "write_at");
        uint32_t bus, address, timeout, at_cycles = 0;
        unsigned timeout_arg = scheduled ? 6u : 5u;
        if (request->argc != (scheduled ? 8 : 7) || !request_u32(request, 3, &bus) ||
            !request_u32(request, 4, &address) ||
            (scheduled && !request_u32(request, 5, &at_cycles)) ||
            !request_u32(request, timeout_arg, &timeout)) {
            return "BAD_REQUEST";
        }
        if (bus >= I2C_BUS_COUNT || !address_valid(address) || !deadline_valid(timeout) ||
            (scheduled && at_cycles < TIMING_WRITE_MIN_CYCLES))
            return "RANGE";
        if (!parse_hex(request->argv[timeout_arg + 1], scratch,
                       scheduled ? I2C_PREPARED_LIMIT : SEQUENCE_ROUTINE_BYTES, &payload_bytes) ||
            !payload_bytes)
            return "RANGE";
        step.kind    = scheduled ? WRITE_AT : WRITE;
        step.bus     = bus;
        step.address = address;
        step.timeout = timeout;
        step.length  = payload_bytes;
        step.cycles  = at_cycles;
    } else if (!strcmp(operation, "poll")) {
        uint32_t bus, address, read_length, interval, timeout, transfer_timeout;
        if (request->argc != 12 || !request_u32(request, 3, &bus) ||
            !request_u32(request, 4, &address) || !request_u32(request, 6, &read_length) ||
            !request_u32(request, 9, &interval) || !request_u32(request, 10, &timeout) ||
            !request_u32(request, 11, &transfer_timeout))
            return "BAD_REQUEST";
        if (bus >= I2C_BUS_COUNT || !address_valid(address) || read_length < 1 ||
            read_length > SEQUENCE_ROUTINE_BYTES || interval > I2C_DEADLINE_LIMIT_US ||
            !deadline_valid(timeout) || !deadline_valid(transfer_timeout))
            return "RANGE";

        /* Poll payload is register bytes, then an equally sized mask and value. */
        if (!parse_hex(request->argv[5], scratch, SEQUENCE_ROUTINE_BYTES, &payload_bytes))
            return "RANGE";
        step.length = payload_bytes;
        size_t field_bytes;
        if (!parse_hex(request->argv[7], scratch + payload_bytes, SEQUENCE_ROUTINE_BYTES,
                       &field_bytes) ||
            field_bytes != read_length)
            return "RANGE";
        payload_bytes += field_bytes;
        if (!parse_hex(request->argv[8], scratch + payload_bytes, SEQUENCE_ROUTINE_BYTES,
                       &field_bytes) ||
            field_bytes != read_length)
            return "RANGE";
        payload_bytes += field_bytes;
        step.kind             = POLL;
        step.bus              = bus;
        step.address          = address;
        step.read_length      = read_length;
        step.interval         = interval;
        step.timeout          = timeout;
        step.transfer_timeout = transfer_timeout;
    } else if (!strcmp(operation, "wait_i2c")) {
        uint32_t bus, address, read, timeout;
        if (request->argc != 8 || !request_u32(request, 3, &bus) ||
            !request_u32(request, 4, &address) || !request_u32(request, 5, &read) ||
            !request_u32(request, 7, &timeout))
            return "BAD_REQUEST";
        if (bus >= I2C_BUS_COUNT || !address_valid(address) || read > 1 || !deadline_valid(timeout))
            return "RANGE";
        if (!parse_hex(request->argv[6], scratch, 2, &payload_bytes))
            return "RANGE";
        step.kind    = WAIT_I2C;
        step.bus     = bus;
        step.address = address;
        step.read    = read;
        step.timeout = timeout;
        step.length  = payload_bytes;
    } else if (!strcmp(operation, "wait_spi")) {
        uint32_t timeout;
        if (request->argc != 5 || !request_u32(request, 4, &timeout))
            return "BAD_REQUEST";
        if (!deadline_valid(timeout) || !parse_hex(request->argv[3], scratch, 4, &payload_bytes) ||
            !payload_bytes) {
            return "RANGE";
        }
        step.kind    = WAIT_SPI;
        step.timeout = timeout;
        step.length  = payload_bytes;
    } else if (!strcmp(operation, "wait_gpio")) {
        uint32_t pin, rising, timeout;
        if (request->argc != 6 || !request_u32(request, 3, &pin) ||
            !request_u32(request, 4, &rising) || !request_u32(request, 5, &timeout)) {
            return "BAD_REQUEST";
        }
        if (pin > 29 || rising > 1 || !deadline_valid(timeout))
            return "RANGE";
        step.kind    = WAIT_GPIO;
        step.pin     = pin;
        step.rising  = rising;
        step.timeout = timeout;
    } else if (!strcmp(operation, "pwm")) {
        uint32_t duty;
        if (request->argc != 4 || !request_u32(request, 3, &duty))
            return "BAD_REQUEST";
        if (duty > 10000)
            return "RANGE";
        step.kind              = PWM_DUTY;
        step.duty_basis_points = duty;
    } else if (!strcmp(operation, "set")) {
        uint32_t pin;
        if (request->argc != 5 || !request_u32(request, 3, &pin))
            return "BAD_REQUEST";
        if (pin > 29)
            return "RANGE";
        if (!strcmp(request->argv[4], "low"))
            step.pin_state = TIMING_PIN_LOW;
        else if (!strcmp(request->argv[4], "high"))
            step.pin_state = TIMING_PIN_HIGH;
        else if (!strcmp(request->argv[4], "floating"))
            step.pin_state = TIMING_PIN_FLOATING;
        else
            return "RANGE";
        step.kind = SET_PIN;
        step.pin  = pin;
    } else {
        return "BAD_REQUEST";
    }

    /* Commit only after validation; failed staging leaves the program unchanged. */
    if (payload_bytes > SEQUENCE_PAYLOAD_LIMIT - payload_length)
        return "RANGE";
    step.offset = payload_length;
    memcpy(payload + payload_length, scratch, payload_bytes);
    payload_length += payload_bytes;
    steps[step_count++] = step;
    return NULL;
}

static bool __not_in_flash_func(timed)(step_kind_t k) {
    return k == DELAY || k == PULSE || k == WAIT_I2C || k == WAIT_SPI || k == WAIT_GPIO ||
           k == PWM_DUTY || k == SET_PIN;
}
static bool __not_in_flash_func(write_reference)(unsigned index) {
    return index + 1u < step_count && steps[index + 1u].kind == WRITE_AT;
}

static bool __not_in_flash_func(hardware_reference)(step_kind_t kind) {
    return kind == PULSE || kind == WAIT_GPIO || kind == WAIT_I2C || kind == WAIT_SPI;
}

static bool __not_in_flash_func(write_barrier)(step_kind_t kind) {
    return kind == WRITE || kind == POLL || kind == SYNC_WRITES;
}

static timing_trigger_t trigger_for(const step_t *step);

static const char *compile_run(uint32_t occupied_pins) {
    if (!step_count)
        return "RANGE";
    memset(blocks, 0, sizeof(blocks));
    memset(block_trigger, 0, sizeof(block_trigger));
    unsigned      waits = 0, used = 0, pwm_used = 0;
    timing_gpio_t gpio_state = *timing_gpio();
    timing_pwm_t  pwm_state  = *timing_pwm();
    uint64_t      cycles = 0, inserted = 0;
    for (unsigned i = 0; i < step_count;) {
        step_t *step = &steps[i];
        if ((step->kind == WRITE || step->kind == WRITE_AT || step->kind == POLL ||
             step->kind == WAIT_I2C) &&
            !i2c_bus_config(step->bus)->configured)
            return "UNCONFIGURED";
        if (!timed(step->kind)) {
            ++i;
            continue;
        }
        timing_instruction_t input[SEQUENCE_STEP_LIMIT];
        unsigned             begin = i, count = 0;
        bool                 has_pwm = false;
        block_wait[begin]            = -1;
        while (i < step_count && timed(steps[i].kind)) {
            /* A reference must observe only its own command, never an earlier pulse. */
            if (i != begin && write_reference(i))
                break;
            step = &steps[i];
            timing_op_t op;
            switch (step->kind) {
                case DELAY:
                    op = TIMING_DELAY;
                    break;
                case PULSE:
                    op = TIMING_PULSE;
                    break;
                case SET_PIN:
                    op = TIMING_SET;
                    if (pwm_state.configured && step->pin == pwm_state.pin)
                        has_pwm = true;
                    else if (!(gpio_state.configured_mask & (1u << step->pin)))
                        return "UNCONFIGURED";
                    break;
                case PWM_DUTY:
                    op      = TIMING_PWM;
                    has_pwm = true;
                    if (!timing_pwm()->configured)
                        return "UNCONFIGURED";
                    break;
                default:
                    op = TIMING_WAIT;
                    if (++waits > 1)
                        return "RANGE";
                    if (timing_sync_pin() < 0)
                        return "UNCONFIGURED";
                    if (step->kind == WAIT_I2C && !i2c_bus_config(step->bus)->configured)
                        return "UNCONFIGURED";
                    if (step->kind == WAIT_SPI && !timing_spi()->configured)
                        return "UNCONFIGURED";
                    if (step->kind == WAIT_GPIO && (occupied_pins & (1u << step->pin)))
                        return "PIN_CONFLICT";
                    block_wait[begin] = (int) i;
                    break;
            }
            input[count++] = (timing_instruction_t) {.op                = op,
                                                     .cycles            = step->cycles,
                                                     .duty_basis_points = step->duty_basis_points,
                                                     .pin               = step->pin,
                                                     .state             = step->pin_state};
            ++i;
            if (write_reference(i - 1u))
                break;
        }
        const char *error;
        if (has_pwm) {
            error = timing_compile_pwm(&gpio_state, &pwm_state, input, count,
                                       pwm_state.final_duty_basis_points, tokens + used,
                                       TIMING_MAX_TOKENS - used, pwm_segments + pwm_used,
                                       TIMING_MAX_PWM_SEGMENTS - pwm_used, &blocks[begin]);
        } else {
            error = timing_compile(&gpio_state, input, count, tokens + used,
                                   TIMING_MAX_TOKENS - used, &blocks[begin]);
        }
        if (error)
            return error;
        used += blocks[begin].count;
        pwm_used += blocks[begin].pwm_count;
        gpio_state.current_levels = blocks[begin].final_levels;
        gpio_state.current_dirs   = blocks[begin].final_dirs;
        if (has_pwm) {
            pwm_state.final_duty_basis_points = blocks[begin].pwm_final_duty;
            pwm_state.final_mode              = blocks[begin].pwm_final_mode;
        }
        cycles += blocks[begin].programmed_cycles;
        inserted += blocks[begin].inserted_cycles;
        block_end[begin] = i;
        if (block_wait[begin] >= 0)
            block_trigger[begin] = trigger_for(&steps[block_wait[begin]]);
    }
    run.compiled_tokens    = used;
    run.compiled_pwm_words = pwm_used * TIMING_PWM_SEGMENT_WORDS;
    run.programmed_cycles  = cycles;
    run.inserted_cycles    = inserted;
    return NULL;
}

static uint64_t wire_cycles(const step_t *step) {
    uint32_t bytes = step->length + step->read_length + 1u;
    if (step->length && step->read_length)
        ++bytes; /* Repeated START/address for a register read. */
    uint32_t hz = i2c_bus_config(step->bus)->hz;
    return (9u * bytes * (uint64_t) clock_get_hz(clk_sys) + hz - 1u) / hz;
}

static const char *validate_schedule(uint32_t timeout_us) {
    bool pending = false;
    uint64_t earliest = 0, pending_end = 0, reference_duration = 0;
    uint64_t sys_hz = clock_get_hz(clk_sys);
    for (unsigned i = 0; i < step_count;) {
        const step_t *step = &steps[i];
        if (write_barrier(step->kind)) {
            if (pending_end > earliest)
                earliest = pending_end;
            pending = false;
        }
        if (timed(step->kind)) {
            reference_duration = blocks[i].programmed_cycles;
            earliest += reference_duration;
            i = block_end[i];
            continue;
        }
        if (step->kind != WRITE_AT) {
            reference_duration = 0;
            if (step->kind == WRITE || step->kind == POLL)
                earliest += wire_cycles(step);
            ++i;
            continue;
        }
        if (!i || pending)
            return "RANGE";
        unsigned begin = i;
        uint64_t previous_wire = 0;
        while (i < step_count && steps[i].kind == WRITE_AT) {
            step = &steps[i];
            if (i - begin == TIMING_WRITE_LIMIT)
                return "RANGE";
            uint64_t wire = wire_cycles(step);
            uint64_t wait = reference_duration + step->cycles;
            if (i != begin) {
                uint32_t previous = steps[i - 1u].cycles;
                if (step->cycles <= previous ||
                    step->cycles - previous <= previous_wire + TIMING_WRITE_READY_LEAD)
                    return "RANGE";
                wait = step->cycles - previous - previous_wire;
            }
            /* Preparation's timeout also covers its pre-reference blocked wait. */
            if ((wait + wire) * 1000000u >= (uint64_t) step->timeout * sys_hz)
                return "RANGE";
            pending_end = earliest + step->cycles + wire;
            if (pending_end * 1000000u >= (uint64_t) timeout_us * sys_hz)
                return "RANGE";
            previous_wire = wire;
            ++i;
        }
        pending = true;
    }
    return NULL;
}
static timing_trigger_t trigger_for(const step_t *step) {
    timing_trigger_t t = {0};
    if (step->kind == WAIT_GPIO) {
        t.kind   = TIMING_TRIGGER_GPIO;
        t.pin    = step->pin;
        t.rising = step->rising;
        return t;
    }
    t.kind          = step->kind == WAIT_SPI ? TIMING_TRIGGER_SPI : TIMING_TRIGGER_I2C;
    t.address       = step->address;
    t.read          = step->read;
    t.prefix_length = step->length;
    memcpy(t.prefix, payload + step->offset, step->length);
    if (t.kind == TIMING_TRIGGER_I2C) {
        t.sda = i2c_bus_config(step->bus)->sda;
        t.scl = i2c_bus_config(step->bus)->scl;
    }
    return t;
}

/* Resolve immutable group metadata and encode native FIFO words before execution.
 * Only controller-idle-dependent FIFO stores remain on the turnaround path. */
static unsigned prepare_schedules(void) {
    memset(schedules, 0, sizeof(schedules));
    unsigned used = 0, slots = 0;
    for (unsigned reference = 0; reference + 1u < step_count; ++reference) {
        if (!write_reference(reference))
            continue;
        const step_t *step = &steps[reference];
        schedules[reference].pin = -1;
        schedules[reference].rising = true;
        if (step->kind == PULSE) {
            schedules[reference].pin = step->pin;
            schedules[reference].rising = !!(timing_gpio()->idle_high_mask & (1u << step->pin));
        } else if (hardware_reference(step->kind)) {
            schedules[reference].pin = timing_sync_pin();
        } else if (timed(step->kind)) {
            schedules[reference].waveform_cycles = blocks[reference].completion_cycles;
        }
        for (unsigned i = reference + 1u; i < step_count && steps[i].kind == WRITE_AT; ++i) {
            step_t *write = &steps[i];
            unsigned slot = schedules[reference].count++;
            schedules[reference].offsets[slot] = write->cycles;
            schedules[reference].registers[slot] = i2c_bus_enable_register(write->bus);
            write->command_offset = (uint16_t) used;
            for (unsigned byte = 0; byte < write->length; ++byte)
                scheduled_commands[used++] = payload[write->offset + byte] |
                                             (byte + 1u == write->length ? (1u << 9) : 0u);
        }
        if (schedules[reference].count > slots)
            slots = schedules[reference].count;
        reference += schedules[reference].count;
    }
    return slots;
}
static void __not_in_flash_func(scheduled_failure)(const char *error, uint32_t detail) {
    run.current_step = scheduled_begin + scheduled_index;
    if (!strcmp(error, "MISSED_DEADLINE")) {
        for (unsigned i = scheduled_index; i < scheduled_count; ++i) {
            if (!timing_write_released(i)) {
                run.current_step = scheduled_begin + i;
                break;
            }
        }
    }
    finish(SEQUENCE_ERROR, error, detail);
}

static bool __not_in_flash_func(begin_schedule)(unsigned reference) {
    scheduled_reference = reference;
    scheduled_begin = reference + 1u;
    scheduled_index = 0;
    scheduled_count = schedules[reference].count;
    scheduled_preparing = scheduled_ready = false;
    const char *error = timing_write_arm(schedules[reference].pin, schedules[reference].rising,
                                         schedules[reference].offsets,
                                         schedules[reference].registers, scheduled_count,
                                         schedules[reference].waveform_cycles);
    if (error) {
        scheduled_failure(error, 0);
        return false;
    }
    return true;
}

/* Owns only the scheduled group; foreground execution and its cursor are independent. */
static void __not_in_flash_func(service_scheduled)(uint64_t now) {
    if (scheduled_preparing && !i2c_bus_busy() && i2c_bus_result()->error) {
        const i2c_bus_result_t *result = i2c_bus_result();
        scheduled_failure(result->error, result->raw_abort);
        return;
    }
    const char *error = timing_write_error();
    if (error) {
        scheduled_failure(error, 0);
        return;
    }
    if (scheduled_preparing && !i2c_bus_busy()) {
        if (!scheduled_ready || !timing_write_released(scheduled_index)) {
            scheduled_failure("TIMING_ERROR", 0);
            return;
        }
        scheduled_preparing = scheduled_ready = false;
        if (++scheduled_index == scheduled_count) {
            timing_write_stop();
            scheduled_count = 0;
            return;
        }
    }
    const step_t *step = &steps[scheduled_begin + scheduled_index];
    if (!scheduled_preparing) {
        error = i2c_bus_prepare(step->bus, step->address, scheduled_commands + step->command_offset,
                                step->length, minimum(run_deadline, now + step->timeout));
        if (error) {
            scheduled_failure(error, 0);
            return;
        }
        scheduled_preparing = true;
        if (!i2c_bus_busy() && i2c_bus_result()->error) {
            const i2c_bus_result_t *result = i2c_bus_result();
            scheduled_failure(result->error, result->raw_abort);
            return;
        }
    }
    if (!scheduled_ready && i2c_bus_prepared()) {
        error = timing_write_ready(scheduled_index);
        if (error) {
            scheduled_failure(error, 0);
            return;
        }
        scheduled_ready = true;
    }
}

static void __not_in_flash_func(advance)(unsigned next, uint64_t now) {
    unsigned reference = run.current_step;
    if (write_reference(reference)) {
        const step_t *step = &steps[reference];
        if (step->kind == WRITE || step->kind == POLL) {
            /* Native predecessors cannot share the controller with a preloaded write. */
            if (!begin_schedule(reference))
                return;
        }
        if (!timed(step->kind)) {
            const char *error = timing_write_reference();
            if (error) {
                scheduled_failure(error, 0);
                return;
            }
            service_scheduled(now);
            if (!sequence_running())
                return;
        }
        while (next < step_count && steps[next].kind == WRITE_AT)
            ++next;
    }
    run.current_step = next;
    step_started     = false;
    attempt_active   = false;
    if (next == step_count && !scheduled_count)
        finish(SEQUENCE_DONE, NULL, 0);
}
static void __not_in_flash_func(service_timed_block)(uint64_t now) {
    int wait_step = block_wait[run.current_step];
    if (!step_started) {
        wait_clock_started       = false;
        if (wait_step >= 0)
            run.matched_trigger = block_trigger[run.current_step];
        const char *error =
            timing_start(&blocks[run.current_step], &block_trigger[run.current_step],
                         (uint32_t) (run_deadline - now));
        if (error) {
            finish(SEQUENCE_ERROR, error, 0);
            return;
        }
        step_started = true;
    }

    timing_status_t status = timing_status();
    if (status.matched)
        run.matched = true;
    if (status.error) {
        finish(SEQUENCE_ERROR, status.error, 0);
        return;
    }
    if (wait_step >= 0 && status.waiting && !wait_clock_started) {
        wait_clock_started = true;
        step_deadline      = minimum(run_deadline, now + steps[wait_step].timeout);
    }
    if (wait_clock_started && !run.matched && now >= step_deadline) {
        finish(SEQUENCE_ERROR, "WAIT_TIMEOUT", 0);
        return;
    }
    if (status.done)
        advance(block_end[run.current_step], now);
}

static void __not_in_flash_func(service_write)(const step_t *step, uint64_t now) {
    if (!step_started) {
        const char *error =
            i2c_bus_begin(step->bus, step->address, payload + step->offset, step->length, NULL, 0,
                          minimum(run_deadline, now + step->timeout));
        if (error) {
            finish(SEQUENCE_ERROR, error, 0);
            return;
        }
        step_started = true;
    } else if (!i2c_bus_busy()) {
        const i2c_bus_result_t *result = i2c_bus_result();
        if (result->error)
            finish(SEQUENCE_ERROR, result->error, result->raw_abort);
        else
            advance(run.current_step + 1u, now);
    }
}


static bool __not_in_flash_func(poll_matches)(const step_t *step, const uint8_t *received) {
    const uint8_t *mask  = payload + step->offset + step->length;
    const uint8_t *value = mask + step->read_length;
    for (unsigned index = 0; index < step->read_length; ++index) {
        if ((received[index] & mask[index]) != (value[index] & mask[index]))
            return false;
    }
    return true;
}

static void __not_in_flash_func(service_poll)(const step_t *step, uint64_t now, uint8_t *received) {
    if (!step_started) {
        step_started        = true;
        step_deadline       = minimum(run_deadline, now + step->timeout);
        next_attempt        = now;
        run.poll_attempts   = 0;
        run.poll_length     = 0;
        run.last_poll_error = NULL;
    }
    if (attempt_active && !i2c_bus_busy()) {
        attempt_active                 = false;
        const i2c_bus_result_t *result = i2c_bus_result();
        if (result->error) {
            run.last_poll_error = result->error;
            run.error_detail    = result->raw_abort;
            if (strcmp(result->error, "NACK")) {
                finish(SEQUENCE_ERROR, result->error, result->raw_abort);
                return;
            }
        } else {
            run.poll_length = result->read;
            memcpy(run.poll_data, received, run.poll_length);
            if (poll_matches(step, received)) {
                advance(run.current_step + 1u, now);
                return;
            }
        }
    }
    if (now >= step_deadline) {
        finish(SEQUENCE_ERROR, "WAIT_TIMEOUT", run.error_detail);
        return;
    }
    if (!attempt_active && now >= next_attempt) {
        const char *error =
            i2c_bus_begin(step->bus, step->address, payload + step->offset, step->length, received,
                          step->read_length, minimum(step_deadline, now + step->transfer_timeout));
        if (error) {
            finish(SEQUENCE_ERROR, error, 0);
            return;
        }
        ++run.poll_attempts;
        attempt_active = true;
        next_attempt   = now + step->interval;
    }
}

void __not_in_flash_func(sequence_service)(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]) {
    const char *pwm_error = timing_pwm_error();
    if (pwm_error) {
        finish(SEQUENCE_ERROR, pwm_error, 0);
        return;
    }
    /* Consume completed/virtual steps without yielding between adjacent writes.
     * The finite program bounds this pass; an in-flight operation returns at once. */
    for (unsigned budget = 0; budget < SEQUENCE_STEP_LIMIT && sequence_running(); ++budget) {
        uint64_t now = time_us_64();
        if (now >= run_deadline) {
            finish(SEQUENCE_ERROR, "RUN_TIMEOUT", 0);
            return;
        }
        if (scheduled_count) {
            service_scheduled(now);
            if (!sequence_running())
                return;
        }
        if (run.current_step == step_count) {
            if (!scheduled_count)
                finish(SEQUENCE_DONE, NULL, 0);
            return;
        }

        unsigned current = run.current_step;
        const step_t *step = &steps[current];
        /* A barrier joins the old group, not its own future group. */
        if (write_barrier(step->kind) && scheduled_count && scheduled_reference != current)
            return;
        if (write_reference(current) && step->kind != WRITE && step->kind != POLL) {
            if (scheduled_reference != current) {
                if (!begin_schedule(current))
                    return;
                service_scheduled(now);
                if (!sequence_running())
                    return;
            }
            if (!step_started && !scheduled_ready)
                return; /* Preload and permit precede the reference command. */
        }
        if (timed(step->kind))
            service_timed_block(now);
        else if (step->kind == WRITE)
            service_write(step, now);
        else if (step->kind == POLL)
            service_poll(step, now, receive_buffer);
        else
            advance(current + 1u, now);
        if (run.current_step == current)
            return;
    }
}

void sequence_clear(void) {
    step_count = payload_length = 0;
    run.current_step = run.poll_attempts = run.poll_length = 0;
    run.state                                              = SEQUENCE_IDLE;
    run.run_error = run.last_poll_error = NULL;
    run.run_start = run.run_elapsed = 0;
    run.error_detail                = 0;
    run.matched                     = false;
    run.compiled_tokens = run.compiled_pwm_words = 0;
    run.programmed_cycles = run.inserted_cycles = 0;
    wait_clock_started                = false;
    run.hold_final                     = true;
    scheduled_count = scheduled_index = 0;
    scheduled_reference = SEQUENCE_STEP_LIMIT;
    scheduled_preparing = scheduled_ready = false;
}

const char *sequence_start(uint32_t timeout_us, bool hold_final, uint32_t occupied_pins) {
    const char *error = compile_run(occupied_pins);
    if (error)
        return error;
    error = validate_schedule(timeout_us);
    if (error)
        return error;
    unsigned write_slots = prepare_schedules();
    error = timing_prepare_run(blocks, SEQUENCE_STEP_LIMIT, write_slots);
    if (error)
        return error;
    scheduled_count = scheduled_index = 0;
    scheduled_reference = SEQUENCE_STEP_LIMIT;
    scheduled_preparing = scheduled_ready = false;

    run.hold_final   = hold_final;
    run.run_start    = time_us_64();
    run_deadline     = run.run_start + timeout_us;
    run.state        = SEQUENCE_RUNNING;
    run.current_step = 0;
    step_started = attempt_active = run.matched = wait_clock_started = false;
    run.run_error = run.last_poll_error = NULL;
    run.error_detail = run.poll_attempts = run.poll_length = 0;
    run.run_elapsed                                        = 0;
    return NULL;
}
