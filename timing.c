#include "timing.h"

static uint32_t rotate_pins(uint32_t mask, unsigned base) {
    return base ? (mask >> base) | (mask << (32u - base)) : mask;
}

typedef struct {
    const timing_gpio_t *gpio;
    uint32_t            *tokens;
    size_t               capacity, used;
    uint32_t             levels, dirs;
    uint64_t             elapsed_cycles, total_cycles, inserted_cycles;
    unsigned             edge_offset, first_edge_offset;
    bool                 needs_spacing, waits;
} wave_compile_t;

static const char *wave_frame(wave_compile_t *wave, uint32_t countdown) {
    if (!wave->used) {
        wave->first_edge_offset = wave->edge_offset;
    }
    if (wave->capacity < 3u || wave->used > wave->capacity - 3u ||
        wave->used > TIMING_MAX_TOKENS - 3u) {
        return "RANGE";
    }
    wave->tokens[wave->used++] = rotate_pins(wave->levels, wave->gpio->out_base);
    wave->tokens[wave->used++] = rotate_pins(wave->dirs, wave->gpio->out_base);
    wave->tokens[wave->used++] = countdown;
    return NULL;
}

/* OUT PINDIRS follows OUT PINS by one cycle. Compensate both physical edge
 * offsets, including push-pull transitions from a previously floating pin. */
static const char *wave_interval(wave_compile_t *wave, unsigned next_edge_offset) {
    uint64_t       remaining_cycles = wave->elapsed_cycles + wave->edge_offset - next_edge_offset;
    const uint32_t maximum_cycles   = TIMING_MAX_COUNTDOWN + 6u;
    while (remaining_cycles > maximum_cycles) {
        /* Leave at least seven cycles for the final non-WAIT frame. */
        uint32_t frame_cycles =
            (uint32_t) (remaining_cycles - 7u < maximum_cycles ? remaining_cycles - 7u
                                                               : maximum_cycles);
        const char *error = wave_frame(wave, frame_cycles - 6u);
        if (error) {
            return error;
        }
        remaining_cycles -= frame_cycles;
    }
    return wave_frame(wave, (uint32_t) remaining_cycles - 6u);
}

static const char *wave_change(wave_compile_t *wave, uint32_t levels, uint32_t dirs) {
    if (levels == wave->levels && dirs == wave->dirs) {
        return NULL;
    }
    unsigned edge_offset = dirs != wave->dirs;
    if (wave->needs_spacing && wave->elapsed_cycles < TIMING_MIN_INTERVAL_CYCLES) {
        uint32_t padding = TIMING_MIN_INTERVAL_CYCLES - (uint32_t) wave->elapsed_cycles;
        wave->elapsed_cycles += padding;
        wave->total_cycles += padding;
        wave->inserted_cycles += padding;
    }
    if (wave->elapsed_cycles) {
        const char *error = wave_interval(wave, edge_offset);
        if (error) {
            return error;
        }
    }
    wave->levels         = levels;
    wave->dirs           = dirs;
    wave->edge_offset    = edge_offset;
    wave->elapsed_cycles = 0;
    wave->needs_spacing  = true;
    return NULL;
}

static const char *wave_wait(wave_compile_t *wave) {
    if (wave->elapsed_cycles) {
        const char *error = wave_interval(wave, 0);
        if (error) {
            return error;
        }
        wave->edge_offset = 0;
    }
    const char *error = wave_frame(wave, 0);
    if (error) {
        return error;
    }
    wave->waits          = true;
    wave->needs_spacing  = true;
    wave->edge_offset    = 0;
    wave->elapsed_cycles = 0;
    return NULL;
}

static void pin_state(const timing_gpio_t *gpio, uint32_t bit, timing_pin_state_t state,
                      uint32_t *levels, uint32_t *dirs) {
    if (state == TIMING_PIN_FLOATING) {
        *dirs &= ~bit; /* Preserve the latch: OUT PINS must not glitch before OE drops. */
    } else if ((gpio->open_drain_mask | gpio->open_source_mask) & bit) {
        bool source = !!(gpio->open_source_mask & bit);
        *levels     = (*levels & ~bit) | (source ? bit : 0);
        *dirs       = (*dirs & ~bit) | ((state == TIMING_PIN_HIGH) == source ? bit : 0);
    } else {
        *levels = (*levels & ~bit) | (state == TIMING_PIN_HIGH ? bit : 0);
        *dirs |= bit;
    }
}

static const char *compile_gpio_instruction(wave_compile_t             *wave,
                                            const timing_instruction_t *instruction) {
    const timing_gpio_t *gpio = wave->gpio;
    if (instruction->pin > 29u) {
        return "RANGE";
    }
    uint32_t bit = 1u << instruction->pin;
    if (!(gpio->configured_mask & bit)) {
        return "UNCONFIGURED";
    }
    if (instruction->op == TIMING_PULSE && instruction->cycles < TIMING_MIN_INTERVAL_CYCLES) {
        return "RANGE";
    }
    uint32_t           levels = wave->levels;
    uint32_t           dirs   = wave->dirs;
    timing_pin_state_t target =
        instruction->op == TIMING_SET
            ? instruction->state
            : (gpio->idle_high_mask & bit ? TIMING_PIN_LOW : TIMING_PIN_HIGH);
    pin_state(gpio, bit, target, &levels, &dirs);
    const char *error = wave_change(wave, levels, dirs);
    if (error) {
        return error;
    }
    if (instruction->op == TIMING_PULSE) {
        wave->elapsed_cycles += instruction->cycles;
        wave->total_cycles += instruction->cycles;
        pin_state(gpio, bit, gpio->idle_high_mask & bit ? TIMING_PIN_HIGH : TIMING_PIN_LOW, &levels,
                  &dirs);
        return wave_change(wave, levels, dirs);
    }
    return NULL;
}

const char *timing_pwm_parameters(uint32_t sys_hz, uint32_t hz, uint32_t *period_cycles) {
    if (!sys_hz || !hz || !period_cycles) {
        return "RANGE";
    }
    uint64_t period = ((uint64_t) sys_hz + hz / 2u) / hz;
    if (period < 16u || period > 32768u) {
        return "RANGE";
    }
    *period_cycles = (uint32_t) period;
    return NULL;
}

static void pwm_phase(uint32_t *words, unsigned word_count, uint32_t duration_cycles, bool level,
                      bool drive) {
    uint32_t set_instruction = 0xe000u | 0x1000u | (drive ? 0x0800u : 0) | level;
    for (unsigned i = 0; i < word_count; ++i) {
        words[i] = 0xa042u; /* NOP */
    }
    if (duration_cycles >= 2u * word_count && duration_cycles <= 2u * word_count + 7u) {
        words[0] = set_instruction | ((duration_cycles - 2u * word_count) << 8);
    } else {
        words[0] = set_instruction;
        words[1] = TIMING_PWM_HOLD_PC; /* Injected absolute JMP, not assembler-relocated. */
        words[2] = duration_cycles - (2u * word_count + 1u);
    }
}

static const char *pwm_encode_mode(uint32_t period, uint32_t duty, timing_pwm_mode_t mode,
                                   uint32_t words[8], uint32_t *count, uint32_t *high_cycles) {
    if (period < 16u || period > 32768u || duty > 10000u || (unsigned) mode > TIMING_PWM_FLOATING ||
        !words || !count || !high_cycles) {
        return "RANGE";
    }
    uint32_t rounded_high_cycles = (period * duty + 5000u) / 10000u;
    bool     carrier             = mode == TIMING_PWM_CARRIER && duty && duty != 10000u;
    if (carrier && (rounded_high_cycles < 3u || period - rounded_high_cycles < 10u)) {
        return "RANGE";
    }
    uint32_t first_phase_cycles = carrier ? rounded_high_cycles : 3u;
    bool     held_high = mode == TIMING_PWM_HIGH || (mode == TIMING_PWM_CARRIER && duty == 10000u);
    unsigned first_words  = first_phase_cycles <= 9u ? 1u : 3u;
    unsigned second_words = first_phase_cycles <= 9u ? 3u : 5u;
    pwm_phase(words, first_words, first_phase_cycles, carrier || held_high,
              mode != TIMING_PWM_FLOATING);
    pwm_phase(words + first_words, second_words, period - first_phase_cycles, held_high,
              mode != TIMING_PWM_FLOATING);
    *count       = first_words + second_words;
    *high_cycles = mode == TIMING_PWM_CARRIER ? rounded_high_cycles : (held_high ? period : 0);
    return NULL;
}

const char *timing_pwm_encode(uint32_t period_cycles, uint32_t duty_basis_points, uint32_t words[8],
                              uint32_t *word_count, uint32_t *high_cycles) {
    return pwm_encode_mode(period_cycles, duty_basis_points, TIMING_PWM_CARRIER, words, word_count,
                           high_cycles);
}

static const char *put_pwm_periods(timing_pwm_segment_t *segments, size_t capacity, size_t *used,
                                   const timing_pwm_segment_t *phase, uint64_t periods) {
    while (periods) {
        if (*used >= capacity || *used >= TIMING_MAX_PWM_SEGMENTS) {
            return "RANGE";
        }
        uint32_t maximum_periods = UINT32_MAX / phase->word_count;
        uint32_t part            = periods > maximum_periods ? maximum_periods : (uint32_t) periods;
        timing_pwm_segment_t *segment = &segments[(*used)++];
        for (unsigned i = 0; i < phase->word_count; ++i) {
            segment->words[i] = phase->words[i];
        }
        segment->periods           = part;
        segment->word_count        = phase->word_count;
        segment->mode              = phase->mode;
        segment->duty_basis_points = phase->duty_basis_points;
        periods -= part;
    }
    return NULL;
}

static const char *compile_pwm_instruction(timing_pwm_segment_t       *phase,
                                           const timing_instruction_t *instruction,
                                           uint32_t                    period_cycles) {
    if (instruction->op == TIMING_PWM) {
        phase->mode              = TIMING_PWM_CARRIER;
        phase->duty_basis_points = instruction->duty_basis_points;
    } else {
        switch (instruction->state) {
            case TIMING_PIN_LOW:
                phase->mode = TIMING_PWM_LOW;
                break;
            case TIMING_PIN_HIGH:
                phase->mode = TIMING_PWM_HIGH;
                break;
            case TIMING_PIN_FLOATING:
                phase->mode = TIMING_PWM_FLOATING;
                break;
        }
        phase->duty_basis_points = instruction->state == TIMING_PIN_HIGH ? 10000u : 0;
    }
    uint32_t high_cycles;
    return pwm_encode_mode(period_cycles, phase->duty_basis_points, phase->mode, phase->words,
                           &phase->word_count, &high_cycles);
}

/* One pass owns GPIO padding and PWM timestamps. No raw, unadjusted second
 * timeline can silently move a PWM update off the requested GPIO boundary. */
static const char *compile_block(const timing_gpio_t *gpio, const timing_pwm_t *pwm,
                                 const timing_instruction_t *instructions, size_t count,
                                 uint32_t initial_duty, uint32_t *tokens, size_t capacity,
                                 timing_pwm_segment_t *segments, size_t pwm_capacity,
                                 timing_block_t *block) {
    if (!gpio) {
        return "UNCONFIGURED";
    }
    if ((gpio->configured_mask & ~0x3fffffffu) || gpio->out_base > 31u ||
        ((gpio->open_drain_mask | gpio->open_source_mask | gpio->idle_high_mask |
          gpio->current_levels | gpio->current_dirs) &
         ~gpio->configured_mask) ||
        (gpio->open_drain_mask & gpio->open_source_mask)) {
        return "RANGE";
    }
    if (!instructions || !count || count > 64u || !tokens || !block || !capacity) {
        return "RANGE";
    }
    bool has_pwm = false;
    for (size_t i = 0; i < count; ++i) {
        has_pwm |= instructions[i].op == TIMING_PWM ||
                   (instructions[i].op == TIMING_SET && pwm && pwm->configured &&
                    instructions[i].pin == pwm->pin);
    }
    timing_pwm_segment_t phase = {0};
    if (has_pwm) {
        if (!pwm || !pwm->configured) {
            return "UNCONFIGURED";
        }
        if (!segments) {
            return "RANGE";
        }
        phase.mode              = pwm->final_mode;
        phase.duty_basis_points = initial_duty;
        uint32_t    high_cycles;
        const char *error = pwm_encode_mode(pwm->period_cycles, initial_duty, phase.mode,
                                            phase.words, &phase.word_count, &high_cycles);
        if (error) {
            return error;
        }
    }
    wave_compile_t wave = {
        .gpio     = gpio,
        .tokens   = tokens,
        .capacity = capacity,
        .levels   = gpio->current_levels,
        .dirs     = gpio->current_dirs,
    };
    size_t   pwm_used           = 0;
    uint64_t previous_pwm_cycle = 0;
    for (size_t i = 0; i < count; ++i) {
        const timing_instruction_t *instruction = &instructions[i];
        if ((unsigned) instruction->op > TIMING_SET) {
            return "BAD_REQUEST";
        }
        if (instruction->op == TIMING_DELAY) {
            if (instruction->cycles && instruction->cycles < TIMING_MIN_INTERVAL_CYCLES) {
                return "RANGE";
            }
            wave.elapsed_cycles += instruction->cycles;
            wave.total_cycles += instruction->cycles;
            continue;
        }
        if (instruction->op == TIMING_WAIT) {
            if (has_pwm || wave.waits || instruction->cycles) {
                return "RANGE";
            }
            const char *error = wave_wait(&wave);
            if (error) {
                return error;
            }
            continue;
        }
        if (instruction->op == TIMING_SET &&
            (instruction->cycles || (unsigned) instruction->state > TIMING_PIN_FLOATING)) {
            return "RANGE";
        }
        if (instruction->op == TIMING_PWM ||
            (has_pwm && instruction->op == TIMING_SET && instruction->pin == pwm->pin)) {
            if (instruction->cycles || wave.total_cycles % pwm->period_cycles) {
                return "RANGE";
            }
            const char *error =
                put_pwm_periods(segments, pwm_capacity, &pwm_used, &phase,
                                (wave.total_cycles - previous_pwm_cycle) / pwm->period_cycles);
            if (error) {
                return error;
            }
            previous_pwm_cycle = wave.total_cycles;
            error              = compile_pwm_instruction(&phase, instruction, pwm->period_cycles);
            if (error) {
                return error;
            }
            continue;
        }
        const char *error = compile_gpio_instruction(&wave, instruction);
        if (error) {
            return error;
        }
    }
    if (wave.elapsed_cycles) {
        const char *error = wave_interval(&wave, 0);
        if (error) {
            return error;
        }
        wave.edge_offset = 0;
    }
    /* The final state remains latched when the empty OUT stalls. */
    const char *error = wave_frame(&wave, 1);
    if (error) {
        return error;
    }
    if (has_pwm) {
        error = put_pwm_periods(segments, pwm_capacity, &pwm_used, &phase,
                                UINT32_MAX / phase.word_count);
        if (error) {
            return error;
        }
    }
    *block = (timing_block_t) {
        .tokens               = tokens,
        .count                = wave.used,
        .waits                = wave.waits,
        .programmed_cycles    = wave.total_cycles,
        .inserted_cycles      = wave.inserted_cycles,
        .final_levels         = wave.levels,
        .final_dirs           = wave.dirs,
        .pwm_segments         = has_pwm ? segments : NULL,
        .pwm_count            = pwm_used,
        .pwm_final_duty       = phase.duty_basis_points,
        .pwm_final_mode       = phase.mode,
        .wave_start_countdown = has_pwm ? 2u - wave.first_edge_offset : 0,
        .completion_cycles =
            count != 1u ? 0 :
            has_pwm ? 6u :
            instructions[0].op == TIMING_DELAY ? (uint64_t) instructions[0].cycles + 1u :
            instructions[0].op == TIMING_SET ? 1u + wave.first_edge_offset : 0,
    };
    return NULL;
}

const char *timing_compile(const timing_gpio_t *gpio, const timing_instruction_t *instructions,
                           size_t count, uint32_t *tokens, size_t capacity, timing_block_t *block) {
    return compile_block(gpio, NULL, instructions, count, 0, tokens, capacity, NULL, 0, block);
}

const char *timing_compile_pwm(const timing_gpio_t *gpio, const timing_pwm_t *pwm,
                               const timing_instruction_t *instructions, size_t count,
                               uint32_t initial_duty, uint32_t *tokens, size_t capacity,
                               timing_pwm_segment_t *segments, size_t pwm_capacity,
                               timing_block_t *block) {
    return compile_block(gpio, pwm, instructions, count, initial_duty, tokens, capacity, segments,
                         pwm_capacity, block);
}

const char *timing_pack_i2c(uint8_t address, bool read, const uint8_t *prefix, size_t length,
                            uint32_t *expected, uint32_t *bit_count) {
    if (address < 0x08 || address > 0x77 || length > 2 || (length && !prefix) || !expected ||
        !bit_count) {
        return "RANGE";
    }
    uint32_t word = ((uint32_t) address << 1) | read;
    for (size_t i = 0; i < length; ++i) {
        word = (word << 9) | prefix[i];
    }
    *expected  = word;
    *bit_count = 8u + (uint32_t) length * 9u;
    return NULL;
}

const char *timing_pack_spi(const uint8_t *prefix, size_t length, bool lsb_first,
                            uint32_t *expected, uint32_t *bit_count) {
    if (!prefix || !length || length > 4 || !expected || !bit_count) {
        return "RANGE";
    }
    uint32_t word = 0;
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = prefix[i];
        if (lsb_first) {
            byte = (uint8_t) (((byte & 0x55u) << 1) | ((byte >> 1) & 0x55u));
            byte = (uint8_t) (((byte & 0x33u) << 2) | ((byte >> 2) & 0x33u));
            byte = (uint8_t) ((byte << 4) | (byte >> 4));
        }
        word = (word << 8) | byte;
    }
    *expected  = word;
    *bit_count = (uint32_t) length * 8u;
    return NULL;
}
