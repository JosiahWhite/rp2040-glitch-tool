#include "timing.h"

#ifdef TIMING_RUNTIME_TEST
#include "tests/timing_runtime_shim.h"
#else
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/regs/pio.h"
#include "timing.pio.h"
#endif

static uint32_t idle_levels(const timing_gpio_t *gpio) {
    return (gpio->idle_high_mask & ~(gpio->open_drain_mask | gpio->open_source_mask)) |
           gpio->open_source_mask;
}

static uint32_t idle_dirs(const timing_gpio_t *gpio) {
    return gpio->configured_mask & ~(gpio->open_drain_mask & gpio->idle_high_mask) &
           ~(gpio->open_source_mask & ~gpio->idle_high_mask);
}

#define WAVE_SM    2u
#define PWM_SM     3u
#define PWM_OFFSET 11u
_Static_assert(PWM_OFFSET + pwm_sequence_offset_command == TIMING_PWM_COMMAND_PC, "PWM command PC");
_Static_assert(PWM_OFFSET + pwm_sequence_offset_hold == TIMING_PWM_HOLD_PC, "PWM hold PC");
_Static_assert(sizeof(timing_pwm_segment_t) == 64u, "PWM segment DMA stride");
#define MATCH_SM    0u
#define BOUNDARY_SM 1u
#define WAVE_OFFSET 24u
#define WRITE_SM    0u
#define WRITE_OFFSET 0u
#define ORIGIN_OFFSET 16u

static timing_gpio_t gpio;
static int           sync_pin = -1;
static timing_spi_t  spi;
static timing_pwm_t  pwm;
static const char   *pwm_error;
_Alignas(32) static uint32_t pwm_final_words[8];
/* AL3 order: CTRL, WRITE_ADDR, TRANS_COUNT, READ_ADDR_TRIG. */
static uint32_t pwm_descriptors[2u * TIMING_MAX_PWM_SEGMENTS][4];
/* Never borrowed by run preparation: a held carrier may outlive staged blocks. */
static uint32_t pwm_tail_descriptors[2][4];
static dma_channel_config pwm_control_config;
static dma_channel_config wave_config, guard_config;
static bool run_prepared, pwm_tail_detached;
static timing_status_t status;
static int  wave_dma = -1, guard_dma = -1, boundary_dma = -1, pwm_dma = -1, pwm_control_dma = -1;
static bool trigger_loaded, has_wave_dma, pwm_loaded;
static uint32_t              final_levels, final_dirs;
static timing_trigger_kind_t loaded_kind;
static volatile uint32_t     underrun_snapshot;
static bool                  active_wait, active_boundary;
static struct {
    int dma;
    bool ready, released;
    dma_channel_config config;
} writes[TIMING_WRITE_LIMIT] = {{.dma = -1}, {.dma = -1}};
static size_t write_count, write_result_count, write_slots;
static bool write_loaded, write_deferred;
static uint64_t write_waveform_cycles;
static pio_sm_config write_config;
static const char *write_error;

static uint32_t wave_stall_bit(void) {
    return 1u << (PIO_FDEBUG_TXSTALL_LSB + WAVE_SM);
}

static uint32_t pwm_stall_bit(void) {
    return 1u << (PIO_FDEBUG_TXSTALL_LSB + PWM_SM);
}

static bool pwm_tail_owned(void) {
    if (!pwm.active) {
        return true;
    }
    uintptr_t address = dma_channel_hw_addr((uint) pwm_dma)->read_addr;
    return address >= (uintptr_t) pwm_final_words && address < (uintptr_t) (pwm_final_words + 8) &&
           !dma_channel_is_busy((uint) pwm_control_dma);
}

static bool pwm_dma_error(void) {
    uint32_t flags = DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
    return (dma_channel_hw_addr((uint) pwm_dma)->ctrl_trig |
            dma_channel_hw_addr((uint) pwm_control_dma)->ctrl_trig) &
           flags;
}

static void halt_pwm(void) {
    pio_sm_set_enabled(pio0, PWM_SM, false);
    /* Disable both before abort: neither may re-trigger its partner mid-abort. */
    if (pwm_control_dma >= 0) {
        hw_clear_bits(&dma_channel_hw_addr((uint) pwm_control_dma)->al1_ctrl,
                      DMA_CH0_CTRL_TRIG_EN_BITS);
    }
    if (pwm_dma >= 0) {
        hw_clear_bits(&dma_channel_hw_addr((uint) pwm_dma)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    }
    if (pwm_control_dma >= 0) {
        dma_channel_abort((uint) pwm_control_dma);
        while (dma_channel_is_busy((uint) pwm_control_dma)) {
            tight_loop_contents();
        }
    }
    if (pwm_dma >= 0) {
        /* An E13 in-flight control write may have re-enabled the data channel. */
        hw_clear_bits(&dma_channel_hw_addr((uint) pwm_dma)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
        dma_channel_abort((uint) pwm_dma);
        while (dma_channel_is_busy((uint) pwm_dma)) {
            tight_loop_contents();
        }
    }
    pio_sm_clear_fifos(pio0, PWM_SM);
    pwm.active = false;
    pwm_tail_detached = false;
}

void timing_pwm_stop(void) {
    halt_pwm();
    if (pwm.configured) {
        pio_sm_set_pins_with_mask(pio0, PWM_SM, 0, 1u << pwm.pin);
        pio_sm_set_pindirs_with_mask(pio0, PWM_SM, 1u << pwm.pin, 1u << pwm.pin);
    }
    pwm.final_duty_basis_points = 0;
    pwm.final_mode              = TIMING_PWM_LOW;
    pwm_error                   = NULL;
}

const timing_pwm_t *timing_pwm(void) {
    return &pwm;
}

const char *__not_in_flash_func(timing_pwm_error)(void) {
    return pwm_error;
}

const char *timing_configure_pwm(unsigned pin, uint32_t hz) {
    if (status.active || pwm.active || write_count) {
        return "BUSY";
    }
    if (pin > 29) {
        return "RANGE";
    }
    uint32_t    period_cycles;
    const char *error = timing_pwm_parameters(clock_get_hz(clk_sys), hz, &period_cycles);
    if (error) {
        return error;
    }
    timing_pwm_stop();
    if (pwm.configured) {
        gpio_put(pwm.pin, false);
        gpio_set_dir(pwm.pin, GPIO_IN);
        gpio_set_function(pwm.pin, GPIO_FUNC_SIO);
    }
    pwm = (timing_pwm_t) {
        .configured    = true,
        .pin           = (uint8_t) pin,
        .hz            = hz,
        .period_cycles = period_cycles,
        .final_mode    = TIMING_PWM_LOW,
    };
    if (!pwm_loaded) {
        pio_add_program_at_offset(pio0, &pwm_sequence_program, PWM_OFFSET);
        pwm_loaded = true;
    }
    pio_sm_config config = pwm_sequence_program_get_default_config(PWM_OFFSET);
    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_sideset_pins(&config, pin);
    sm_config_set_out_shift(&config, true, true, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio0, PWM_SM, PWM_OFFSET, &config);
    pio_sm_set_pins_with_mask(pio0, PWM_SM, 0, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio0, PWM_SM, 1u << pin, 1u << pin);
    gpio_disable_pulls(pin);
    pio_gpio_init(pio0, pin);
    /* The 31-pin OUT window wraps modulo 32 and excludes the carrier pin. */
    gpio.out_base = (uint8_t) ((pin + 1u) & 31u);
    pio_sm_set_out_pins(pio0, WAVE_SM, gpio.out_base, 31);
    return NULL;
}

static void release_output_pins(void) {
    gpio.current_levels = idle_levels(&gpio);
    gpio.current_dirs   = idle_dirs(&gpio);
    if (gpio.configured_mask) {
        pio_sm_set_pins_with_mask(pio0, WAVE_SM, gpio.current_levels, gpio.configured_mask);
        pio_sm_set_pindirs_with_mask(pio0, WAVE_SM, gpio.current_dirs, gpio.configured_mask);
    }
    if (sync_pin >= 0) {
        pio_sm_set_pins_with_mask(pio1, MATCH_SM, 0, 1u << sync_pin);
    }
}

static void stop_hardware(void) {
    pio_sm_set_enabled(pio1, BOUNDARY_SM, false);
    pio_sm_set_enabled(pio1, MATCH_SM, false);
    if (boundary_dma >= 0) {
        dma_channel_abort((uint) boundary_dma);
    }
    pio_sm_set_enabled(pio0, WAVE_SM, false);
    if (wave_dma >= 0) {
        dma_channel_abort((uint) wave_dma);
    }
    if (guard_dma >= 0) {
        dma_channel_abort((uint) guard_dma);
    }
    pio_sm_clear_fifos(pio1, BOUNDARY_SM);
    pio_sm_clear_fifos(pio1, MATCH_SM);
    pio_sm_clear_fifos(pio0, WAVE_SM);
    if (sync_pin >= 0) {
        pio_sm_set_pins_with_mask(pio1, MATCH_SM, 0, 1u << sync_pin);
    }
}

void timing_init(void) {
    /* Claiming these SMs does not configure pins or start any hardware. */
    pio_sm_claim(pio0, WAVE_SM);
    pio_sm_claim(pio0, PWM_SM);
    pio_sm_claim(pio1, MATCH_SM);
    pio_sm_claim(pio1, BOUNDARY_SM);
    wave_dma        = dma_claim_unused_channel(true);
    guard_dma       = dma_claim_unused_channel(true);
    boundary_dma    = dma_claim_unused_channel(true);
    pwm_dma         = dma_claim_unused_channel(true);
    pwm_control_dma = dma_claim_unused_channel(true);
    wave_config = dma_channel_get_default_config((uint) wave_dma);
    channel_config_set_transfer_data_size(&wave_config, DMA_SIZE_32);
    channel_config_set_read_increment(&wave_config, true);
    channel_config_set_write_increment(&wave_config, false);
    channel_config_set_dreq(&wave_config, pio_get_dreq(pio0, WAVE_SM, true));
    channel_config_set_high_priority(&wave_config, true);
    channel_config_set_chain_to(&wave_config, (uint) guard_dma);
    guard_config = dma_channel_get_default_config((uint) guard_dma);
    channel_config_set_transfer_data_size(&guard_config, DMA_SIZE_32);
    channel_config_set_read_increment(&guard_config, false);
    channel_config_set_write_increment(&guard_config, false);
    channel_config_set_high_priority(&guard_config, true);
    pio_add_program_at_offset(pio0, &waveform_program, WAVE_OFFSET);
    pio_sm_config config = waveform_program_get_default_config(WAVE_OFFSET);
    sm_config_set_out_pins(&config, 0, 30);
    sm_config_set_out_shift(&config, true, true, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio0, WAVE_SM, WAVE_OFFSET, &config);
}

const timing_gpio_t *timing_gpio(void) {
    return &gpio;
}

int timing_sync_pin(void) {
    return sync_pin;
}

const timing_spi_t *timing_spi(void) {
    return &spi;
}

const char *timing_configure_gpio(unsigned pin, timing_drive_t drive, bool idle) {
    if (status.active || write_count) {
        return "BUSY";
    }
    if (pin > 29u || (unsigned) drive > TIMING_OPEN_SOURCE) {
        return "RANGE";
    }
    uint32_t bit = 1u << pin;
    /* Release before changing the output latch, including drive-mode changes. */
    pio_sm_set_pindirs_with_mask(pio0, WAVE_SM, 0, bit);
    gpio.configured_mask |= bit;
    gpio.open_drain_mask = (gpio.open_drain_mask & ~bit) | (drive == TIMING_OPEN_DRAIN ? bit : 0);
    gpio.open_source_mask =
        (gpio.open_source_mask & ~bit) | (drive == TIMING_OPEN_SOURCE ? bit : 0);
    gpio.idle_high_mask = (gpio.idle_high_mask & ~bit) | (idle ? bit : 0);
    gpio.current_levels = (gpio.current_levels & ~bit) | (idle_levels(&gpio) & bit);
    gpio.current_dirs   = (gpio.current_dirs & ~bit) | (idle_dirs(&gpio) & bit);
    pio_sm_set_pins_with_mask(pio0, WAVE_SM, idle_levels(&gpio), bit);
    pio_sm_set_pindirs_with_mask(pio0, WAVE_SM, idle_dirs(&gpio), bit);
    gpio_disable_pulls(pin);
    gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
    gpio_set_oeover(pin, GPIO_OVERRIDE_NORMAL);
    pio_gpio_init(pio0, pin);
    return NULL;
}

const char *timing_configure_sync(unsigned pin) {
    if (status.active || write_count) {
        return "BUSY";
    }
    if (pin > 29u) {
        return "RANGE";
    }
    if (sync_pin >= 0) {
        pio_sm_set_pindirs_with_mask(pio1, MATCH_SM, 0, 1u << sync_pin);
        gpio_set_dir((uint) sync_pin, GPIO_IN);
        gpio_set_function((uint) sync_pin, GPIO_FUNC_SIO);
    }
    sync_pin                                                 = (int) pin;
    pio0->instr_mem[WAVE_OFFSET + waveform_offset_wait_sync] = pio_encode_wait_gpio(true, pin);
    pio_sm_set_pins_with_mask(pio1, MATCH_SM, 0, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio1, MATCH_SM, 1u << pin, 1u << pin);
    gpio_disable_pulls(pin);
    gpio_set_outover(pin, GPIO_OVERRIDE_NORMAL);
    gpio_set_oeover(pin, GPIO_OVERRIDE_NORMAL);
    pio_gpio_init(pio1, pin);
    return NULL;
}

const char *timing_configure_spi(const timing_spi_t *configuration) {
    if (status.active || write_count) {
        return "BUSY";
    }
    if (!configuration || configuration->cs > 29 || configuration->sck > 29 ||
        configuration->data > 29 || configuration->mode > 3 || !configuration->max_hz ||
        configuration->max_hz > 10000000u) {
        return "RANGE";
    }
    if (configuration->cs == configuration->sck || configuration->cs == configuration->data ||
        configuration->sck == configuration->data) {
        return "PIN_CONFLICT";
    }
    if (spi.configured) {
        gpio_set_function(spi.cs, GPIO_FUNC_SIO);
        gpio_set_function(spi.sck, GPIO_FUNC_SIO);
        gpio_set_function(spi.data, GPIO_FUNC_SIO);
    }
    spi                  = *configuration;
    spi.configured       = true;
    const uint8_t pins[] = {spi.cs, spi.sck, spi.data};
    for (size_t i = 0; i < 3; ++i) {
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
    }
    return NULL;
}

static void unload_trigger(void) {
    if (trigger_loaded) {
        if (loaded_kind == TIMING_TRIGGER_I2C) {
            pio_remove_program(pio1, &i2c_matcher_program, 0);
            pio_remove_program(pio1, &i2c_boundary_program, 11);
        } else if (loaded_kind == TIMING_TRIGGER_SPI) {
            pio_remove_program(pio1, &spi_matcher_program, 0);
            pio_remove_program(pio1, &spi_boundary_program, 12);
        } else {
            pio_remove_program(pio1, &gpio_matcher_program, 0);
        }
        trigger_loaded = false;
    }
}

static void load_trigger(timing_trigger_kind_t kind) {
    if (trigger_loaded && loaded_kind == kind) {
        return;
    }
    unload_trigger();
    if (kind == TIMING_TRIGGER_I2C) {
        pio_add_program_at_offset(pio1, &i2c_matcher_program, 0);
        pio_add_program_at_offset(pio1, &i2c_boundary_program, 11);
    } else if (kind == TIMING_TRIGGER_SPI) {
        pio_add_program_at_offset(pio1, &spi_matcher_program, 0);
        pio_add_program_at_offset(pio1, &spi_boundary_program, 12);
    } else {
        pio_add_program_at_offset(pio1, &gpio_matcher_program, 0);
    }
    trigger_loaded = true;
    loaded_kind    = kind;
}

static void preload_register(PIO pio, uint sm, uint32_t value, enum pio_src_dest dest) {
    pio_sm_put(pio, sm, value);
    pio_sm_exec(pio, sm, pio_encode_pull(false, true));
    pio_sm_exec(pio, sm, pio_encode_mov(dest, pio_osr));
}


static void __not_in_flash_func(snapshot_write_releases)(void) {
    for (unsigned i = 0; i < write_count; ++i) {
        uint channel = (uint) writes[i].dma;
        uint32_t flags = dma_channel_hw_addr(channel)->ctrl_trig;
        writes[i].released |= !dma_channel_is_busy(channel) &&
                              dma_channel_hw_addr(channel)->transfer_count == 0 &&
                              !(flags & (DMA_CH0_CTRL_TRIG_READ_ERROR_BITS |
                                         DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS));
    }
}

static void halt_writes(void) {
    snapshot_write_releases();
    /* First remove every producer, then quiesce every consumer before returning
     * ownership of the controller registers. Abort alone is not a BUSY barrier
     * on RP2040 (E13 permits a final in-flight transfer after abort clears). */
    for (unsigned i = 0; i < write_count; ++i) {
        pio_sm_set_enabled(pio0, WRITE_SM + i, false);
        hw_clear_bits(&dma_channel_hw_addr((uint) writes[i].dma)->al1_ctrl,
                      DMA_CH0_CTRL_TRIG_EN_BITS);
    }
    for (unsigned i = 0; i < write_count; ++i) {
        uint channel = (uint) writes[i].dma;
        dma_channel_abort(channel);
        while (dma_channel_is_busy(channel)) {
            tight_loop_contents();
        }
        pio_sm_clear_fifos(pio0, WRITE_SM + i);
        pio_sm_restart(pio0, WRITE_SM + i);
        pio0->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + WRITE_SM + i)) |
                      (1u << (PIO_FDEBUG_TXSTALL_LSB + WRITE_SM + i));
        writes[i].ready = false;
    }
    write_count = 0;
    write_deferred = false;
    write_waveform_cycles = 0;
}

void timing_write_stop(void) {
    halt_writes();
    write_result_count = 0;
    write_error = NULL;
}

const char *__not_in_flash_func(timing_write_error)(void) {
    snapshot_write_releases();
    for (unsigned i = 0; !write_error && i < write_count; ++i) {
        uint32_t flags = dma_channel_hw_addr((uint) writes[i].dma)->ctrl_trig;
        if (flags & (DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)) {
            write_error = "TIMING_ERROR";
        } else if (pio_sm_get_pc(pio0, WRITE_SM + i) == WRITE_OFFSET + scheduled_write_offset_missed) {
            write_error = "MISSED_DEADLINE";
        }
    }
    if (write_error) {
        halt_writes();
    }
    return write_error;
}

bool __not_in_flash_func(timing_write_released)(unsigned index) {
    timing_write_error();
    return index < write_result_count && writes[index].released;
}

const char *__not_in_flash_func(timing_write_ready)(unsigned index) {
    const char *error = timing_write_error();
    if (error) {
        return error;
    }
    if (index >= write_count) {
        return "RANGE";
    }
    if (writes[index].ready) {
        return "BAD_REQUEST";
    }
    unsigned pc = pio_sm_get_pc(pio0, WRITE_SM + index);
    if (pc > WRITE_OFFSET + scheduled_write_offset_cutoff && pc < ORIGIN_OFFSET) {
        write_error = "MISSED_DEADLINE";
        halt_writes();
        return write_error;
    }
    writes[index].ready = true;
    pio_sm_put(pio0, WRITE_SM + index, 1u);
    /* A racing cutoff can reject this permit, but can never release late. */
    return timing_write_error();
}

const char *timing_write_arm(int pin, bool rising, const uint32_t *at_cycles,
                              volatile uint32_t *const *enable_registers, size_t count,
                              uint64_t waveform_cycles) {
    if (pin < -1 || pin > 29 || !at_cycles || !enable_registers || !count ||
        count > TIMING_WRITE_LIMIT || waveform_cycles > (UINT64_C(1) << 32) ||
        (waveform_cycles && pin != -1)) {
        return "RANGE";
    }
    for (size_t i = 0; i < count; ++i) {
        if (at_cycles[i] < TIMING_WRITE_MIN_CYCLES || (i && at_cycles[i] <= at_cycles[i - 1u]) ||
            !enable_registers[i] || ((uintptr_t) enable_registers[i] & 3u)) {
            return "RANGE";
        }
    }
    if (status.active || write_count) {
        return "BUSY";
    }
    if (!run_prepared || count > write_slots) {
        return "RESOURCE_BUSY";
    }
    if (pwm_error) {
        return pwm_error;
    }
    write_deferred = pin == -1;
    write_waveform_cycles = waveform_cycles;
    pio0->instr_mem[WRITE_OFFSET] =
        write_deferred ? pio_encode_nop() : pio_encode_wait_gpio(rising, (uint) pin);
    if (!write_deferred) {
        /* Clear before the forced opposite WAIT sees a stale matcher latch. */
        if (pin == sync_pin) {
            pio_sm_set_pins_with_mask(pio1, MATCH_SM, 0, 1u << pin);
        }
        hw_clear_bits(&pio0->input_sync_bypass, 1u << pin);
    }
    write_error = NULL;
    write_count = write_result_count = count;
    uint32_t mask = 0;
    for (unsigned i = 0; i < count; ++i) {
        unsigned sm = WRITE_SM + i;
        writes[i].ready = writes[i].released = false;
        pio_sm_init(pio0, sm, waveform_cycles ? ORIGIN_OFFSET : WRITE_OFFSET, &write_config);
        preload_register(pio0, sm, at_cycles[i] - 7u, pio_x);
        if (waveform_cycles) {
            preload_register(pio0, sm, (uint32_t) (waveform_cycles - 1u), pio_y);
        }
        dma_channel_configure((uint) writes[i].dma, &writes[i].config, enable_registers[i],
                              &pio0->rxf[sm], 1u, true);
        if (!write_deferred) {
            /* A stalled SMx_INSTR does not advance PC. Keep this forced WAIT last. */
            pio_sm_exec(pio0, sm, pio_encode_wait_gpio(!rising, (uint) pin));
        }
        mask |= 1u << sm;
    }
    if (!write_deferred) {
        pio_enable_sm_mask_in_sync(pio0, mask);
    }
    return NULL;
}

const char *__not_in_flash_func(timing_write_reference)(void) {
    const char *error = timing_write_error();
    if (error) {
        return error;
    }
    if (!write_count || !write_deferred || write_waveform_cycles) {
        return "BAD_REQUEST";
    }
    write_deferred = false;
    pio_enable_sm_mask_in_sync(pio0, ((1u << write_count) - 1u) << WRITE_SM);
    return NULL;
}

static void arm_trigger(const timing_trigger_t *trigger, uint32_t expected, uint32_t bits,
                        uint32_t boundary_budget) {
    load_trigger(trigger->kind);
    if (trigger->kind == TIMING_TRIGGER_GPIO) {
        /* The staged pin is validated before start, and only configured here.
         * Leave it input-only with no pulls, including after disarming. */
        gpio_set_dir(trigger->pin, GPIO_IN);
        gpio_disable_pulls(trigger->pin);
        gpio_set_function(trigger->pin, GPIO_FUNC_SIO);
        hw_clear_bits(&pio1->input_sync_bypass, 1u << trigger->pin);
        pio1->instr_mem[0]   = pio_encode_wait_gpio(!trigger->rising, trigger->pin);
        pio1->instr_mem[1]   = pio_encode_wait_gpio(trigger->rising, trigger->pin);
        pio_sm_config config = gpio_matcher_program_get_default_config(0);
        sm_config_set_set_pins(&config, (uint) sync_pin, 1);
        pio_sm_init(pio1, MATCH_SM, 0, &config);
        pio_sm_set_enabled(pio1, MATCH_SM, true);
        return;
    }
    pio_sm_config matcher, boundary;
    uint          boundary_offset;
    if (trigger->kind == TIMING_TRIGGER_I2C) {
        matcher  = i2c_matcher_program_get_default_config(0);
        boundary = i2c_boundary_program_get_default_config(11);
        sm_config_set_in_pins(&matcher, trigger->sda);
        sm_config_set_in_pins(&boundary, trigger->sda);
        sm_config_set_jmp_pin(&boundary, trigger->scl);
        boundary_offset = 11;
    } else {
        bool inactive       = spi.cs_active_low;
        bool sample_high    = ((spi.mode & 1u) == ((spi.mode >> 1) & 1u));
        pio1->instr_mem[0]  = pio_encode_wait_gpio(inactive, spi.cs);
        pio1->instr_mem[1]  = pio_encode_wait_gpio(!inactive, spi.cs);
        pio1->instr_mem[4]  = pio_encode_wait_gpio(!sample_high, spi.sck);
        pio1->instr_mem[5]  = pio_encode_wait_gpio(sample_high, spi.sck);
        pio1->instr_mem[12] = pio_encode_wait_gpio(!inactive, spi.cs);
        pio1->instr_mem[13] = pio_encode_wait_gpio(inactive, spi.cs);
        matcher             = spi_matcher_program_get_default_config(0);
        boundary            = spi_boundary_program_get_default_config(12);
        sm_config_set_in_pins(&matcher, spi.data);
        boundary_offset = 12;
    }
    sm_config_set_set_pins(&matcher, (uint) sync_pin, 1);
    sm_config_set_in_shift(&matcher, false, false, 32);
    sm_config_set_in_shift(&boundary, false, false, 32);
    pio_sm_init(pio1, MATCH_SM, 0, &matcher);
    pio_sm_init(pio1, BOUNDARY_SM, boundary_offset, &boundary);
    preload_register(pio1, MATCH_SM, expected, pio_y);
    preload_register(pio1, MATCH_SM, bits - 1u, pio_osr);
    if (trigger->kind == TIMING_TRIGGER_I2C) {
        preload_register(pio1, BOUNDARY_SM, pio_encode_jmp(i2c_matcher_offset_start), pio_x);
        preload_register(pio1, BOUNDARY_SM, pio_encode_jmp(i2c_matcher_offset_idle), pio_y);
    } else {
        preload_register(pio1, BOUNDARY_SM, pio_encode_jmp(spi_matcher_offset_idle), pio_x);
    }
    pio1->fdebug              = 1u << (PIO_FDEBUG_RXSTALL_LSB + BOUNDARY_SM);
    dma_channel_config config = dma_channel_get_default_config((uint) boundary_dma);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(pio1, BOUNDARY_SM, false));
    channel_config_set_high_priority(&config, true);
    dma_channel_configure((uint) boundary_dma, &config, &pio1->sm[MATCH_SM].instr,
                          &pio1->rxf[BOUNDARY_SM], boundary_budget, true);
    pio_enable_sm_mask_in_sync(pio1, (1u << MATCH_SM) | (1u << BOUNDARY_SM));
}

static void fail_timing(const char *error) {
    halt_writes();
    write_error = error;
    stop_hardware();
    release_output_pins();
    timing_pwm_stop();
    status.active  = false;
    status.waiting = false;
    status.error   = error;
}

static void start_pwm(const timing_block_t *block) {
    halt_pwm(); /* Keep the previous pin level through the startup prefetch. */
    const timing_pwm_segment_t *final_segment = &block->pwm_segments[block->pwm_count - 1u];
    for (unsigned i = 0; i < final_segment->word_count; ++i) {
        pwm_final_words[i] = final_segment->words[i];
    }
    for (unsigned i = 0; i < 4u; ++i) {
        pwm_tail_descriptors[0][i] = pwm_tail_descriptors[1][i] =
            pwm_descriptors[block->pwm_tail_descriptor][i];
    }
    pio_sm_restart(pio0, PWM_SM);
    pio_sm_exec(pio0, PWM_SM, pio_encode_jmp(PWM_OFFSET));
    for (unsigned i = 0; i < 8u; ++i) {
        pio_sm_put(pio0, PWM_SM, block->pwm_prefill[i]);
    }
    dma_channel_configure((uint) pwm_control_dma, &pwm_control_config,
                          &dma_channel_hw_addr((uint) pwm_dma)->al3_ctrl,
                          pwm_descriptors[block->pwm_descriptor], 4u, true);
    pio0->fdebug                = pwm_stall_bit();
    pwm.active                  = true;
    pwm.final_duty_basis_points = block->pwm_final_duty;
    pwm.final_mode              = block->pwm_final_mode;
}

static const char *validate_pwm_block(const timing_block_t *block) {
    if (!pwm.configured) {
        return "UNCONFIGURED";
    }
    if (!block->pwm_segments || ((uintptr_t) block->pwm_segments & 31u) ||
        block->pwm_count > TIMING_MAX_PWM_SEGMENTS || block->waits ||
        block->pwm_final_duty > 10000u || (unsigned) block->pwm_final_mode > TIMING_PWM_FLOATING ||
        (block->wave_start_countdown != 1u && block->wave_start_countdown != 2u)) {
        return "RANGE";
    }
    for (size_t i = 0; i < block->pwm_count; ++i) {
        const timing_pwm_segment_t *segment = &block->pwm_segments[i];
        if ((segment->word_count != 4u && segment->word_count != 8u) || !segment->periods ||
            segment->periods > UINT32_MAX / segment->word_count) {
            return "RANGE";
        }
    }
    return NULL;
}

void timing_release_run(void) {
    timing_write_stop();
    stop_hardware();
    status.active = status.waiting = false;
    if (pwm.active && !pwm_tail_detached) {
        if (!pwm_tail_owned() ||
            dma_channel_hw_addr((uint) pwm_dma)->transfer_count <= (1u << 20)) {
            halt_pwm();
        } else {
            dma_channel_set_read_addr((uint) pwm_control_dma, pwm_tail_descriptors, false);
            pwm_tail_detached = true;
        }
    }
    for (unsigned i = 0; i < write_slots; ++i) {
        dma_channel_unclaim((uint) writes[i].dma);
        pio_sm_unclaim(pio0, WRITE_SM + i);
        writes[i].dma = -1;
    }
    write_slots = 0;
    if (write_loaded) {
        pio_remove_program(pio0, &scheduled_write_program, WRITE_OFFSET);
        pio_remove_program(pio0, &scheduled_origin_program, ORIGIN_OFFSET);
        write_loaded = false;
    }
    run_prepared = false;
}

const char *timing_prepare_run(timing_block_t *blocks, size_t count, unsigned slots) {
    if (run_prepared || status.active || write_count || (pwm.active && !pwm_tail_detached)) {
        return "BUSY";
    }
    if ((!blocks && count) || slots > TIMING_WRITE_LIMIT) {
        return "RANGE";
    }
    if (pwm_error) {
        return pwm_error;
    }
    size_t segments = 0;
    for (size_t i = 0; i < count; ++i) {
        timing_block_t *block = &blocks[i];
        if (!block->count) {
            continue;
        }
        if (!block->tokens || block->count > TIMING_MAX_TOKENS || block->count % 3u) {
            return "RANGE";
        }
        if (block->pwm_count) {
            const char *error = validate_pwm_block(block);
            if (error) {
                return error;
            }
            segments += block->pwm_count;
            if (segments > TIMING_MAX_PWM_SEGMENTS) {
                return "RANGE";
            }
        }
    }
    if (slots) {
        for (unsigned i = 0; i < slots; ++i) {
            if (pio_sm_is_claimed(pio0, WRITE_SM + i)) {
                return "RESOURCE_BUSY";
            }
        }
        if (!pio_can_add_program_at_offset(pio0, &scheduled_write_program, WRITE_OFFSET) ||
            !pio_can_add_program_at_offset(pio0, &scheduled_origin_program, ORIGIN_OFFSET)) {
            return "RESOURCE_BUSY";
        }
        for (unsigned i = 0; i < slots; ++i) {
            int channel = dma_claim_unused_channel(false);
            if (channel < 0) {
                timing_release_run();
                return "RESOURCE_BUSY";
            }
            pio_sm_claim(pio0, WRITE_SM + i);
            writes[i].dma = channel;
            ++write_slots;
            dma_channel_config *config = &writes[i].config;
            *config = dma_channel_get_default_config((uint) channel);
            channel_config_set_transfer_data_size(config, DMA_SIZE_32);
            channel_config_set_read_increment(config, false);
            channel_config_set_write_increment(config, false);
            channel_config_set_high_priority(config, true);
            channel_config_set_dreq(config, pio_get_dreq(pio0, WRITE_SM + i, false));
            config->ctrl |= DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
        }
        pio_add_program_at_offset(pio0, &scheduled_write_program, WRITE_OFFSET);
        pio_add_program_at_offset(pio0, &scheduled_origin_program, ORIGIN_OFFSET);
        pio0->instr_mem[ORIGIN_OFFSET + 1u] = pio_encode_jmp(WRITE_OFFSET + 1u);
        write_config = scheduled_write_program_get_default_config(WRITE_OFFSET);
        write_loaded = true;
    }
    dma_channel_config data = dma_channel_get_default_config((uint) pwm_dma);
    channel_config_set_transfer_data_size(&data, DMA_SIZE_32);
    channel_config_set_read_increment(&data, true);
    channel_config_set_write_increment(&data, false);
    channel_config_set_dreq(&data, pio_get_dreq(pio0, PWM_SM, true));
    channel_config_set_high_priority(&data, true);
    channel_config_set_chain_to(&data, (uint) pwm_control_dma);
    pwm_control_config = dma_channel_get_default_config((uint) pwm_control_dma);
    channel_config_set_transfer_data_size(&pwm_control_config, DMA_SIZE_32);
    channel_config_set_read_increment(&pwm_control_config, true);
    channel_config_set_write_increment(&pwm_control_config, true);
    channel_config_set_ring(&pwm_control_config, true, 4u);
    channel_config_set_high_priority(&pwm_control_config, true);
    pwm_control_config.ctrl |= DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
    size_t used = 0;
    for (size_t b = 0; b < count; ++b) {
        timing_block_t *block = &blocks[b];
        if (!block->count || !block->pwm_count) {
            continue;
        }
        block->pwm_descriptor = (uint16_t) used;
        unsigned prefilled = 0;
        for (size_t i = 0; i < block->pwm_count; ++i) {
            const timing_pwm_segment_t *segment = &block->pwm_segments[i];
            uint32_t words = segment->periods * segment->word_count;
            uint32_t skip = words < 8u - prefilled ? words : 8u - prefilled;
            for (uint32_t j = 0; j < skip; ++j) {
                block->pwm_prefill[prefilled++] = segment->words[j % segment->word_count];
            }
            channel_config_set_ring(&data, false, segment->word_count == 4u ? 4u : 5u);
            bool tail = i + 1u == block->pwm_count;
            const uint32_t *source = tail ? pwm_final_words : segment->words;
            uint32_t *descriptor = pwm_descriptors[used];
            if (words > skip) {
                descriptor[0] = data.ctrl | DMA_CH0_CTRL_TRIG_READ_ERROR_BITS |
                                DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
                descriptor[1] = (uintptr_t) &pio0->txf[PWM_SM];
                descriptor[2] = words - skip;
                descriptor[3] = (uintptr_t) (source + skip % segment->word_count);
                ++used;
            }
            if (tail) {
                /* Guard is a complete tail, not its startup-shortened descriptor. */
                block->pwm_tail_descriptor = (uint16_t) used;
                descriptor = pwm_descriptors[used++];
                descriptor[0] = data.ctrl | DMA_CH0_CTRL_TRIG_READ_ERROR_BITS |
                                DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
                descriptor[1] = (uintptr_t) &pio0->txf[PWM_SM];
                descriptor[2] = words;
                descriptor[3] = (uintptr_t) pwm_final_words;
                while (prefilled < 8u) {
                    block->pwm_prefill[prefilled] = segment->words[prefilled % segment->word_count];
                    ++prefilled;
                }
            }
        }
    }
    run_prepared = true;
    return NULL;
}

static const char *prepare_trigger(const timing_trigger_t *trigger, uint32_t remaining_run_us,
                                   uint32_t *expected, uint32_t *bit_count,
                                   uint32_t *boundary_budget) {
    if (sync_pin < 0) {
        return "UNCONFIGURED";
    }
    if (!trigger || trigger->kind == TIMING_TRIGGER_NONE) {
        return "BAD_REQUEST";
    }
    if (trigger->kind == TIMING_TRIGGER_I2C) {
        if (trigger->sda > 28 || trigger->scl != trigger->sda + 1u) {
            return "RANGE";
        }
        const char *error = timing_pack_i2c(trigger->address, trigger->read, trigger->prefix,
                                            trigger->prefix_length, expected, bit_count);
        *boundary_budget =
            (uint32_t) (((uint64_t) remaining_run_us * 800000u + 999999u) / 1000000u + 4u);
        return error;
    }
    if (trigger->kind == TIMING_TRIGGER_SPI) {
        if (!spi.configured) {
            return "UNCONFIGURED";
        }
        const char *error  = timing_pack_spi(trigger->prefix, trigger->prefix_length, spi.lsb_first,
                                             expected, bit_count);
        uint64_t    budget = ((uint64_t) clock_get_hz(clk_sys) * remaining_run_us) / 64000000u + 4u;
        if (budget > UINT32_MAX) {
            return "RANGE";
        }
        *boundary_budget = (uint32_t) budget;
        return error;
    }
    if (trigger->kind == TIMING_TRIGGER_GPIO) {
        return trigger->pin > 29 ? "RANGE" : NULL;
    }
    return "BAD_REQUEST";
}

static void start_wave_stream(const timing_block_t *block) {
    size_t prefill = block->count < 8u ? block->count : 8u;
    for (size_t i = 0; i < prefill; ++i) {
        pio_sm_put(pio0, WAVE_SM, block->tokens[i]);
    }
    pio0->fdebug      = wave_stall_bit();
    underrun_snapshot = 0;
    has_wave_dma      = block->count > prefill;
    if (!has_wave_dma) {
        return;
    }
    /* Snapshot the sticky flag after the LAST FIFO write. An earlier stall
     * cannot be hidden by the normal final stall when execution services late.
     * Conservatively reject if snapshot service itself is delayed to end. */
    dma_channel_configure((uint) guard_dma, &guard_config, &underrun_snapshot, &pio0->fdebug, 1,
                          false);
    dma_channel_configure((uint) wave_dma, &wave_config, &pio0->txf[WAVE_SM],
                          block->tokens + prefill, (uint32_t) (block->count - prefill), true);
}

const char *timing_start(const timing_block_t *block, const timing_trigger_t *trigger,
                         uint32_t remaining_run_us) {
    if (status.active) {
        return "BUSY";
    }
    if (!block || !block->tokens || !block->count || block->count > TIMING_MAX_TOKENS ||
        block->count % 3u || !remaining_run_us || remaining_run_us > 60000000u) {
        return "RANGE";
    }
    if (pwm_error) {
        return pwm_error;
    }
    if (!run_prepared) {
        return "BAD_REQUEST";
    }
    if (write_waveform_cycles &&
        (block->waits || block->completion_cycles != write_waveform_cycles)) {
        return "BAD_REQUEST";
    }
    uint32_t expected        = 0;
    uint32_t bit_count       = 0;
    uint32_t boundary_budget = 0;
    if (block->waits) {
        const char *error =
            prepare_trigger(trigger, remaining_run_us, &expected, &bit_count, &boundary_budget);
        if (error) {
            return error;
        }
    } else if (trigger && trigger->kind != TIMING_TRIGGER_NONE) {
        return "BAD_REQUEST";
    }
    stop_hardware();
    status          = (timing_status_t) {.active = true};
    final_levels    = block->final_levels;
    final_dirs      = block->final_dirs;
    active_wait     = block->waits;
    active_boundary = active_wait && trigger->kind != TIMING_TRIGGER_GPIO;
    pio_sm_restart(pio0, WAVE_SM);
    if (block->pwm_count) {
        start_pwm(block);
        /* Inject only the countdown, never an output write. With seed 1 the
         * first direction edge is cycle 6; seed 2 aligns push-pull OUTs. */
        pio_sm_put(pio0, WAVE_SM, block->wave_start_countdown);
        pio_sm_exec(pio0, WAVE_SM, pio_encode_pull(false, true));
        pio_sm_exec(pio0, WAVE_SM, pio_encode_jmp(WAVE_OFFSET + 3u));
    } else {
        pio_sm_exec(pio0, WAVE_SM, pio_encode_jmp(WAVE_OFFSET));
    }
    start_wave_stream(block);
    if (active_wait) {
        arm_trigger(trigger, expected, bit_count, boundary_budget);
    }
    uint32_t mask = (1u << WAVE_SM) | (block->pwm_count ? 1u << PWM_SM : 0u);
    if (write_deferred && write_waveform_cycles) {
        mask |= ((1u << write_count) - 1u) << WRITE_SM;
        write_deferred = false;
        write_waveform_cycles = 0;
    }
    pio_enable_sm_mask_in_sync(pio0, mask);
    return NULL;
}

void __not_in_flash_func(timing_service)(void) {
    if (write_count) {
        const char *error = timing_write_error();
        if (error) {
            fail_timing(error);
            return;
        }
    }
    if (pwm.active) {
        if ((pio0->fdebug & pwm_stall_bit()) || pwm_dma_error()) {
            fail_timing("TIMING_ERROR");
            pwm_error = "TIMING_ERROR"; /* Public stop clears; failure remains latched. */
            return;
        }
        if (pwm_tail_owned() && dma_channel_hw_addr((uint) pwm_dma)->transfer_count > (1u << 20)) {
            /* Tail lasts >=68 seconds at 125 MHz. The remaining-word guard
             * leaves >8 ms even at one DMA word/clock, preventing a chain
             * race during the <=1 ms service window. No edge is CPU-dispatched. */
            dma_channel_set_read_addr((uint) pwm_control_dma, pwm_tail_descriptors, false);
            pwm_tail_detached = true;
        }
    }
    if (!status.active) {
        return;
    }
    if (active_wait) {
        status.matched |= gpio_get((uint) sync_pin);
        status.waiting = !status.matched &&
                         pio_sm_get_pc(pio0, WAVE_SM) == WAVE_OFFSET + waveform_offset_wait_sync;
        if (active_boundary && ((pio1->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + BOUNDARY_SM))) ||
                                !dma_channel_hw_addr((uint) boundary_dma)->transfer_count)) {
            fail_timing("TRIGGER_OVERFLOW");
            return;
        }
    }
    bool dma_finished =
        !has_wave_dma ||
        (!dma_channel_is_busy((uint) wave_dma) && !dma_channel_is_busy((uint) guard_dma) &&
         dma_channel_hw_addr((uint) guard_dma)->transfer_count == 0);
    if ((has_wave_dma && (underrun_snapshot & wave_stall_bit())) ||
        (!dma_finished && (pio0->fdebug & wave_stall_bit()))) {
        fail_timing("TIMING_ERROR");
        return;
    }
    /* Completion releases staged buffers only after DMA owns the retained tail. */
    if (dma_finished && pwm_tail_owned() && pio_sm_is_tx_fifo_empty(pio0, WAVE_SM) &&
        pio_sm_get_pc(pio0, WAVE_SM) == WAVE_OFFSET + 1u && (pio0->fdebug & wave_stall_bit())) {
        if (active_wait && !status.matched) {
            fail_timing("TIMING_ERROR");
            return;
        }
        stop_hardware();
        gpio.current_levels = final_levels;
        gpio.current_dirs   = final_dirs;
        if (pwm.active && pwm.final_mode != TIMING_PWM_CARRIER) {
            halt_pwm();
        }
        status.active  = false;
        status.waiting = false;
        status.done    = true;
    }
}

timing_status_t __not_in_flash_func(timing_status)(void) {
    return status;
}

void timing_stop(void) {
    timing_write_stop();
    stop_hardware();
    release_output_pins();
    status.active  = false;
    status.waiting = false;
}
