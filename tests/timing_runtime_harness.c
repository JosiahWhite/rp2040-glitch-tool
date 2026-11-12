/* Build by test_pio.py against assembled program data and the production runtime. */
#define TIMING_RUNTIME_TEST
#include "../timing_hw.c"

static uint32_t test_tokens[TIMING_MAX_TOKENS];
_Alignas(32) static timing_pwm_segment_t test_segments[TIMING_MAX_PWM_SEGMENTS];
static timing_block_t test_block;

static void compile_test(const timing_instruction_t *steps, size_t count, bool with_pwm) {
    const char *error = with_pwm ?
        timing_compile_pwm(timing_gpio(), timing_pwm(), steps, count,
                           timing_pwm()->final_duty_basis_points, test_tokens, TIMING_MAX_TOKENS,
                           test_segments, TIMING_MAX_PWM_SEGMENTS, &test_block) :
        timing_compile(timing_gpio(), steps, count, test_tokens, TIMING_MAX_TOKENS, &test_block);
    assert(!error);
    staged_begin=(uintptr_t)test_segments;
    staged_end=(uintptr_t)(test_segments+TIMING_MAX_PWM_SEGMENTS);
    assert(!(staged_begin & 31u));
}
static void compile_start(const timing_instruction_t *steps, size_t count, bool with_pwm) {
    if (!write_count) timing_release_run();
    compile_test(steps, count, with_pwm);
    if (!run_prepared) assert(!timing_prepare_run(&test_block, 1, 0));
    assert(!timing_start(&test_block, NULL, 1000000));
}
static void run_cycles(uint count) {
    for (uint i=0; i<count; ++i) {
        native_tick(1);
        if (i % 23u == 0) timing_service();
    }
    timing_service();
    assert(!timing_status().error);
    assert(!timing_pwm_error());
}
static void assert_state_at(uint64_t cycle, bool high, bool drive) {
    for (size_t i=0; i<native_edge_count; ++i) if (native_edges[i].cycle==cycle) {
        assert(!!(native_edges[i].levels & (1u<<13))==high);
        assert(!!(native_edges[i].dirs & (1u<<13))==drive);
        return;
    }
    assert(!"missing physical PWM transition");
}
static void assert_carrier(uint64_t begin, uint64_t end, uint high, uint period) {
    uint64_t rise=0, previous_rise=0;
    bool previous=false, have_rise=false;
    unsigned periods=0;
    for (size_t i=0; i<native_edge_count; ++i) {
        uint64_t cycle=native_edges[i].cycle;
        bool level=!!(native_edges[i].levels & (1u<<13));
        if (cycle < begin) { previous=level; continue; }
        if (cycle >= end) break;
        if (level != previous) {
            if (level) {
                if (have_rise) assert(cycle-previous_rise==period);
                previous_rise=rise=cycle; have_rise=true;
            } else if (have_rise) { assert(cycle-rise==high); ++periods; }
        }
        previous=level;
    }
    assert(periods>=2u);
}
static volatile uint32_t enables[2];
static volatile uint32_t *enable_registers[2] = {&enables[0], &enables[1]};

static void arm_writes(bool rising, const uint32_t *times, size_t count) {
    timing_stop();
    timing_release_run();
    timing_pwm_stop();
    assert(!timing_configure_gpio(11, TIMING_PUSH_PULL, rising));
    native_tick(4); /* Synchronizers run while engines are disabled. */
    enables[0] = enables[1] = 3u; /* ENABLE | TX_CMD_BLOCK */
    assert(!timing_prepare_run(NULL, 0, (unsigned) count));
    assert(!timing_write_arm(11, rising, times, enable_registers, count, 0));
}

static uint64_t write_reference(bool rising) {
    native_output(rising ? 0 : 1u << 11, 1u << 11, false);
    native_tick(4);
    assert(pio_sm_get_pc(pio0, WRITE_SM) == WRITE_OFFSET);
    assert(!native_sm(pio0, WRITE_SM)->has_forced);
    native_output(rising ? 1u << 11 : 0, 1u << 11, false);
    return native_cycle + 2u;
}

static void scheduled_runtime(void) {
    dma_fault = false;
    timing_stop();
    timing_release_run();
    timing_pwm_stop();
    const uint32_t times[] = {16u, 100u};
    uint32_t levels = native_levels, dirs = native_dirs;
    assert(!strcmp(timing_write_arm(30, false, times, enable_registers, 1, 0), "RANGE"));
    assert(!strcmp(timing_write_arm(-2, false, times, enable_registers, 1, 0), "RANGE"));
    assert(!strcmp(timing_write_arm(11, false, NULL, enable_registers, 1, 0), "RANGE"));
    assert(!strcmp(timing_write_arm(11, false, times, NULL, 1, 0), "RANGE"));
    assert(!strcmp(timing_write_arm(11, false, times, enable_registers, 0, 0), "RANGE"));
    assert(!strcmp(timing_write_arm(11, false, times, enable_registers, 3, 0), "RANGE"));
    const uint32_t invalid[][2] = {{7, 100}, {16, 16}, {16, 8}};
    for (size_t i = 0; i < 3; ++i)
        assert(!strcmp(timing_write_arm(11, false, invalid[i], enable_registers, 2, 0), "RANGE"));
    volatile uint32_t *bad[] = {NULL, (volatile uint32_t *) ((uintptr_t) &enables[1] + 1u)};
    for (size_t i = 0; i < 2; ++i)
        assert(!strcmp(timing_write_arm(11, false, times, bad + i, 1, 0), "RANGE"));
    assert(native_levels == levels && native_dirs == dirs && claimed_dma == 5u);
    assert(!timing_write_released(0));
    assert(!strcmp(timing_write_ready(0), "RANGE"));
    assert(!strcmp(timing_write_reference(), "BAD_REQUEST"));

    pio_sm_claim(pio0, WRITE_SM + 1u);
    assert(!strcmp(timing_prepare_run(NULL, 0, 2), "RESOURCE_BUSY"));
    assert(!timing_prepare_run(NULL, 0, 1)); /* Only the requested slot is needed. */
    timing_release_run();
    pio_sm_unclaim(pio0, WRITE_SM + 1u);
    int reserved[6];
    for (size_t i = 0; i < 6; ++i) reserved[i] = dma_claim_unused_channel(false);
    assert(!strcmp(timing_prepare_run(NULL, 0, 2), "RESOURCE_BUSY"));
    assert(claimed_dma == 11u && !pio_sm_is_claimed(pio0, WRITE_SM));
    assert(native_levels == levels && native_dirs == dirs);
    for (size_t i = 0; i < 6; ++i) dma_channel_unclaim((uint) reserved[i]);
    /* Matcher ownership does not grant ownership of the separate event slots. */
    load_trigger(TIMING_TRIGGER_GPIO);
    const uint16_t occupied_code[] = {0};
    const pio_program_t occupied = {
        .instructions = occupied_code, .length = 1, .origin = WRITE_OFFSET};
    pio_add_program_at_offset(pio0, &occupied, WRITE_OFFSET);
    assert(!strcmp(timing_prepare_run(NULL, 0, 2), "RESOURCE_BUSY"));
    pio_remove_program(pio0, &occupied, WRITE_OFFSET);
    assert(!timing_prepare_run(NULL, 0, 2));
    timing_release_run();

    /* Four capture DMA reservations coexist with all five ordinary timing
     * channels and both release channels. Neither capture SM is touched. */
    for (size_t i = 0; i < 4; ++i) reserved[i] = dma_claim_unused_channel(false);
    for (unsigned rising = 0; rising < 2; ++rising) {
        arm_writes(rising, times, 2);
        assert(claimed_dma == 11u && trigger_loaded && write_loaded);
        assert(native_sm(pio0, WRITE_SM)->has_forced);
        assert(!strcmp(timing_write_reference(), "BAD_REQUEST"));
        native_tick(20);
        assert(enables[0] == 3u && enables[1] == 3u);
        assert(native_sm(pio0, WRITE_SM)->has_forced);
        uint channels[2] = {(uint) writes[0].dma, (uint) writes[1].dma};
        uint32_t counts[2] = {dma_write_count[channels[0]], dma_write_count[channels[1]]};
        for (unsigned i = 0; i < 2; ++i) {
            dma_channel_hw_t *dma = &native_dma[channels[i]];
            assert(dma->read_addr == (uintptr_t) &pio0->rxf[WRITE_SM + i]);
            assert(dma->write_addr == (uintptr_t) enable_registers[i]);
            assert(dma->transfer_count == 1u && dma_channel_is_busy(channels[i]));
            assert(((dma->ctrl_trig >> 15) & 63u) == pio_get_dreq(pio0, WRITE_SM + i, false));
            assert((dma->ctrl_trig & (3u << 2)) == (DMA_SIZE_32 << 2));
            assert(!(dma->ctrl_trig & ((1u << 4) | (1u << 5))));
        }
        assert(!strcmp(timing_prepare_run(NULL, 0, 2), "BUSY"));
        assert(!strcmp(timing_configure_gpio(11, TIMING_PUSH_PULL, !rising), "BUSY"));
        assert(!timing_write_ready(0));
        assert(!strcmp(timing_write_ready(0), "BAD_REQUEST"));
        assert(!strcmp(timing_write_ready(2), "RANGE"));
        assert(!timing_write_released(0));
        const timing_instruction_t pulse = {.op = TIMING_PULSE, .pin = 11, .cycles = 8};
        compile_start(&pulse, 1, false);
        uint limit = 100;
        while (pio_sm_get_pc(pio0, WRITE_SM) != WRITE_OFFSET + 1u && --limit) native_tick(1);
        assert(limit);
        uint64_t reference = native_cycle - 1u;
        native_tick(30);
        timing_service(); /* Leading waveform completion must not cancel event 2. */
        assert(timing_status().done && native_sm(pio0, WRITE_SM + 1u)->enabled);
        assert(timing_write_released(0) && !timing_write_released(1));
        assert(!timing_write_ready(1));
        native_tick(1000); /* No CPU service around either release. */
        for (unsigned i = 0; i < 2; ++i) {
            assert(timing_write_released(i));
            assert(enables[i] == 1u);
            assert(dma_write_cycle[channels[i]] == reference + times[i]);
            assert(dma_write_value[channels[i]] == 1u);
            assert(dma_write_count[channels[i]] == counts[i] + 1u);
            assert(!native_sm(pio0, WRITE_SM + i)->rx_count);
        }
        timing_write_stop();
        assert(claimed_dma == 11u && !timing_write_released(0));
        assert(!timing_write_error());
    }
    for (size_t i = 0; i < 4; ++i) dma_channel_unclaim((uint) reserved[i]);

    /* Before/at/after the actual assembled PULL; at means before executing the
     * instruction in that cycle. A CPU publication racing it may lose, never slip. */
    for (unsigned rising = 0; rising < 2; ++rising) {
        for (int relative = -1; relative <= 1; ++relative) {
            const uint32_t at = TIMING_WRITE_MIN_CYCLES;
            arm_writes(rising, &at, 1);
            uint channel = (uint) writes[0].dma;
            uint64_t reference = write_reference(rising);
            uint64_t cutoff = reference + at - TIMING_WRITE_READY_LEAD;
            native_tick((uint) (cutoff + relative - native_cycle));
            const char *error = timing_write_ready(0);
            if (relative <= 0) assert(!error);
            else assert(error && !strcmp(error, "MISSED_DEADLINE"));
            native_tick(100);
            assert(enables[0] == (relative <= 0 ? 1u : 3u));
            if (relative <= 0) {
                assert(dma_write_cycle[channel] == reference + at);
                assert(timing_write_released(0));
            } else {
                assert(!strcmp(timing_write_error(), "MISSED_DEADLINE"));
                assert(!strcmp(timing_write_ready(0), "MISSED_DEADLINE"));
                assert(!timing_write_released(0));
            }
        }
    }
    /* Both clocks start at the reference, not at first completion: an
     * unprepared second event halts, even with very late CPU servicing. */
    arm_writes(false, times, 2);
    assert(!timing_write_ready(0));
    write_reference(false);
    native_tick(1000);
    assert(enables[0] == 1u && enables[1] == 3u);
    assert(!strcmp(timing_write_error(), "MISSED_DEADLINE"));
    assert(timing_write_released(0) && !timing_write_released(1));
    assert(!strcmp(timing_write_ready(1), "MISSED_DEADLINE"));
    native_tick(100);
    assert(enables[1] == 3u && claimed_dma == 7u);

    /* Cancel one cycle before PUSH, then cancel with an undrained RX token. */
    for (unsigned pushed = 0; pushed < 2; ++pushed) {
        arm_writes(false, times, 1);
        assert(!timing_write_ready(0));
        uint64_t reference = write_reference(false);
        dma_fault = pushed;
        native_tick((uint) (reference + times[0] + (pushed ? 1 : -1) - native_cycle));
        assert(native_sm(pio0, WRITE_SM)->rx_count == pushed);
        dma_fault = false;
        timing_write_stop();
        assert(!native_sm(pio0, WRITE_SM)->rx_count);
        assert(!native_sm(pio0, WRITE_SM)->fifo_count);
        native_tick(100);
        assert(enables[0] == 3u);
    }
    /* E13-style in-flight write: stop must wait for BUSY before returning.
     * Resetting/reusing the controller after stop cannot receive an old token. */
    arm_writes(false, times, 1);
    uint channel = (uint) writes[0].dma;
    dma_write_delay[channel] = 3;
    assert(!timing_write_ready(0));
    uint64_t reference = write_reference(false);
    native_tick((uint) (reference + times[0] + 1u - native_cycle));
    assert(dma_pending[channel] && enables[0] == 3u);
    assert(!timing_write_released(0));
    timing_write_stop();
    assert(!dma_pending[channel] && !dma_busy[channel] && enables[0] == 1u);
    dma_write_delay[channel] = 0;
    enables[0] = 3u;
    native_tick(100);
    assert(enables[0] == 3u);
    arm_writes(false, times, 1);
    write_reference(false);
    native_tick(100);
    assert(enables[0] == 3u && !strcmp(timing_write_error(), "MISSED_DEADLINE"));

    for (unsigned flag = 29; flag <= 30; ++flag) {
        arm_writes(false, times, 2);
        uint failed = (uint) writes[1].dma;
        assert(!timing_write_ready(0));
        native_dma[failed].ctrl_trig |= 1u << flag;
        assert(!strcmp(timing_write_error(), "TIMING_ERROR"));
        assert(!native_sm(pio0, WRITE_SM)->enabled && !native_sm(pio0, WRITE_SM + 1u)->enabled);
        assert(!strcmp(timing_write_error(), "TIMING_ERROR"));
        native_tick(100);
        assert(enables[0] == 3u && enables[1] == 3u);
    }
    timing_write_stop();
    assert(!timing_write_error() && claimed_dma == 7u);
    const uint32_t maximum = UINT32_MAX;
    arm_writes(false, &maximum, 1);
    assert(native_sm(pio0, WRITE_SM)->x == UINT32_MAX - 7u);
    timing_write_stop();

    /* Cancelling a stalled forced WAIT must remove it before later pin edges. */
    arm_writes(false, times, 1);
    assert(native_sm(pio0, WRITE_SM)->has_forced);
    timing_stop();
    assert(!native_sm(pio0, WRITE_SM)->has_forced);
    native_output(1u << 11, 1u << 11, false);
    native_tick(8);
    native_output(0, 1u << 11, false);
    native_tick(200);
    assert(enables[0] == 3u);
    timing_release_run();
    assert(claimed_dma == 5u && !write_loaded);
}

static void software_references(void) {
    const uint32_t times[] = {16u, 100u};
    timing_stop();
    timing_release_run();
    assert(!timing_prepare_run(NULL, 0, 2));
    enables[0] = enables[1] = 3u;
    assert(!timing_write_arm(-1, false, times, enable_registers, 2, 0));
    assert(!timing_write_ready(0));
    native_tick(200);
    for (unsigned i = 0; i < 2; ++i) {
        assert(!native_sm(pio0, WRITE_SM + i)->enabled);
        assert(pio_sm_get_pc(pio0, WRITE_SM + i) == WRITE_OFFSET);
        assert(enables[i] == 3u);
    }
    uint64_t reference = native_cycle;
    assert(!timing_write_reference());
    assert(!strcmp(timing_write_reference(), "BAD_REQUEST"));
    native_tick(40);
    assert(timing_write_released(0) && !timing_write_released(1));
    assert(!timing_write_ready(1));
    native_tick(200);
    for (unsigned i = 0; i < 2; ++i) {
        assert(native_sm(pio0, WRITE_SM + i)->push_cycle == reference + times[i]);
        assert(dma_write_cycle[writes[i].dma] == reference + times[i]);
        assert(enables[i] == 1u && timing_write_released(i));
    }
    timing_write_stop();
    assert(!strcmp(timing_write_reference(), "BAD_REQUEST"));

    for (int relative = -1; relative <= 1; ++relative) {
        const uint32_t at = TIMING_WRITE_MIN_CYCLES;
        enables[0] = 3u;
        assert(!timing_write_arm(-1, true, &at, enable_registers, 1, 0));
        reference = native_cycle;
        assert(!timing_write_reference());
        native_tick(at - TIMING_WRITE_READY_LEAD + relative);
        const char *error = timing_write_ready(0);
        if (relative <= 0) assert(!error);
        else assert(error && !strcmp(error, "MISSED_DEADLINE"));
        native_tick(100);
        assert(enables[0] == (relative <= 0 ? 1u : 3u));
        if (relative <= 0)
            assert(native_sm(pio0, WRITE_SM)->push_cycle == reference + at);
        timing_write_stop();
    }
    assert(!timing_write_arm(-1, false, times, enable_registers, 1, 0));
    assert(!timing_write_ready(0));
    timing_write_stop(); /* A prepared but never referenced group is cancellable. */
    enables[0] = 3u;
    native_tick(200);
    assert(enables[0] == 3u && !native_sm(pio0, WRITE_SM)->fifo_count);
    timing_release_run();
}

static void matcher_references(void) {
    timing_stop();
    assert(!timing_configure_sync(12));
    const timing_spi_t config = {
        .cs = 21, .sck = 22, .data = 23, .mode = 0, .cs_active_low = true, .max_hz = 1000000};
    assert(!timing_configure_spi(&config));
    const timing_trigger_t triggers[] = {
        {.kind = TIMING_TRIGGER_GPIO, .pin = 20, .rising = true},
        {.kind = TIMING_TRIGGER_I2C, .sda = 4, .scl = 5, .address = 0x50},
        {.kind = TIMING_TRIGGER_SPI, .prefix = {0xa5}, .prefix_length = 1},
    };
    const uint32_t times[] = {16u, 100u};
    const timing_instruction_t wait = {.op = TIMING_WAIT};
    for (size_t kind = 0; kind < 3; ++kind) {
        timing_stop();
        timing_release_run();
        assert(!timing_compile(timing_gpio(), &wait, 1, test_tokens, TIMING_MAX_TOKENS, &test_block));
        assert(!timing_prepare_run(&test_block, 1, 2));
        native_output((1u << 4) | (1u << 5) | (1u << 21),
                      (1u << 4) | (1u << 5) | (1u << 20) | (1u << 21) | (1u << 22), false);
        load_trigger(triggers[kind].kind);
        uint32_t matcher_words[32];
        memcpy(matcher_words, pio1->instr_mem, sizeof(matcher_words));
        /* A previous successful matcher left a synchronized high latch. Arm
         * must clear it before the event's opposite WAIT can complete. */
        native_output(1u << 12, 1u << 12, false);
        native_tick(4);
        enables[0] = enables[1] = 3u;
        assert(!timing_write_arm(12, true, times, enable_registers, 2, 0));
        assert(!memcmp(matcher_words, pio1->instr_mem, sizeof(matcher_words)));
        assert(!(native_levels & (1u << 12)));
        assert(!timing_write_ready(0) && !timing_write_ready(1));
        native_tick(200);
        assert(enables[0] == 3u && enables[1] == 3u);
        assert(!native_sm(pio0, WRITE_SM)->has_forced);
        assert(pio_sm_get_pc(pio0, WRITE_SM) == WRITE_OFFSET);
        assert(!timing_start(&test_block, &triggers[kind], 1000000));
        native_tick(20);
        assert(enables[0] == 3u && enables[1] == 3u);
        if (triggers[kind].kind == TIMING_TRIGGER_GPIO) {
            native_output(1u << 20, 1u << 20, false);
        } else if (triggers[kind].kind == TIMING_TRIGGER_I2C) {
            native_output(0, 1u << 4, false); /* START, then address/write + ACK. */
            native_tick(24);
            for (int bit = 8; bit >= 0; --bit) {
                native_output(0, 1u << 5, false);
                native_output(((0xa0u << 1) & (1u << bit)) ? 1u << 4 : 0, 1u << 4, false);
                native_tick(8);
                native_output(1u << 5, 1u << 5, false);
                native_tick(8);
            }
        } else {
            native_output(0, 1u << 21, false);
            native_tick(16);
            for (int bit = 7; bit >= 0; --bit) {
                native_output(0, 1u << 22, false);
                native_output((0xa5u & (1u << bit)) ? 1u << 23 : 0, 1u << 23, false);
                native_tick(8);
                native_output(1u << 22, 1u << 22, false);
                native_tick(8);
            }
        }
        native_tick(200);
        timing_service();
        assert(timing_status().done && timing_status().matched && !timing_status().error);
        assert(trigger_loaded && write_loaded);
        uint64_t reference = native_sm(pio0, WRITE_SM)->wait_cycle;
        for (unsigned i = 0; i < 2; ++i) {
            assert(native_sm(pio0, WRITE_SM + i)->wait_cycle == reference);
            assert(native_sm(pio0, WRITE_SM + i)->push_cycle == reference + times[i]);
            assert(dma_write_cycle[writes[i].dma] >= reference + times[i]);
            assert(timing_write_released(i) && enables[i] == 1u);
        }
    }
    timing_stop();
    timing_release_run();
}

static void concurrent_foreground(void) {
    const uint32_t times[] = {300u, 600u};
    const timing_instruction_t carrier[] = {
        {.op = TIMING_PWM, .duty_basis_points = 5000}, {.op = TIMING_DELAY, .cycles = 20}};
    const timing_instruction_t ordinary[] = {
        {.op = TIMING_SET, .pin = 11, .state = TIMING_PIN_HIGH},
        {.op = TIMING_DELAY, .cycles = 8}};
    const timing_instruction_t wait = {.op = TIMING_WAIT};
    const timing_trigger_t edge = {.kind = TIMING_TRIGGER_GPIO, .pin = 20, .rising = true};
    compile_test(carrier, 2, true);
    assert(!timing_prepare_run(&test_block, 1, 2));
    enables[0] = enables[1] = 3u;
    assert(!timing_write_arm(-1, false, times, enable_registers, 2, 0));
    assert(!timing_write_ready(0) && !timing_write_ready(1));
    uint64_t reference = native_cycle;
    assert(!timing_write_reference());
    assert(!timing_start(&test_block, NULL, 1000000));
    run_cycles(80);
    assert(timing_status().done && timing_pwm()->active && enables[0] == 3u);
    compile_start(ordinary, 2, false);
    run_cycles(80);
    assert(timing_status().done && enables[0] == 3u);
    assert(!timing_compile(timing_gpio(), &wait, 1, test_tokens, TIMING_MAX_TOKENS, &test_block));
    native_output(0, 1u << 20, false);
    native_tick(4);
    assert(!timing_start(&test_block, &edge, 1000000));
    native_tick(20);
    native_output(1u << 20, 1u << 20, false);
    run_cycles(80);
    assert(timing_status().done && timing_status().matched && enables[0] == 3u);
    run_cycles(400);
    for (unsigned i = 0; i < 2; ++i) {
        assert(native_sm(pio0, WRITE_SM + i)->push_cycle == reference + times[i]);
        assert(enables[i] == 1u && timing_write_released(i));
    }
    timing_write_stop();
    assert(timing_pwm()->active && run_prepared);
    assert(!timing_write_arm(-1, false, times, enable_registers, 1, 0));
    timing_stop();
    timing_pwm_stop();

    /* A missed background event faults and cancels an active foreground block. */
    const uint32_t soon = TIMING_WRITE_MIN_CYCLES;
    assert(!timing_write_arm(-1, false, &soon, enable_registers, 1, 0));
    assert(!timing_write_reference());
    const timing_instruction_t delay = {.op = TIMING_DELAY, .cycles = 1000};
    compile_start(&delay, 1, false);
    native_tick(40);
    timing_service();
    assert(!strcmp(timing_status().error, "MISSED_DEADLINE"));
    assert(!timing_status().active && !native_sm(pio0, WAVE_SM)->enabled);
    assert(!native_sm(pio0, WRITE_SM)->enabled && claimed_dma == 7u);
    timing_stop();
    timing_release_run();
}

static void hardware_references(void) {
    const timing_instruction_t commands[] = {
        {.op = TIMING_DELAY},
        {.op = TIMING_DELAY, .cycles = 8},
        {.op = TIMING_SET, .pin = 11, .state = TIMING_PIN_HIGH},
        {.op = TIMING_SET, .pin = 11, .state = TIMING_PIN_LOW},
        {.op = TIMING_SET, .pin = 10, .state = TIMING_PIN_LOW},
        {.op = TIMING_SET, .pin = 11, .state = TIMING_PIN_FLOATING},
        {.op = TIMING_PWM, .duty_basis_points = 5000},
        {.op = TIMING_SET, .pin = 13, .state = TIMING_PIN_FLOATING},
    };
    const uint64_t offsets[] = {1, 9, 1, 1, 2, 2, 6, 6};
    const uint32_t times[] = {8, 80};
    static timing_block_t sparse[64];
    uint32_t next_tokens[3];
    for (unsigned command = 0; command < sizeof(commands) / sizeof(*commands); ++command) {
        for (int relative = -1; relative <= 1; ++relative) {
            timing_stop();
            timing_release_run();
            timing_pwm_stop();
            assert(!timing_configure_gpio(11, TIMING_PUSH_PULL, false));
            assert(!timing_configure_gpio(10, TIMING_OPEN_DRAIN, true));
            compile_test(&commands[command], 1, command >= 6);
            assert(test_block.completion_cycles == offsets[command]);
            memset(sparse, 0, sizeof(sparse));
            sparse[0].pwm_count = SIZE_MAX; /* A count-zero slot is not a block. */
            sparse[7] = test_block;
            timing_gpio_t next_gpio = *timing_gpio();
            next_gpio.current_levels = test_block.final_levels;
            next_gpio.current_dirs = test_block.final_dirs;
            const timing_instruction_t next = {.op = TIMING_DELAY};
            assert(!timing_compile(&next_gpio, &next, 1, next_tokens, 3, &sparse[63]));
            assert(!timing_prepare_run(sparse, 64, 2));
            enables[0] = enables[1] = 3u;
            assert(!timing_write_arm(-1, false, times, enable_registers, 2,
                                     sparse[7].completion_cycles));
            assert(!timing_write_ready(0)); /* Permits are valid during the prelude. */
            assert(!strcmp(timing_write_reference(), "BAD_REQUEST"));
            native_tick(100);
            assert(!native_sm(pio0, WRITE_SM)->enabled && enables[0] == 3u);
            uint64_t begin = native_cycle, reference = begin + offsets[command];
            assert(!timing_start(&sparse[7], NULL, 1000000));
            assert(native_sm(pio0, WRITE_SM)->enable_cycle == begin);
            native_tick((uint) offsets[command] + 24u);
            assert(enables[0] == 1u && enables[1] == 3u);
            assert(native_sm(pio0, WRITE_SM)->push_cycle == reference + times[0]);
            timing_service(); /* Completion observed well after its hardware origin. */
            assert(timing_status().done && !timing_status().error);
            assert(!timing_start(&sparse[63], NULL, 1000000));
            assert(native_sm(pio0, WRITE_SM + 1u)->enable_cycle == begin);
            native_tick((uint) (reference + times[1] - TIMING_WRITE_READY_LEAD +
                               relative - native_cycle));
            const char *error = timing_write_ready(1);
            if (relative <= 0) assert(!error);
            else assert(error && !strcmp(error, "MISSED_DEADLINE"));
            native_tick(160);
            assert(enables[1] == (relative <= 0 ? 1u : 3u));
            if (relative <= 0)
                assert(native_sm(pio0, WRITE_SM + 1u)->push_cycle == reference + times[1]);
            timing_stop();
            timing_release_run();
            assert(claimed_dma == 5u && !write_loaded);
        }
    }
    timing_pwm_stop();
    const timing_instruction_t maximum = {.op = TIMING_DELAY, .cycles = UINT32_MAX};
    compile_test(&maximum, 1, false);
    assert(test_block.completion_cycles == (UINT64_C(1) << 32));
    assert(!timing_prepare_run(&test_block, 1, 1));
    assert(!timing_write_arm(-1, false, times, enable_registers, 1, test_block.completion_cycles));
    assert(native_sm(pio0, WRITE_SM)->y == UINT32_MAX);
    timing_write_stop();
    assert(!strcmp(timing_write_arm(-1, false, times, enable_registers, 1,
                                   (UINT64_C(1) << 32) + 1), "RANGE"));
    assert(!strcmp(timing_write_arm(11, false, times, enable_registers, 1, 1), "RANGE"));
    timing_release_run();

    /* The largest legal set of independent PWM blocks fills the descriptor
     * pool exactly; preparation is still separate from execution. */
    const timing_instruction_t carrier = {.op = TIMING_PWM, .duty_basis_points = 5000};
    for (unsigned i = 0; i < 64; ++i) {
        assert(!timing_compile_pwm(timing_gpio(), timing_pwm(), &carrier, 1, 0,
                                  test_tokens + i * 3u, 3, test_segments + i, 1, &sparse[i]));
    }
    assert(!timing_prepare_run(sparse, 64, 2));
    assert(sparse[63].pwm_tail_descriptor == 127u && claimed_dma == 7u);
    assert(!timing_start(&sparse[63], NULL, 1000000));
    run_cycles(100);
    timing_release_run();
    assert(claimed_dma == 5u && timing_pwm()->active);
    timing_pwm_stop();
}

int main(void) {
    timing_init();
    assert(claimed_dma==5u);
    assert(!timing_configure_gpio(10,TIMING_OPEN_DRAIN,true));
    assert(!timing_configure_gpio(11,TIMING_PUSH_PULL,false));
    assert(!timing_configure_gpio(29,TIMING_PUSH_PULL,false));
    assert(!timing_configure_pwm(13,6250000));
    const timing_instruction_t modes[]={
        {.op=TIMING_PWM,.duty_basis_points=1500}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_PWM,.duty_basis_points=5000}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_SET,.pin=13,.state=TIMING_PIN_FLOATING}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_PWM,.duty_basis_points=1500}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_SET,.pin=13,.state=TIMING_PIN_HIGH}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_PWM,.duty_basis_points=5000}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_SET,.pin=13,.state=TIMING_PIN_LOW}};
    compile_start(modes,sizeof(modes)/sizeof(*modes),true);
    run_cycles(200);
    assert(timing_status().done && !timing_pwm()->active);
    assert(!native_sm(pio0,PWM_SM)->enabled);
    assert(!dma_busy[pwm_dma] && !dma_busy[pwm_control_dma]);
    assert_state_at(6,true,true);
    assert_state_at(26,true,true);
    assert_state_at(46,false,false);
    assert_state_at(66,true,true); /* Latch + output enable resume in the same cycle. */
    assert_state_at(86,true,true);
    assert_state_at(116,false,true); /* Final LOW is already selected: no spurious edge. */
    assert(timing_pwm()->final_mode==TIMING_PWM_LOW);
    assert(!native_fifo_gap);
    size_t reads=staged_reads;
    native_tick(100);
    assert(reads==staged_reads);

    for (timing_pin_state_t state=TIMING_PIN_LOW; state<=TIMING_PIN_FLOATING; ++state) {
        timing_instruction_t step={.op=TIMING_SET,.pin=13,.state=state};
        compile_start(&step,1,true);
        run_cycles(60);
        assert(!timing_pwm()->active && timing_status().done);
        assert(!!(native_levels & (1u<<13))==(state==TIMING_PIN_HIGH));
        assert(!!(native_dirs & (1u<<13))==(state!=TIMING_PIN_FLOATING));
        uint32_t levels=native_levels, dirs=native_dirs;
        native_tick(80);
        assert(native_levels==levels && native_dirs==dirs);
    }
    timing_instruction_t high={.op=TIMING_SET,.pin=13,.state=TIMING_PIN_HIGH};
    compile_start(&high,1,true);
    run_cycles(60);
    const timing_instruction_t release[]={
        {.op=TIMING_DELAY,.cycles=20}, {.op=TIMING_SET,.pin=13,.state=TIMING_PIN_FLOATING}};
    uint64_t release_begin=native_cycle;
    compile_start(release,2,true);
    assert(native_levels & (1u<<13));
    native_tick(26);
    assert((native_levels & (1u<<13)) && (native_dirs & (1u<<13)));
    native_tick(1);
    assert_state_at(release_begin+26,false,false);
    run_cycles(40);

    /* A held floating prefix must not be driven low by CPU startup. */
    const timing_instruction_t resume[]={
        {.op=TIMING_DELAY,.cycles=20}, {.op=TIMING_PWM,.duty_basis_points=1500}};
    uint64_t resume_begin=native_cycle;
    compile_start(resume,2,true);
    assert(!(native_dirs & (1u<<13)));
    native_tick(26);
    assert(!(native_dirs & (1u<<13)));
    native_tick(1);
    assert_state_at(resume_begin+26,true,true);
    run_cycles(60);

    const timing_instruction_t carrier[]={
        {.op=TIMING_PWM,.duty_basis_points=5000}, {.op=TIMING_DELAY,.cycles=20},
        {.op=TIMING_PWM,.duty_basis_points=1500}};
    uint64_t begin=native_cycle;
    compile_start(carrier,3,true);
    run_cycles(120);
    assert(timing_status().done && timing_pwm()->active && pwm_tail_owned());
    assert(run_prepared);
    assert_carrier(begin+26,begin+120,3,20);
    reads=staged_reads;
    timing_release_run();
    memset(test_segments,0xa5,sizeof(test_segments));
    const timing_instruction_t restaged = {.op = TIMING_PWM, .duty_basis_points = 5000};
    compile_test(&restaged, 1, true);
    assert(!timing_prepare_run(&test_block, 1, 0));
    /* Preparing another carrier overwrites all run descriptors, not the held tail. */
    /* Force the real control chain across a retained-tail boundary without
     * waiting 137 seconds. Keep the current read-ring offset intact. */
    timing_service();
    uint offset=(native_dma[pwm_dma].read_addr-(uintptr_t)pwm_final_words)/4u;
    native_dma[pwm_dma].transfer_count=4u-offset;
    begin=native_cycle;
    run_cycles(160);
    assert(staged_reads==reads && pwm_tail_owned() && !native_fifo_gap);
    assert_carrier(begin,begin+160,3,20);

    /* Restart only the waveform while the retained carrier continues. */
    const timing_instruction_t gpio_steps[]={
        {.op=TIMING_SET,.pin=11,.state=TIMING_PIN_HIGH},
        {.op=TIMING_SET,.pin=29,.state=TIMING_PIN_FLOATING},
        {.op=TIMING_PULSE,.pin=29,.cycles=8}};
    begin=native_cycle;
    compile_start(gpio_steps,3,false);
    assert(test_block.programmed_cycles==24 && test_block.inserted_cycles==16);
    run_cycles(100);
    assert(timing_gpio()->current_levels & (1u<<11));
    assert(timing_gpio()->current_dirs & (1u<<29));
    assert(!(timing_gpio()->current_levels & (1u<<29)));
    assert(timing_pwm()->active);
    assert_carrier(begin,begin+100,3,20);
    timing_stop(); timing_pwm_stop();
    assert(!(native_levels & ((1u<<11)|(1u<<13)|(1u<<29))));
    assert(!(native_dirs & (1u<<10)) && (native_dirs & (1u<<13)));
    assert(timing_gpio()->current_levels==idle_levels(timing_gpio()));
    assert(timing_gpio()->current_dirs==idle_dirs(timing_gpio()));
    assert(!dma_busy[pwm_dma] && !dma_busy[pwm_control_dma]);

    /* First effective Set from floating uses the same cycle-6 warmup as PWM. */
    timing_instruction_t floating={.op=TIMING_SET,.pin=29,.state=TIMING_PIN_FLOATING};
    compile_start(&floating,1,false);
    run_cycles(32);
    const timing_instruction_t aligned[]={
        {.op=TIMING_PWM,.duty_basis_points=5000},
        {.op=TIMING_SET,.pin=29,.state=TIMING_PIN_HIGH}};
    begin=native_cycle;
    compile_start(aligned,2,true);
    assert(test_block.wave_start_countdown==1u);
    run_cycles(80);
    assert_state_at(begin+6,true,true);
    bool simultaneous=false;
    for (size_t i=0; i<native_edge_count; ++i)
        if (native_edges[i].cycle==begin+6)
            simultaneous=(native_edges[i].dirs & (1u<<29)) && (native_edges[i].levels & (1u<<29));
    assert(simultaneous);
    timing_stop(); timing_pwm_stop();

    /* Actual DMA error detection remains active after successful completion. */
    compile_start(carrier,3,true);
    run_cycles(100);
    native_dma[pwm_dma].ctrl_trig |= DMA_CH0_CTRL_TRIG_READ_ERROR_BITS;
    timing_service();
    assert(timing_pwm_error() && !strcmp(timing_pwm_error(),"TIMING_ERROR"));
    assert(!timing_pwm()->active && !(native_levels & (1u<<13)));
    assert(!dma_busy[pwm_dma] && !dma_busy[pwm_control_dma]);
    timing_pwm_stop();
    compile_start(carrier,3,true);
    dma_fault=true;
    native_tick(200);
    timing_service();
    assert(timing_pwm_error() && !timing_pwm()->active);
    assert(!(native_levels & (1u<<13)) && (native_dirs & (1u<<13)));
    dma_fault=false;
    timing_pwm_stop();
    /* CPU prefill is fixed work even with broken DMA. The first real starvation
     * still faults instead of hiding a gap or spinning through bus service. */
    timing_release_run();
    assert(!timing_compile_pwm(timing_gpio(),timing_pwm(),carrier,3,0,test_tokens,
        TIMING_MAX_TOKENS,test_segments,TIMING_MAX_PWM_SEGMENTS,&test_block));
    assert(!timing_prepare_run(&test_block, 1, 0));
    dma_fault=true;
    assert(!timing_start(&test_block,NULL,1000000));
    native_tick(200);
    timing_service();
    assert(!timing_status().active && !timing_pwm()->active);
    assert(!native_sm(pio0,PWM_SM)->enabled);
    assert(!(native_levels & (1u<<13)) && (native_dirs & (1u<<13)));
    scheduled_runtime();
    software_references();
    matcher_references();
    concurrent_foreground();
    hardware_references();
    puts("production timing runtime: PWM, scheduled PIO releases, DMA lifecycle, conflicts and faults passed");
    return 0;
}
