/* Exercise actual production controller preparation, completion, and ownership. */
#define I2C_RUNTIME_TEST
#include "../i2c_hw.c"

static const uint8_t payload[17] = {0x23, 0xd4, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const uint16_t single_command[] = {0x23 | I2C_IC_DATA_CMD_STOP_BITS};
static const uint16_t commands[] = {0x23, 0xd4, 0x00 | I2C_IC_DATA_CMD_STOP_BITS};

static const uint16_t *encode_payload(size_t length) {
    static uint16_t encoded[I2C_PREPARED_LIMIT];
    assert(length && length <= I2C_PREPARED_LIMIT);
    for (size_t i = 0; i < length; ++i)
        encoded[i] = payload[i];
    encoded[length - 1] |= I2C_IC_DATA_CMD_STOP_BITS;
    return encoded;
}

static void encoding_checks(void) {
    i2c_transfer_t model, before;
    memset(&model, 0x5a, sizeof(model));
    memcpy(&before, &model, sizeof(model));
    assert(!strcmp(i2c_transfer_prepare(&model, NULL, 1, 100, 1000), "RANGE"));
    assert(!strcmp(i2c_transfer_prepare(&model, single_command, 0, 100, 1000), "RANGE"));
    assert(!strcmp(i2c_transfer_prepare(&model, single_command, 17, 100, 1000), "RANGE"));
    assert(!strcmp(i2c_transfer_prepare(&model, single_command, 1, 100, 100), "RANGE"));
    assert(!strcmp(i2c_transfer_prepare(&model, single_command, 1, 100, 60000101), "RANGE"));
    const uint16_t invalid[][3] = {
        {0x223, 0xd4, 0x200}, /* Early STOP. */
        {0x23, 0xd4, 0x00},   /* Missing final STOP. */
        {0x123, 0xd4, 0x200}, /* Read before final word. */
        {0x23, 0xd4, 0x300},  /* Final read. */
        {0x423, 0xd4, 0x200}, /* Restart before final word. */
        {0x23, 0xd4, 0x600},  /* Final restart. */
        {0x823, 0xd4, 0x200}, /* Reserved command bits. */
        {0x23, 0xd4, 0x8200},
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        assert(!strcmp(i2c_transfer_prepare(&model, invalid[i], 3, 100, 1000), "RANGE"));
    assert(!memcmp(&model, &before, sizeof(model)));
    for (size_t length = 1; length <= I2C_PREPARED_LIMIT; ++length) {
        const uint16_t *encoded = encode_payload(length);
        assert(!i2c_transfer_prepare(&model, encoded, length, 100, 1000));
        assert(model.commands == encoded && !model.write && !model.read && !model.read_length);
        model.command_blocked = true;
        uint16_t command = 0xffff;
        assert(!i2c_transfer_advance(&model, 101, 0, true, false, false, &command));
        assert(!model.writes_queued && command == 0xffff);
        for (size_t i = 0; i < length; ++i) {
            assert(i2c_transfer_advance(&model, 101, 0, true, true, false, &command));
            assert(command == encoded[i] && model.writes_queued == i + 1);
            assert(!model.result.written);
        }
        assert(!i2c_transfer_advance(&model, 102, 0, true, true, true, &command));
        assert(model.busy && !model.result.written);
        model.command_blocked = false;
        assert(!i2c_transfer_advance(&model, 103, 0, false, true, false, &command));
        assert(model.busy && !model.result.written);
        assert(!i2c_transfer_advance(&model, 104, 0, true, true, true, &command));
        assert(!model.busy && !model.result.error && model.result.written == length);
    }
}

static void fresh(void) {
    memset(configs, 0, sizeof(configs));
    memset(controllers, 0, sizeof(controllers));
    memset(&transfer, 0, sizeof(transfer));
    memset(native_regs, 0, sizeof(native_regs));
    memset(native_overrides, 0, sizeof(native_overrides));
    memset(native_functions, 0, sizeof(native_functions));
    transfer_owned = transfer_scheduled = transfer_stop_seen = false;
    native_now = 100;
    native_now_step = 0;
    native_reset_count = 0;
    native_resets.reset_done = RESETS_RESET_I2C0_BITS | RESETS_RESET_I2C1_BITS;
    native_lines_high = true;
    native_release_armed = false;
    assert(!i2c_bus_configure(0, 4, 5, 1000000, false));
    assert(!i2c_bus_configure(1, 6, 7, 400000, true));
}

static i2c_hw_t *preload(unsigned bus, size_t length) {
    i2c_hw_t *hw = &native_regs[bus];
    assert(!i2c_bus_prepare(bus, 0x25, encode_payload(length), length, 1000));
    assert(i2c_bus_busy() && !i2c_bus_prepared());
    assert(!hw->enable && !transfer.writes_queued);
    /* Disable acknowledgement is asynchronous: TAR cannot change early. */
    i2c_bus_service();
    assert(!hw->tar && !transfer.writes_queued);
    hw->enable_status = 0;
    hw->raw_intr_stat = 0;
    i2c_bus_service();
    assert(hw->enable == 5 && hw->tar == 0x25);
    assert(transfer.writes_queued == length && !i2c_bus_result()->written);
    assert(hw->data_cmd == (payload[length - 1] | I2C_IC_DATA_CMD_STOP_BITS));
    assert(i2c_bus_prepared());
    hw->enable_status = 1;
    hw->txflr = (uint32_t)length;
    hw->status = length < 16 ? I2C_IC_STATUS_TFNF_BITS : 0;
    return hw;
}

static void complete(i2c_hw_t *hw, size_t length) {
    /* The release value is written by the external engine, never bus service. */
    *i2c_bus_enable_register(transfer_bus) = 1;
    assert(!i2c_bus_prepared());
    hw->enable_status = 1;
    hw->txflr = 0;
    hw->status = I2C_IC_STATUS_TFE_BITS | I2C_IC_STATUS_TFNF_BITS;
    hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_STOP_DET_BITS | I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
    i2c_bus_service();
    assert(!i2c_bus_busy() && !i2c_bus_result()->error);
    assert(i2c_bus_result()->written == length && !transfer_owned);
    hw->raw_intr_stat = 0;
}

static void preparation_checks(void) {
    fresh();
    assert(i2c_bus_enable_register(0) == &native_regs[0].enable);
    assert(i2c_bus_enable_register(1) == &native_regs[1].enable);
    assert(!i2c_bus_enable_register(2));
    assert(!strcmp(i2c_bus_prepare(0, 0x25, single_command, 0, 1000), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(0, 0x25, single_command, 17, 1000), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(0, 0x25, NULL, 1, 1000), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(0, 0x25, single_command, 1, 100), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(2, 0x25, single_command, 1, 1000), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(0, 7, single_command, 1, 1000), "RANGE"));
    configs[1].configured = false;
    assert(!strcmp(i2c_bus_prepare(1, 0x25, single_command, 1, 1000), "UNCONFIGURED"));
    configs[1].configured = true;
    assert(!i2c_bus_busy() && !transfer_owned && !native_reset_count);
    const uint16_t missing_stop[] = {0x23};
    const uint16_t read_command[] = {0x23 | I2C_IC_DATA_CMD_CMD_BITS | I2C_IC_DATA_CMD_STOP_BITS};
    assert(!strcmp(i2c_bus_prepare(0, 0x25, missing_stop, 1, 1000), "RANGE"));
    assert(!strcmp(i2c_bus_prepare(0, 0x25, read_command, 1, 1000), "RANGE"));
    assert(!i2c_bus_busy() && !transfer_owned && !transfer.writes_queued && !native_reset_count);

    /* Another master may own the wire while this local FIFO is prepared. */
    native_lines_high = false;
    i2c_hw_t *hw = preload(0, 16);
    assert(native_overrides[4] == GPIO_OVERRIDE_NORMAL && native_overrides[5] == GPIO_OVERRIDE_NORMAL);
    assert(!strcmp(i2c_bus_begin(1, 0x25, payload, 1, NULL, 0, 1000), "BUSY"));
    assert(!strcmp(i2c_bus_prepare(1, 0x25, single_command, 1, 1000), "BUSY"));
    assert(!strcmp(i2c_bus_configure(1, 6, 7, 100000, false), "BUSY"));
    /* A foreign STOP and even an empty snapshot cannot acknowledge blocked data. */
    hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_STOP_DET_BITS | I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
    i2c_bus_service();
    assert(i2c_bus_prepared() && !i2c_bus_result()->written && hw->enable == 5);
    complete(hw, 16);
    assert(!native_reset_count);

    /* Same valid TAR is reusable only with an idle controller and empty FIFOs. */
    hw->status |= I2C_IC_STATUS_ACTIVITY_BITS;
    assert(!i2c_bus_prepare(0, 0x25, commands, 3, 1000));
    i2c_bus_service();
    assert(!transfer.writes_queued && hw->enable == 1);
    hw->status = I2C_IC_STATUS_TFNF_BITS;
    i2c_bus_service();
    assert(!transfer.writes_queued && hw->enable == 1);
    hw->status |= I2C_IC_STATUS_TFE_BITS;
    hw->rxflr = 1;
    i2c_bus_service();
    assert(!transfer.writes_queued && hw->enable == 1);
    hw->rxflr = 0;
    i2c_bus_service();
    assert(i2c_bus_prepared() && hw->enable == 5 && transfer.writes_queued == 3);
    complete(hw, 3);

    assert(!i2c_bus_prepare(0, 0x26, commands, 3, 1000));
    assert(!hw->enable && hw->tar == 0x25 && !transfer.writes_queued);
    i2c_bus_service();
    assert(!hw->enable && hw->tar == 0x25 && !transfer.writes_queued);
    hw->enable_status = 0;
    i2c_bus_service();
    assert(i2c_bus_prepared() && hw->tar == 0x26);
    complete(hw, 3);
    assert(!native_reset_count);
}

static void failure_checks(void) {
    const uint32_t aborts[] = {0, 1u, 8u, (1u << 12) | 8u, (15u << 23) | 8u};
    const char *errors[] = {"BUS_TIMEOUT", "NACK", "NACK", "ARBITRATION_LOST", "NACK"};
    for (unsigned released = 0; released < 2; ++released) {
        for (unsigned n = 0; n < sizeof(aborts) / sizeof(aborts[0]); ++n) {
            fresh();
            i2c_hw_t *hw = preload(1, 16);
            native_release_armed = true;
            if (released) hw->enable = 1;
            hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS | I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
            if (aborts[n]) hw->raw_intr_stat |= I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS;
            hw->tx_abrt_source = aborts[n];
            native_now = 1000;
            i2c_bus_service();
            assert(!i2c_bus_busy() && !i2c_bus_prepared());
            assert(!strcmp(i2c_bus_result()->error, errors[n]));
            assert(i2c_bus_result()->raw_abort == aborts[n] && !i2c_bus_result()->written);
            assert(transfer_owned && !native_reset_count && hw->enable == (released ? 1u : 5u));
            assert(!strcmp(i2c_bus_prepare(0, 0x25, single_command, 1, 2000), "BUSY"));
            assert(!strcmp(i2c_bus_configure(0, 4, 5, 100000, false), "BUSY"));
            i2c_bus_service();
            assert(!native_reset_count);
            native_release_armed = false; /* Caller cancels release DMA first. */
            i2c_bus_cancel();
            assert(native_reset_count == 1 && !transfer_owned && !hw->enable);
            assert(native_overrides[6] == GPIO_OVERRIDE_LOW && native_overrides[7] == GPIO_OVERRIDE_LOW);
            assert(!strcmp(i2c_bus_result()->error, errors[n]) && i2c_bus_result()->raw_abort == aborts[n]);
            i2c_bus_cancel();
            assert(native_reset_count == 1);
            /* Reset restoration keeps saved clock/pin configuration, but never
             * treats the reset target register as a reusable TAR. */
            assert(!i2c_bus_prepare(1, 0x25, commands, 3, 2000));
            assert(hw->enable == 5 && controllers[1].target_valid);
            assert(hw->con == controllers[1].control);
            assert(hw->fs_scl_hcnt == controllers[1].scl_high_count);
            assert(hw->fs_scl_lcnt == controllers[1].scl_low_count);
            assert(hw->sda_hold == controllers[1].sda_hold);
            assert(hw->fs_spklen == controllers[1].spike_length);
            assert(i2c_bus_prepared() && !i2c_bus_result()->written);
            i2c_bus_cancel();
            assert(!strcmp(i2c_bus_result()->error, "ABORTED") && native_reset_count == 2);
        }
    }

    /* Preparation failure before any ENABLE/block write also requires cancel. */
    fresh();
    native_regs[0].status |= I2C_IC_STATUS_ACTIVITY_BITS;
    assert(!i2c_bus_prepare(0, 0x25, commands, 3, 200));
    i2c_bus_service();
    native_now = 200;
    i2c_bus_service();
    assert(!i2c_bus_busy() && transfer_owned && !native_reset_count);
    i2c_bus_cancel();
    assert(native_reset_count == 1 && !strcmp(i2c_bus_result()->error, "BUS_TIMEOUT"));
}

static void turnaround_checks(void) {
    for (unsigned scheduled = 0; scheduled < 2; ++scheduled) {
        fresh();
        i2c_hw_t *hw = preload(0, 1);
        complete(hw, 1);
        for (size_t length = 1; length <= I2C_PREPARED_LIMIT; ++length) {
            /* No intervening service call: entry itself must reuse TAR and fill
             * every available FIFO slot immediately after the previous STOP. */
            const char *error = scheduled
                ? i2c_bus_prepare(0, 0x25, encode_payload(length), length, 1000)
                : i2c_bus_begin(0, 0x25, payload, length, NULL, 0, 1000);
            assert(!error && i2c_bus_busy() && !i2c_bus_result()->error);
            assert(transfer.writes_queued == length && !i2c_bus_result()->written);
            assert(hw->enable == (scheduled ? 5u : 1u) && hw->tar == 0x25);
            assert(hw->data_cmd == (payload[length - 1] | I2C_IC_DATA_CMD_STOP_BITS));
            assert(i2c_bus_prepared() == (bool)scheduled && !transfer_stop_seen);
            if (scheduled) {
                /* Foreign STOP while blocked must not survive hardware release. */
                hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_STOP_DET_BITS |
                                    I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
                i2c_bus_service();
                assert(!transfer_stop_seen && !i2c_bus_result()->written);
                hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
                hw->enable = 1;
                i2c_bus_service();
                assert(i2c_bus_busy());
            }
            /* STOP may precede the local master-active bit dropping. The
             * read-to-clear event must survive into the next service call. */
            hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_STOP_DET_BITS |
                                I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
            hw->status |= I2C_IC_STATUS_MST_ACTIVITY_BITS;
            i2c_bus_service();
            assert(i2c_bus_busy() && transfer_stop_seen);
            hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
            hw->status &= ~I2C_IC_STATUS_MST_ACTIVITY_BITS;
            i2c_bus_service();
            assert(!i2c_bus_busy() && !transfer_owned && !i2c_bus_result()->error);
            assert(i2c_bus_result()->written == length && !native_reset_count);
            hw->raw_intr_stat = 0;
        }
    }

    fresh();
    /* A controller that already acknowledges disabled needs no extra service
     * turn before TAR setup and FIFO prefill, including reset restoration. */
    i2c_hw_t *hw = &native_regs[0];
    hw->enable = hw->enable_status = 0;
    assert(!i2c_bus_prepare(0, 0x26, commands, 3, 1000));
    assert(i2c_bus_prepared() && hw->tar == 0x26 && transfer.writes_queued == 3);
    i2c_bus_cancel();
    native_resets.reset_done &= ~RESETS_RESET_I2C0_BITS;
    assert(!i2c_bus_prepare(0, 0x27, commands, 3, 1000));
    assert(i2c_bus_busy() && !i2c_bus_prepared() && !transfer.writes_queued);
    assert(!hw->enable && !controllers[0].target_valid);
    i2c_bus_service();
    assert(!transfer.writes_queued && !hw->tar);
    native_resets.reset_done |= RESETS_RESET_I2C0_BITS;
    i2c_bus_service();
    assert(i2c_bus_prepared() && transfer.writes_queued == 3 && hw->tar == 0x27);
    complete(hw, 3);

    uint8_t received = 0;
    assert(!i2c_bus_begin(0, 0x27, payload, 1, &received, 1, 1000));
    assert(transfer.writes_queued == 1 && transfer.reads_queued == 1);
    assert(hw->data_cmd == (I2C_IC_DATA_CMD_CMD_BITS | I2C_IC_DATA_CMD_RESTART_BITS |
                            I2C_IC_DATA_CMD_STOP_BITS));
    hw->data_cmd = 0x5a;
    hw->rxflr = 1;
    complete(hw, 1);
    assert(received == 0x5a && i2c_bus_result()->read == 1);
}

static void synchronous_failure_checks(void) {
    for (unsigned scheduled = 0; scheduled < 2; ++scheduled) {
        fresh();
        i2c_hw_t *hw = preload(0, 1);
        complete(hw, 1);
        native_now_step = 100;
        native_release_armed = scheduled;
        const char *error = scheduled
            ? i2c_bus_prepare(0, 0x25, commands, 3, 200)
            : i2c_bus_begin(0, 0x25, payload, 3, NULL, 0, 200);
        /* Admission succeeded at t=100, then service expired at t=200.
         * Caller must inspect the result even though entry returns NULL. */
        assert(!error && !i2c_bus_busy() && !transfer.writes_queued);
        assert(!strcmp(i2c_bus_result()->error, "BUS_TIMEOUT"));
        assert(transfer_owned == (bool)scheduled);
        assert(native_reset_count == (scheduled ? 0u : 1u));
        if (scheduled) {
            assert(!strcmp(i2c_bus_begin(0, 0x25, payload, 1, NULL, 0, 1000), "BUSY"));
            assert(!strcmp(i2c_bus_configure(0, 4, 5, 100000, false), "BUSY"));
        }
        native_release_armed = false;
        i2c_bus_cancel();
        assert(!transfer_owned && native_reset_count == 1);
    }
}

static void ordinary_checks(void) {
    fresh();
    i2c_hw_t *hw = &native_regs[0];
    native_lines_high = false;
    assert(!i2c_bus_begin(0, 0x25, payload, 17, NULL, 0, 1000));
    i2c_bus_service();
    assert(hw->enable == 1 && !transfer.writes_queued);
    native_lines_high = true;
    i2c_bus_service();
    assert(!hw->enable);
    hw->enable_status = 0;
    hw->raw_intr_stat = 0;
    i2c_bus_service();
    assert(hw->enable == 1 && transfer.writes_queued == 16 && !i2c_bus_result()->written);
    assert(!(hw->data_cmd & I2C_IC_DATA_CMD_STOP_BITS));
    hw->status = 0;
    i2c_bus_service();
    assert(transfer.writes_queued == 16);
    hw->status = I2C_IC_STATUS_TFNF_BITS;
    i2c_bus_service();
    assert(transfer.writes_queued == 17 && hw->data_cmd == (payload[16] | I2C_IC_DATA_CMD_STOP_BITS));
    assert(!i2c_bus_result()->written);
    complete(hw, 17);
    assert(!i2c_bus_begin(0, 0x25, payload, 17, NULL, 0, 1000));
    assert(transfer.writes_queued == 16 && !i2c_bus_result()->written);
    assert(!(hw->data_cmd & I2C_IC_DATA_CMD_STOP_BITS));
    i2c_bus_service();
    assert(transfer.writes_queued == 17);
    complete(hw, 17);

    /* Ordinary failures continue to auto-release: no scheduled DMA owns them. */
    hw->enable_status = 1;
    assert(!i2c_bus_begin(0, 0x25, payload, 1, NULL, 0, 1000));
    i2c_bus_service();
    hw->raw_intr_stat = I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS | I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
    hw->tx_abrt_source = 8;
    native_now = 1000;
    i2c_bus_service();
    assert(!i2c_bus_busy() && !strcmp(i2c_bus_result()->error, "NACK"));
    assert(i2c_bus_result()->raw_abort == 8 && native_reset_count == 1 && !transfer_owned);
    i2c_bus_cancel();
    assert(native_reset_count == 1);
}

int main(void) {
    encoding_checks();
    preparation_checks();
    failure_checks();
    turnaround_checks();
    synchronous_failure_checks();
    ordinary_checks();
    puts("production I2C register lifecycle checks passed");
    return 0;
}
