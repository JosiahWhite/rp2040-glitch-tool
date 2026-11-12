#include "i2c_bus.h"

#ifdef I2C_RUNTIME_TEST
#include "tests/i2c_runtime_shim.h"
#else
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/regs/resets.h"
#include "hardware/resets.h"
#include "pico/platform.h"
#include "pico/time.h"
#endif

/* The hardware-independent transfer core emits these RP2040 encodings. */
_Static_assert((1u << 8) == I2C_IC_DATA_CMD_CMD_BITS, "read command layout");
_Static_assert((1u << 9) == I2C_IC_DATA_CMD_STOP_BITS, "stop command layout");
_Static_assert((1u << 10) == I2C_IC_DATA_CMD_RESTART_BITS, "restart command layout");
_Static_assert((1u << 12) == I2C_IC_TX_ABRT_SOURCE_ARB_LOST_BITS, "abort layout");

/* Core1 owns execution; core0 accesses it only while the executor is parked. */
static i2c_bus_config_t configs[I2C_BUS_COUNT];
static i2c_transfer_t   transfer;
static unsigned         transfer_bus;
static uint8_t          transfer_address;
static enum {
    TRANSFER_WAIT_FREE,
    TRANSFER_DISABLE,
    TRANSFER_ACTIVE
} transfer_phase;
static bool transfer_owned, transfer_scheduled, transfer_stop_seen;

/* Saved timing survives a local cancellation reset without calling blocking SDK
 * transfer/initialization functions from the service loop. */
typedef struct {
    uint32_t control, scl_high_count, scl_low_count, sda_hold, spike_length;
    bool     reset_pending, target_valid;
} controller_state_t;
static controller_state_t controllers[I2C_BUS_COUNT];

static inline i2c_inst_t *controller_instance(unsigned bus) {
    return bus ? i2c1 : i2c0;
}

static inline uint32_t controller_reset_mask(unsigned bus) {
    return bus ? RESETS_RESET_I2C1_BITS : RESETS_RESET_I2C0_BITS;
}

static void __not_in_flash_func(controller_release)(unsigned bus) {
    const i2c_bus_config_t *config = &configs[bus];
    /* Never request IC_ENABLE.ABORT: it deliberately generates a STOP. Force
     * only local output enables inactive, then reset the local peripheral. */
    gpio_set_oeover(config->sda, GPIO_OVERRIDE_LOW);
    gpio_set_oeover(config->scl, GPIO_OVERRIDE_LOW);
    controller_instance(bus)->hw->enable = 0;
    reset_block_mask(controller_reset_mask(bus));
    unreset_block_mask(controller_reset_mask(bus));
    controller_instance(bus)->restart_on_next = false;
    controllers[bus].reset_pending            = true;
    controllers[bus].target_valid             = false;
}

static bool __not_in_flash_func(controller_restore)(unsigned bus) {
    controller_state_t *controller = &controllers[bus];
    if (!controller->reset_pending)
        return true;
    if (!(resets_hw->reset_done & controller_reset_mask(bus)))
        return false;
    i2c_hw_t *hw              = controller_instance(bus)->hw;
    hw->enable                = 0;
    hw->con                   = controller->control;
    hw->fs_scl_hcnt           = controller->scl_high_count;
    hw->fs_scl_lcnt           = controller->scl_low_count;
    hw->sda_hold              = controller->sda_hold;
    hw->fs_spklen             = controller->spike_length;
    hw->rx_tl                 = 0;
    hw->tx_tl                 = 0;
    hw->intr_mask             = 0;
    controller->reset_pending = false;
    return true;
}

static void __not_in_flash_func(transfer_timeout)(void) {
    transfer.result.error = "BUS_TIMEOUT";
    transfer.busy         = false;
}

static void __not_in_flash_func(transfer_finish)(void) {
    if (transfer.busy || !transfer_owned)
        return;
    if (transfer.result.error) {
        /* A scheduled release DMA may still target ENABLE. Its owner must
         * quiesce it before cancel resets this controller, even after timeout. */
        if (transfer_scheduled)
            return;
        controller_release(transfer_bus);
    }
    transfer_owned = false;
}

const i2c_bus_config_t *i2c_bus_config(unsigned bus) {
    return bus < I2C_BUS_COUNT ? &configs[bus] : NULL;
}

const char *i2c_bus_configure(unsigned bus, unsigned sda, unsigned scl, uint32_t hz, bool pullups) {
    const char *error = i2c_bus_validate(bus, sda, scl, hz);
    if (error)
        return error;
    if (transfer_owned)
        return "BUSY";
    i2c_bus_config_t *config = &configs[bus];
    if (config->configured) {
        controller_release(bus);
        if (config->sda != sda) {
            gpio_set_dir(config->sda, GPIO_IN);
            gpio_set_dir(config->scl, GPIO_IN);
            gpio_set_function(config->sda, GPIO_FUNC_SIO);
            gpio_set_function(config->scl, GPIO_FUNC_SIO);
            gpio_disable_pulls(config->sda);
            gpio_disable_pulls(config->scl);
            gpio_set_oeover(config->sda, GPIO_OVERRIDE_NORMAL);
            gpio_set_oeover(config->scl, GPIO_OVERRIDE_NORMAL);
        }
    }
    /* Pins are released before switching mux, and mux precedes i2c_init. */
    gpio_set_oeover(sda, GPIO_OVERRIDE_LOW);
    gpio_set_oeover(scl, GPIO_OVERRIDE_LOW);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_set_pulls(sda, pullups, false);
    gpio_set_pulls(scl, pullups, false);
    i2c_init(controller_instance(bus), hz);
    i2c_hw_t *hw     = controller_instance(bus)->hw;
    hw->intr_mask    = 0;
    controllers[bus] = (controller_state_t) {.control        = hw->con,
                                             .scl_high_count = hw->fs_scl_hcnt,
                                             .scl_low_count  = hw->fs_scl_lcnt,
                                             .sda_hold       = hw->sda_hold,
                                             .spike_length   = hw->fs_spklen};
    *config          = (i2c_bus_config_t) {true, (uint8_t) sda, (uint8_t) scl, hz, pullups};
    gpio_set_oeover(sda, GPIO_OVERRIDE_NORMAL);
    gpio_set_oeover(scl, GPIO_OVERRIDE_NORMAL);
    return NULL;
}

static const char *__not_in_flash_func(transfer_begin)(unsigned bus, unsigned address, const uint8_t *write,
                                   size_t write_length, uint8_t *read, size_t read_length,
                                   uint64_t deadline_us, const uint16_t *commands, bool scheduled) {
    if (bus >= I2C_BUS_COUNT || address < 0x08 || address > 0x77)
        return "RANGE";
    if (!configs[bus].configured)
        return "UNCONFIGURED";
    if (transfer_owned)
        return "BUSY";
    const char *error = scheduled
        ? i2c_transfer_prepare(&transfer, commands, write_length, time_us_64(), deadline_us)
        : i2c_transfer_init(&transfer, write, write_length, read, read_length,
                              time_us_64(), deadline_us);
    if (error)
        return error;
    transfer_bus       = bus;
    transfer_address   = (uint8_t) address;
    transfer_phase     = TRANSFER_WAIT_FREE;
    transfer_owned     = true;
    transfer_scheduled = scheduled;
    transfer_stop_seen = false;
    i2c_bus_service();
    return NULL;
}

const char *__not_in_flash_func(i2c_bus_begin)(unsigned bus, unsigned address, const uint8_t *write, size_t write_length,
                          uint8_t *read, size_t read_length, uint64_t deadline_us) {
    return transfer_begin(bus, address, write, write_length, read, read_length, deadline_us, NULL, false);
}

const char *__not_in_flash_func(i2c_bus_prepare)(unsigned bus, unsigned address, const uint16_t *commands, size_t length,
                            uint64_t deadline_us) {
    if (!length || length > I2C_PREPARED_LIMIT)
        return "RANGE";
    return transfer_begin(bus, address, NULL, length, NULL, 0, deadline_us, commands, true);
}

bool __not_in_flash_func(i2c_bus_prepared)(void) {
    return transfer.busy && transfer_scheduled && transfer_phase == TRANSFER_ACTIVE &&
           transfer.writes_queued == transfer.write_length &&
           (controller_instance(transfer_bus)->hw->enable & I2C_IC_ENABLE_TX_CMD_BLOCK_BITS);
}

volatile uint32_t *__not_in_flash_func(i2c_bus_enable_register)(unsigned bus) {
    return bus < I2C_BUS_COUNT ? &controller_instance(bus)->hw->enable : NULL;
}

void __not_in_flash_func(i2c_bus_service)(void) {
    if (!transfer.busy)
        return;
    uint64_t  now = time_us_64();
    i2c_hw_t *hw  = controller_instance(transfer_bus)->hw;
    if (transfer_phase != TRANSFER_ACTIVE && now >= transfer.deadline_us) {
        transfer_timeout();
        transfer_finish();
        return;
    }
    if (!controller_restore(transfer_bus))
        return;
    uint32_t enable = I2C_IC_ENABLE_ENABLE_BITS |
                      (transfer_scheduled ? I2C_IC_ENABLE_TX_CMD_BLOCK_BITS : 0);
    if (transfer_phase == TRANSFER_WAIT_FREE) {
        /* Prearming needs an idle LOCAL controller, not ownership of the wire.
         * Only ordinary immediate transfers wait for high physical lines. */
        const i2c_bus_config_t *config = &configs[transfer_bus];
        uint32_t status = hw->status;
        if ((status & I2C_IC_STATUS_ACTIVITY_BITS) || !(status & I2C_IC_STATUS_TFE_BITS) ||
            hw->rxflr || (!transfer_scheduled && (!gpio_get(config->sda) || !gpio_get(config->scl))))
            return;
        if (controllers[transfer_bus].target_valid && hw->tar == transfer_address &&
            (hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS)) {
            /* TAR is reusable only after idle+empty; no disable flush or rewrite. */
            (void) hw->clr_intr;
            hw->enable     = enable;
            transfer_phase = TRANSFER_ACTIVE;
        } else {
            hw->enable     = 0;
            transfer_phase = TRANSFER_DISABLE;
        }
    }
    if (transfer_phase == TRANSFER_DISABLE) {
        if (hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS)
            return;
        (void) hw->clr_intr;
        hw->tar                                 = transfer_address;
        controllers[transfer_bus].target_valid = true;
        hw->enable                              = enable;
        gpio_set_oeover(configs[transfer_bus].sda, GPIO_OVERRIDE_NORMAL);
        gpio_set_oeover(configs[transfer_bus].scl, GPIO_OVERRIDE_NORMAL);
        transfer_phase                          = TRANSFER_ACTIVE;
    }
    /* Bound each service call, but fill available FIFO slots without waiting
     * for TX_EMPTY between bytes. Read commands retain the RX outstanding cap. */
    for (unsigned budget = 0; budget < I2C_PREPARED_LIMIT && transfer.busy; ++budget) {
        uint32_t interrupts = hw->raw_intr_stat;
        if (!(interrupts & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)) {
            unsigned available = hw->rxflr;
            for (unsigned received = 0; received < available && transfer.busy; ++received)
                i2c_transfer_receive(&transfer, (uint8_t) hw->data_cmd);
        }
        interrupts = hw->raw_intr_stat;
        uint32_t abort_source =
            interrupts & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS ? hw->tx_abrt_source : 0;
        if (interrupts & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) {
            (void) hw->clr_tx_abrt;
            if (!abort_source) {
                transfer_timeout();
                break;
            }
        }
        if (!abort_source && (interrupts & (I2C_IC_RAW_INTR_STAT_RX_OVER_BITS |
                                            I2C_IC_RAW_INTR_STAT_TX_OVER_BITS))) {
            transfer_timeout();
            break;
        }
        transfer.command_blocked = hw->enable & I2C_IC_ENABLE_TX_CMD_BLOCK_BITS;
        if (interrupts & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) {
            /* Retain completion until the final shift/controller status settles.
             * A blocked command cannot own a foreign master's STOP. */
            if (!transfer.command_blocked)
                transfer_stop_seen = true;
            (void) hw->clr_stop_det;
        }
        bool tx_empty = interrupts & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS;
        bool stop = !transfer.command_blocked && transfer_stop_seen && tx_empty &&
                    !(hw->status & I2C_IC_STATUS_MST_ACTIVITY_BITS) &&
                    transfer.reads_queued == transfer.read_length &&
                    transfer.writes_queued == transfer.write_length;
        uint16_t command;
        bool emit = i2c_transfer_advance(&transfer, now, abort_source, tx_empty,
                                         hw->status & I2C_IC_STATUS_TFNF_BITS, stop, &command);
        if (!emit)
            break;
        hw->data_cmd = command;
    }
    transfer_finish();
}

bool __not_in_flash_func(i2c_bus_busy)(void) {
    return transfer.busy;
}

const i2c_bus_result_t *__not_in_flash_func(i2c_bus_result)(void) {
    return &transfer.result;
}

void __not_in_flash_func(i2c_bus_cancel)(void) {
    if (!transfer_owned)
        return;
    i2c_transfer_cancel(&transfer);
    controller_release(transfer_bus);
    transfer_owned = false;
}
