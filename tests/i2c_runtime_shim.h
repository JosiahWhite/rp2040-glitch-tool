/* Register snapshots for the production i2c_hw.c controller lifecycle. The
 * transfer core separately checks every FIFO command; this is not a wire model. */
#ifndef I2C_RUNTIME_SHIM_H
#define I2C_RUNTIME_SHIM_H
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define __not_in_flash_func(name) name

#define I2C_IC_DATA_CMD_CMD_BITS (1u << 8)
#define I2C_IC_DATA_CMD_STOP_BITS (1u << 9)
#define I2C_IC_DATA_CMD_RESTART_BITS (1u << 10)
#define I2C_IC_TX_ABRT_SOURCE_ARB_LOST_BITS (1u << 12)
#define I2C_IC_ENABLE_ENABLE_BITS 1u
#define I2C_IC_ENABLE_TX_CMD_BLOCK_BITS 4u
#define I2C_IC_ENABLE_STATUS_IC_EN_BITS 1u
#define I2C_IC_STATUS_ACTIVITY_BITS 1u
#define I2C_IC_STATUS_TFNF_BITS 2u
#define I2C_IC_STATUS_TFE_BITS 4u
#define I2C_IC_STATUS_MST_ACTIVITY_BITS (1u << 5)
#define I2C_IC_RAW_INTR_STAT_RX_OVER_BITS (1u << 1)
#define I2C_IC_RAW_INTR_STAT_TX_OVER_BITS (1u << 3)
#define I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS (1u << 4)
#define I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS (1u << 6)
#define I2C_IC_RAW_INTR_STAT_STOP_DET_BITS (1u << 9)
#define RESETS_RESET_I2C0_BITS (1u << 3)
#define RESETS_RESET_I2C1_BITS (1u << 4)

enum { GPIO_IN, GPIO_FUNC_SIO, GPIO_FUNC_I2C, GPIO_OVERRIDE_NORMAL, GPIO_OVERRIDE_LOW };
typedef struct {
    volatile uint32_t enable, enable_status, con, fs_scl_hcnt, fs_scl_lcnt, sda_hold, fs_spklen;
    volatile uint32_t rx_tl, tx_tl, intr_mask, status, tar, rxflr, txflr, raw_intr_stat;
    volatile uint32_t tx_abrt_source, clr_intr, clr_tx_abrt, clr_stop_det, data_cmd;
} i2c_hw_t;
typedef struct { i2c_hw_t *hw; bool restart_on_next; } i2c_inst_t;
static i2c_hw_t native_regs[2];
static i2c_inst_t native_instances[2] = {{&native_regs[0], false}, {&native_regs[1], false}};
#define i2c0 (&native_instances[0])
#define i2c1 (&native_instances[1])
static struct { uint32_t reset_done; } native_resets;
#define resets_hw (&native_resets)
static uint64_t native_now, native_now_step;
static unsigned native_reset_count, native_overrides[30], native_functions[30];
static bool native_lines_high, native_release_armed;

static inline uint64_t time_us_64(void) {
    uint64_t now = native_now;
    native_now += native_now_step;
    return now;
}
static inline bool gpio_get(unsigned pin) { (void)pin; return native_lines_high; }
static inline void gpio_set_oeover(unsigned pin, unsigned value) { native_overrides[pin] = value; }
static inline void gpio_set_function(unsigned pin, unsigned value) { native_functions[pin] = value; }
static inline void gpio_set_dir(unsigned pin, unsigned value) { (void)pin; (void)value; }
static inline void gpio_disable_pulls(unsigned pin) { (void)pin; }
static inline void gpio_set_pulls(unsigned pin, bool up, bool down) { (void)pin; (void)up; (void)down; }
static inline void reset_block_mask(uint32_t mask) {
    /* This fails if service resets a failed prepared controller before its
     * caller has quiesced the external release DMA. */
    assert(!native_release_armed);
    ++native_reset_count;
    unsigned bus = mask == RESETS_RESET_I2C0_BITS ? 0u : 1u;
    memset(&native_regs[bus], 0, sizeof(native_regs[bus]));
    native_regs[bus].status = I2C_IC_STATUS_TFE_BITS | I2C_IC_STATUS_TFNF_BITS;
    native_resets.reset_done &= ~mask;
}
static inline void unreset_block_mask(uint32_t mask) { native_resets.reset_done |= mask; }
static inline unsigned i2c_init(i2c_inst_t *instance, uint32_t hz) {
    *instance->hw = (i2c_hw_t){
        .enable = 1, .enable_status = 1, .con = 0x165,
        .fs_scl_hcnt = 125000000u / hz / 4u, .fs_scl_lcnt = 125000000u / hz * 3u / 4u,
        .sda_hold = hz < 1000000u ? 38u : 16u, .fs_spklen = 1,
        .status = I2C_IC_STATUS_TFE_BITS | I2C_IC_STATUS_TFNF_BITS,
        .raw_intr_stat = I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS,
    };
    return hz;
}
#endif
