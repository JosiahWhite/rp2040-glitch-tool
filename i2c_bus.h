#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define I2C_BUS_COUNT         2u
#define I2C_TRANSFER_LIMIT    1024u
#define I2C_PREPARED_LIMIT    16u
#define I2C_DEADLINE_LIMIT_US 60000000u

typedef struct {
    bool     configured;
    uint8_t  sda, scl;
    uint32_t hz;
    bool     pullups;
} i2c_bus_config_t;

typedef struct {
    const char *error;
    uint32_t    raw_abort;
    size_t      written, read; /* written is a confirmed ACK lower bound on error. */
} i2c_bus_result_t;

/* Configuration validates local pin/mux/rate constraints before hardware changes.
 * The caller owns cross-module pin collision and run-state validation. */
const char             *i2c_bus_validate(unsigned bus, unsigned sda, unsigned scl, uint32_t hz);
const i2c_bus_config_t *i2c_bus_config(unsigned bus);
const char *i2c_bus_configure(unsigned bus, unsigned sda, unsigned scl, uint32_t hz, bool pullups);
/* Buffers remain owned by the caller and valid until busy() becomes false.
 * deadline_us is an absolute monotonic microsecond deadline. begin/prepare each
 * service the controller once, without waiting, and queue up to sixteen commands.
 * NULL accepts the request; inspect busy()/result() even on that first call. */
const char *i2c_bus_begin(unsigned bus, unsigned address, const uint8_t *write, size_t write_length,
                          uint8_t *read, size_t read_length, uint64_t deadline_us);
/* Preload encoded write-only commands with TX_CMD_BLOCK set; preparation never
 * drives the bus. Each word contains only data bits 7:0, with STOP (bit 9) on
 * exactly the final word. At most I2C_PREPARED_LIMIT commands may be borrowed.
 * The deadline includes preparation, the blocked wait, and transmission.
 * Only hardware release clears the block. On failure stop release DMA BEFORE
 * cancel(); the failed controller remains owned, even after busy() is false. */
const char *i2c_bus_prepare(unsigned bus, unsigned address, const uint16_t *commands, size_t length,
                            uint64_t deadline_us);
bool        i2c_bus_prepared(void);
volatile uint32_t *i2c_bus_enable_register(unsigned bus);
void        i2c_bus_service(void);
bool        i2c_bus_busy(void);
const i2c_bus_result_t *i2c_bus_result(void);
void                    i2c_bus_cancel(void);

/* Transfer core consumes actual hardware observations, never sleeps or retries.
 * Commands use the RP2040 IC_DATA_CMD bit layout. command_blocked is the current
 * hardware block bit; tx_empty requires TX_EMPTY_CTRL=1 and TX_TL=0. */
typedef struct {
    const uint8_t   *write;
    const uint16_t  *commands; /* Pre-encoded scheduled writes, otherwise NULL. */
    uint8_t         *read;
    size_t           write_length, read_length, writes_queued, reads_queued;
    uint64_t         deadline_us;
    bool             busy, command_blocked;
    i2c_bus_result_t result;
} i2c_transfer_t;
const char *i2c_transfer_init(i2c_transfer_t *transfer, const uint8_t *write, size_t write_length,
                              uint8_t *read, size_t read_length, uint64_t now_us,
                              uint64_t deadline_us);
const char *i2c_transfer_prepare(i2c_transfer_t *transfer, const uint16_t *commands, size_t length,
                                 uint64_t now_us, uint64_t deadline_us);
void        i2c_transfer_receive(i2c_transfer_t *transfer, uint8_t value);
bool        i2c_transfer_advance(i2c_transfer_t *transfer, uint64_t now_us, uint32_t raw_abort,
                                 bool tx_empty, bool tx_space, bool stop, uint16_t *command);
void        i2c_transfer_cancel(i2c_transfer_t *transfer);

#endif
