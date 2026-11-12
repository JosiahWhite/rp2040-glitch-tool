#include "i2c_bus.h"

#ifdef PICO_ON_DEVICE
#include "pico.h"
#else
#define __not_in_flash_func(name) name
#endif

/* RP2040 IC_DATA_CMD and TX_ABRT_SOURCE encodings, shared with native tests. */
#define CMD_READ       (1u << 8)
#define CMD_STOP       (1u << 9)
#define CMD_RESTART    (1u << 10)
#define ABORT_ARB_LOST (1u << 12)
#define ABORT_NACK     0x1fu

const char *i2c_bus_validate(unsigned bus, unsigned sda, unsigned scl, uint32_t hz) {
    if (bus >= I2C_BUS_COUNT || sda >= 29 || scl != sda + 1 || sda % 4 != bus * 2 || hz < 10000 ||
        hz > 1500000)
        return "RANGE";
    return NULL;
}

const char *__not_in_flash_func(i2c_transfer_init)(i2c_transfer_t *transfer, const uint8_t *write, size_t write_length,
                              uint8_t *read, size_t read_length, uint64_t now_us,
                              uint64_t deadline_us) {
    if (write_length > I2C_TRANSFER_LIMIT || read_length > I2C_TRANSFER_LIMIT ||
        (!write_length && !read_length) || (write_length && !write) || (read_length && !read) ||
        deadline_us <= now_us || deadline_us - now_us > I2C_DEADLINE_LIMIT_US)
        return "RANGE";
    *transfer = (i2c_transfer_t) {.write        = write,
                                  .read         = read,
                                  .write_length = write_length,
                                  .read_length  = read_length,
                                  .deadline_us  = deadline_us,
                                  .busy         = true};
    return NULL;
}

const char *__not_in_flash_func(i2c_transfer_prepare)(i2c_transfer_t *transfer, const uint16_t *commands,
                                                     size_t length, uint64_t now_us,
                                                     uint64_t deadline_us) {
    if (!commands || !length || length > I2C_PREPARED_LIMIT ||
        deadline_us <= now_us || deadline_us - now_us > I2C_DEADLINE_LIMIT_US)
        return "RANGE";
    for (size_t i = 0; i < length; ++i) {
        if ((commands[i] & ~0xffu) != (i + 1 == length ? CMD_STOP : 0))
            return "RANGE";
    }
    *transfer = (i2c_transfer_t) {.commands     = commands,
                                  .write_length = length,
                                  .deadline_us  = deadline_us,
                                  .busy         = true};
    return NULL;
}

static void __not_in_flash_func(transfer_fail)(i2c_transfer_t *transfer, const char *error) {
    transfer->result.error = error;
    transfer->busy         = false;
}

void __not_in_flash_func(i2c_transfer_receive)(i2c_transfer_t *transfer, uint8_t value) {
    if (!transfer->busy)
        return;
    if (transfer->result.read >= transfer->read_length ||
        transfer->result.read >= transfer->reads_queued) {
        transfer_fail(transfer, "BUS_TIMEOUT");
        return;
    }
    transfer->read[transfer->result.read++] = value;
}

bool __not_in_flash_func(i2c_transfer_advance)(i2c_transfer_t *transfer, uint64_t now_us, uint32_t raw_abort,
                          bool tx_empty, bool tx_space, bool stop, uint16_t *command) {
    if (!transfer->busy)
        return false;
    if (raw_abort) {
        transfer->result.raw_abort = raw_abort;
        if (raw_abort & ABORT_ARB_LOST) {
            transfer_fail(transfer, "ARBITRATION_LOST");
        } else if (raw_abort & ABORT_NACK) {
            transfer_fail(transfer, "NACK");
        } else {
            transfer_fail(transfer, "BUS_TIMEOUT");
        }
        return false;
    }
    if (now_us >= transfer->deadline_us) {
        transfer_fail(transfer, "BUS_TIMEOUT");
        return false;
    }
    /* TX_EMPTY_CTRL includes the last shift-register command. FIFO submission
     * and abort flush counts do not prove ACKs; retain only confirmed batches.
     * A blocked FIFO can never contribute to the acknowledged lower bound. */
    if (!transfer->command_blocked && tx_empty)
        transfer->result.written = transfer->writes_queued;
    if (!transfer->command_blocked && stop) {
        if (!tx_empty || transfer->writes_queued != transfer->write_length ||
            transfer->result.read != transfer->read_length ||
            transfer->reads_queued != transfer->read_length) {
            transfer_fail(transfer, "BUS_TIMEOUT");
        } else {
            transfer->busy = false;
        }
        return false;
    }
    if (!tx_space)
        return false;
    if (transfer->writes_queued < transfer->write_length) {
        if (transfer->commands) {
            *command = transfer->commands[transfer->writes_queued++];
        } else {
            *command = transfer->write[transfer->writes_queued++];
            if (transfer->writes_queued == transfer->write_length && !transfer->read_length)
                *command |= CMD_STOP;
        }
        return true;
    }
    /* Outstanding reads include RX FIFO bytes, so an arbitrarily late service
     * call cannot make the peripheral overrun its sixteen-byte RX FIFO. */
    if (transfer->reads_queued < transfer->read_length &&
        transfer->reads_queued - transfer->result.read < 16) {
        *command = CMD_READ;
        if (!transfer->reads_queued && transfer->write_length)
            *command |= CMD_RESTART;
        if (++transfer->reads_queued == transfer->read_length)
            *command |= CMD_STOP;
        return true;
    }
    return false;
}

void __not_in_flash_func(i2c_transfer_cancel)(i2c_transfer_t *transfer) {
    if (transfer->busy)
        transfer_fail(transfer, "ABORTED");
}
