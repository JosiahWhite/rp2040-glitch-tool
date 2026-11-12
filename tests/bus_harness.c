#include "i2c_bus.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define READ (1u << 8)
#define STOP (1u << 9)
#define RESTART (1u << 10)

static void transfer_checks(void) {
    assert(!i2c_bus_validate(0, 4, 5, 100000));
    assert(!i2c_bus_validate(1, 6, 7, 400000));
    assert(!i2c_bus_validate(0, 28, 29, 10000));
    assert(!i2c_bus_validate(0, 4, 5, 400001));
    assert(!i2c_bus_validate(0, 4, 5, 1000000));
    assert(!i2c_bus_validate(0, 4, 5, 1200000));
    assert(!i2c_bus_validate(1, 6, 7, 1500000));
    assert(i2c_bus_validate(0, 6, 7, 100000));
    assert(i2c_bus_validate(1, 4, 5, 100000));
    assert(i2c_bus_validate(2, 4, 5, 100000));
    assert(i2c_bus_validate(0, 4, 6, 100000));
    assert(i2c_bus_validate(0, 30, 31, 100000));
    assert(i2c_bus_validate(0, 4, 5, 9999));
    assert(i2c_bus_validate(0, 4, 5, 1500001));

    uint8_t write[] = {0x10, 0x42};
    uint8_t read[24] = {0};
    i2c_transfer_t t, before;
    memset(&t, 0, sizeof(t));
    memcpy(&before, &t, sizeof(t));
    assert(i2c_transfer_init(&t, NULL, 1, read, 1, 100, 1000));
    assert(!memcmp(&before, &t, sizeof(t)));
    assert(i2c_transfer_init(&t, write, 1025, read, 1, 100, 1000));
    assert(i2c_transfer_init(&t, write, 1, read, 1025, 100, 1000));
    assert(i2c_transfer_init(&t, write, 0, read, 0, 100, 1000));
    assert(i2c_transfer_init(&t, write, 1, NULL, 1, 100, 1000));
    assert(i2c_transfer_init(&t, write, 1, read, 1, 100, 100));
    assert(i2c_transfer_init(&t, write, 1, read, 1, 100, 60000101));
    assert(!memcmp(&before, &t, sizeof(t)));

    uint16_t cmd = 0xffff;
    assert(!i2c_transfer_init(&t, write, 2, NULL, 0, 100, 1000));
    assert(i2c_transfer_advance(&t, 101, 0, true, true, false, &cmd));
    assert(cmd == 0x10 && t.result.written == 0);
    assert(i2c_transfer_advance(&t, 102, 0, false, true, false, &cmd));
    assert(cmd == (0x42 | STOP) && t.result.written == 0 && t.writes_queued == 2);
    assert(!i2c_transfer_advance(&t, 103, 0, false, true, false, &cmd));
    assert(t.busy && t.result.written == 0);
    assert(!i2c_transfer_advance(&t, 104, 0, true, true, false, &cmd));
    assert(t.busy && t.result.written == 2);
    assert(!i2c_transfer_advance(&t, 105, 0, true, true, true, &cmd));
    assert(!t.busy && !t.result.error && t.result.read == 0);

    assert(!i2c_transfer_init(&t, write, 1, read, 2, 100, 1000));
    assert(i2c_transfer_advance(&t, 101, 0, true, true, false, &cmd) && cmd == 0x10);
    assert(i2c_transfer_advance(&t, 102, 0, false, true, false, &cmd));
    assert(cmd == (READ | RESTART));
    assert(i2c_transfer_advance(&t, 103, 0, false, true, false, &cmd));
    assert(cmd == (READ | STOP));
    i2c_transfer_receive(&t, 0xaa);
    i2c_transfer_receive(&t, 0x55);
    assert(!i2c_transfer_advance(&t, 104, 0, true, true, false, &cmd) && t.busy);
    assert(!i2c_transfer_advance(&t, 105, 0, true, true, true, &cmd));
    assert(!t.busy && !t.result.error && t.result.written == 1 && t.result.read == 2);
    assert(read[0] == 0xaa && read[1] == 0x55);

    assert(!i2c_transfer_init(&t, NULL, 0, read, 20, 100, 1000));
    for (unsigned n = 0; n < 16; ++n) {
        assert(i2c_transfer_advance(&t, 101 + n, 0, false, true, false, &cmd));
        assert(cmd == READ);
    }
    assert(!i2c_transfer_advance(&t, 120, 0, true, true, false, &cmd));
    for (unsigned n = 0; n < 8; ++n) i2c_transfer_receive(&t, (uint8_t)n);
    for (unsigned n = 16; n < 20; ++n) {
        assert(i2c_transfer_advance(&t, 121 + n, 0, false, true, false, &cmd));
        assert(cmd == (READ | (n == 19 ? STOP : 0)));
    }
    for (unsigned n = 8; n < 20; ++n) i2c_transfer_receive(&t, (uint8_t)n);
    assert(!i2c_transfer_advance(&t, 150, 0, true, true, true, &cmd));
    assert(!t.busy && !t.result.error && t.result.read == 20);
    for (unsigned n = 0; n < 20; ++n) assert(read[n] == n);

    const uint32_t aborts[] = {
        1u, 8u, 1u << 12, (1u << 12) | 1u, 1u << 11,
        (1u << 23) | 1u, (1u << 23) | 8u, (1u << 23) | (1u << 12)
    };
    const char *errors[] = {
        "NACK", "NACK", "ARBITRATION_LOST", "ARBITRATION_LOST", "BUS_TIMEOUT",
        "NACK", "NACK", "ARBITRATION_LOST"
    };
    for (unsigned n = 0; n < sizeof(aborts) / sizeof(aborts[0]); ++n) {
        assert(!i2c_transfer_init(&t, write, 2, NULL, 0, 100, 1000));
        assert(i2c_transfer_advance(&t, 101, 0, false, true, false, &cmd));
        assert(i2c_transfer_advance(&t, 102, 0, false, true, false, &cmd));
        /* Abort wins over simultaneous deadline, TX_EMPTY and STOP. Neither a
         * flushed FIFO nor the NACKed, already-popped byte proves an ACK. */
        assert(!i2c_transfer_advance(&t, 1000, aborts[n], true, true, true, &cmd));
        assert(!t.busy && !strcmp(t.result.error, errors[n]));
        assert(t.result.raw_abort == aborts[n] && t.result.written == 0);
        /* No STOP observation is needed, notably after arbitration loss. */
    }
    assert(!i2c_transfer_init(&t, NULL, 0, read, 2, 100, 1000));
    assert(i2c_transfer_advance(&t, 101, 0, true, true, false, &cmd));
    assert(i2c_transfer_advance(&t, 102, 0, true, true, false, &cmd));
    i2c_transfer_receive(&t, 7);
    assert(!i2c_transfer_advance(&t, 103, 0, true, true, true, &cmd));
    assert(!t.busy && !strcmp(t.result.error, "BUS_TIMEOUT") && t.result.read == 1);

    assert(!i2c_transfer_init(&t, write, 1, NULL, 0, 100, 200));
    assert(!i2c_transfer_advance(&t, 199, 0, false, false, false, &cmd) && t.busy);
    assert(!i2c_transfer_advance(&t, 200, 0, false, false, false, &cmd));
    assert(!t.busy && !strcmp(t.result.error, "BUS_TIMEOUT"));
    assert(!i2c_transfer_init(&t, write, 1, NULL, 0, 100, 200));
    i2c_transfer_cancel(&t);
    assert(!t.busy && !strcmp(t.result.error, "ABORTED"));
    memcpy(&before, &t, sizeof(t));
    i2c_transfer_cancel(&t);
    assert(!memcmp(&before, &t, sizeof(t)));

    assert(!i2c_transfer_init(&t, NULL, 0, read, 1, 100, 1000));
    assert(i2c_transfer_advance(&t, 101, 0, true, true, false, &cmd));
    read[1] = 0xcc;
    i2c_transfer_receive(&t, 0x33);
    i2c_transfer_receive(&t, 0x44);
    assert(!t.busy && !strcmp(t.result.error, "BUS_TIMEOUT"));
    assert(read[0] == 0x33 && read[1] == 0xcc);
}

static void batching_checks(void) {
    uint8_t write[I2C_TRANSFER_LIMIT], read[I2C_TRANSFER_LIMIT];
    for (size_t n = 0; n < sizeof(write); ++n) write[n] = (uint8_t)n;
    i2c_transfer_t t;
    uint16_t command;
    assert(!i2c_transfer_init(&t, write, sizeof(write), NULL, 0, 0, 10000));
    for (size_t n = 0; n < sizeof(write); ++n) {
        if (n && n % 16 == 0) {
            /* A completed batch is retained as the conservative ACK bound. */
            assert(!i2c_transfer_advance(&t, n, 0, true, false, false, &command));
            assert(t.result.written == n);
        }
        assert(i2c_transfer_advance(&t, n, 0, false, true, false, &command));
        assert(command == (write[n] | (n + 1 == sizeof(write) ? STOP : 0)));
    }
    assert(t.busy && t.writes_queued == sizeof(write));
    assert(t.result.written == sizeof(write) - 16);
    assert(!i2c_transfer_advance(&t, 1100, 0, true, true, true, &command));
    assert(!t.busy && !t.result.error && t.result.written == sizeof(write));

    assert(!i2c_transfer_init(&t, write, sizeof(write), NULL, 0, 0, 10000));
    for (unsigned n = 0; n < 16; ++n)
        assert(i2c_transfer_advance(&t, 1, 0, false, true, false, &command));
    assert(!i2c_transfer_advance(&t, 2, 0, true, false, false, &command));
    assert(t.result.written == 16);
    for (unsigned n = 0; n < 16; ++n)
        assert(i2c_transfer_advance(&t, 3, 0, false, true, false, &command));
    assert(!i2c_transfer_advance(&t, 4, (15u << 23) | 8u, true, true, true, &command));
    assert(!strcmp(t.result.error, "NACK") && t.result.written == 16 && t.writes_queued == 32);

    /* Prepared bytes remain queued, not acknowledged, even if a stale empty or
     * unrelated STOP observation is supplied while hardware execution blocks. */
    assert(!i2c_transfer_init(&t, write, I2C_PREPARED_LIMIT, NULL, 0, 0, 10000));
    t.command_blocked = true;
    for (unsigned n = 0; n < I2C_PREPARED_LIMIT; ++n) {
        assert(i2c_transfer_advance(&t, 1, 0, true, true, true, &command));
        assert(command == (write[n] | (n + 1 == I2C_PREPARED_LIMIT ? STOP : 0)));
        assert(t.result.written == 0);
    }
    assert(!i2c_transfer_advance(&t, 2, 0, true, true, true, &command));
    assert(t.busy && t.result.written == 0);
    t.command_blocked = false;
    assert(!i2c_transfer_advance(&t, 3, 0, false, true, false, &command));
    assert(t.busy && !t.result.written);
    assert(!i2c_transfer_advance(&t, 4, 0, true, true, true, &command));
    assert(!t.busy && !t.result.error && t.result.written == I2C_PREPARED_LIMIT);

    assert(!i2c_transfer_init(&t, write, 16, NULL, 0, 0, 10));
    t.command_blocked = true;
    for (unsigned n = 0; n < 16; ++n)
        assert(i2c_transfer_advance(&t, 1, 0, true, true, false, &command));
    assert(!i2c_transfer_advance(&t, 10, 0, true, true, true, &command));
    assert(!t.busy && !strcmp(t.result.error, "BUS_TIMEOUT") && !t.result.written);

    /* Delayed reads never allow more than sixteen commands outstanding. */
    assert(!i2c_transfer_init(&t, write, 2, read, sizeof(read), 0, 10000));
    assert(i2c_transfer_advance(&t, 1, 0, false, true, false, &command));
    assert(command == write[0]);
    assert(i2c_transfer_advance(&t, 1, 0, false, true, false, &command));
    assert(command == write[1]);
    for (unsigned base = 0; base < sizeof(read); base += 16) {
        for (unsigned n = base; n < base + 16; ++n) {
            assert(i2c_transfer_advance(&t, 2, 0, false, true, false, &command));
            assert(command == (READ | (!n ? RESTART : 0) |
                               (n + 1 == sizeof(read) ? STOP : 0)));
        }
        assert(!i2c_transfer_advance(&t, 3, 0, false, true, false, &command));
        for (unsigned n = base; n < base + 16; ++n) i2c_transfer_receive(&t, (uint8_t)n);
    }
    assert(!i2c_transfer_advance(&t, 4, 0, true, true, true, &command));
    assert(!t.busy && !t.result.error && t.result.written == 2 && t.result.read == sizeof(read));
    assert(!memcmp(write, read, sizeof(read)));
}

int main(void) {
    transfer_checks();
    batching_checks();
    puts("production transfer checks passed");
    return 0;
}
