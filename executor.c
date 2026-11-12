#include "executor.h"

#include <stdatomic.h>

#ifdef EXECUTOR_RUNTIME_TEST
#include "tests/executor_runtime_shim.h"
#else
#include "pico/multicore.h"
#include "pico/stdlib.h"
#endif

/* One core0 producer, one core1 consumer. A fresh ticket distinguishes repeated
 * commands; release/acquire publishes all plain fields and staged DMA buffers.
 * No atomic RMW operations (which need software locks on Cortex-M0+). */
enum { CONTROL_ACQUIRE, CONTROL_RELEASE, CONTROL_STATUS, CONTROL_ABORT };
static struct {
    _Atomic uint32_t request, acknowledgment;
    _Atomic bool    busy;
    bool            acquired;
    executor_status_t snapshot;
} control;
static uint32_t ticket; /* Core0 only. */
static uint8_t *received;

static void __not_in_flash_func(executor_worker)(void) {
    uint32_t seen = 0;
    bool parked = false;
    for (;;) {
        uint32_t request = atomic_load_explicit(&control.request, memory_order_acquire);
        /* Ownership and cancellation precede service. STATUS waits for the
         * bounded completion/refill pass instead of delaying it with a copy. */
        if (request != seen && (request & 3u) != CONTROL_STATUS) {
            switch (request & 3u) {
                case CONTROL_ACQUIRE:
                    control.acquired = !sequence_running() && !i2c_bus_busy();
                    parked = control.acquired;
                    break;
                case CONTROL_RELEASE:
                    parked = false;
                    break;
                case CONTROL_ABORT:
                    sequence_abort();
                    break;
            }
            atomic_store_explicit(&control.busy, sequence_running() || i2c_bus_busy(),
                                  memory_order_release);
            seen = request;
            atomic_store_explicit(&control.acknowledgment, request, memory_order_release);
        }
        if (parked) {
            tight_loop_contents();
            continue;
        }
        i2c_bus_service();
        timing_service();
        sequence_service(received);
        atomic_store_explicit(&control.busy, sequence_running() || i2c_bus_busy(),
                              memory_order_release);
        /* Keep servicing even while idle: a held PWM carrier can still fault. */
        if (request != seen) {
            control.snapshot.sequence = *sequence_status();
            control.snapshot.pwm = *timing_pwm();
            if (control.snapshot.sequence.state == SEQUENCE_RUNNING)
                control.snapshot.sequence.run_elapsed =
                    time_us_64() - control.snapshot.sequence.run_start;
            seen = request;
            atomic_store_explicit(&control.acknowledgment, request, memory_order_release);
        }
    }
}

static void request_control(uint32_t command) {
    ticket += 4u;
    uint32_t request = ticket | command;
    atomic_store_explicit(&control.request, request, memory_order_release);
    while (atomic_load_explicit(&control.acknowledgment, memory_order_acquire) != request)
        tight_loop_contents();
}

void executor_init(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]) {
    received = receive_buffer;
    ticket = 0;
    atomic_init(&control.request, 0);
    atomic_init(&control.acknowledgment, 0);
    atomic_init(&control.busy, false);
    multicore_launch_core1(executor_worker);
}

bool executor_acquire(void) {
    request_control(CONTROL_ACQUIRE);
    return control.acquired;
}

void executor_release(void) {
    request_control(CONTROL_RELEASE);
}

bool executor_busy(void) {
    return atomic_load_explicit(&control.busy, memory_order_acquire);
}

void executor_status(executor_status_t *out) {
    request_control(CONTROL_STATUS);
    *out = control.snapshot;
}

void executor_abort(void) {
    request_control(CONTROL_ABORT);
}
