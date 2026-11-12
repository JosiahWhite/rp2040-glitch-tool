/* Real core0/core1 threads exercise the production mailbox. Runtime models
 * assert exclusive ownership; condition-variable barriers replace sleeps. */
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define EXECUTOR_RUNTIME_TEST
#include "../executor.c"

static pthread_t worker;
static _Thread_local bool on_worker;
static _Atomic bool host_owned, shutdown_worker, complete_transfer, inject_pwm_fault;
static _Atomic bool abort_returned;
static pthread_mutex_t progress_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t progress_changed = PTHREAD_COND_INITIALIZER;
static uint64_t service_ticks, park_spins;
static bool gate_abort, abort_entered, finish_abort;
static bool gate_service, service_entered, finish_service;

/* These are deliberately plain fields, just like the production runtime. */
static sequence_status_t run;
static timing_pwm_t pwm;
static i2c_bus_result_t result;
static bool bus_busy, dma_active, pio_active, controller_active, pwm_fault;
static uint64_t model_time = 100;
static uint8_t rx[I2C_TRANSFER_LIMIT], payload[4] = {0x23, 0xd4, 0x00, 0x51};
static const uint8_t *borrowed_payload;
static uint8_t *borrowed_rx;
static uint8_t expected_payload[sizeof(payload)];

static void assert_owner(void) {
    assert(on_worker ? !atomic_load(&host_owned) : atomic_load(&host_owned));
}

static struct timespec deadline(void) {
    struct timespec until;
    assert(timespec_get(&until, TIME_UTC) == TIME_UTC);
    until.tv_sec += 5;
    return until;
}

static uint64_t progress(bool parked) {
    assert(!pthread_mutex_lock(&progress_lock));
    uint64_t value = parked ? park_spins : service_ticks;
    assert(!pthread_mutex_unlock(&progress_lock));
    return value;
}

static void wait_progress(bool parked, uint64_t target) {
    struct timespec until = deadline();
    assert(!pthread_mutex_lock(&progress_lock));
    while ((parked ? park_spins : service_ticks) < target)
        assert(!pthread_cond_timedwait(&progress_changed, &progress_lock, &until));
    assert(!pthread_mutex_unlock(&progress_lock));
}

static void publish_progress(bool parked) {
    assert(!pthread_mutex_lock(&progress_lock));
    if (parked)
        ++park_spins;
    else
        ++service_ticks;
    if (!parked && gate_service) {
        service_entered = true;
        assert(!pthread_cond_broadcast(&progress_changed));
        struct timespec until = deadline();
        while (!finish_service)
            assert(!pthread_cond_timedwait(&progress_changed, &progress_lock, &until));
        gate_service = service_entered = false;
    }
    assert(!pthread_cond_broadcast(&progress_changed));
    assert(!pthread_mutex_unlock(&progress_lock));
}

void tight_loop_contents(void) {
    if (on_worker) {
        if (atomic_load(&shutdown_worker))
            pthread_exit(NULL);
        publish_progress(true);
    } else {
        assert(!pthread_mutex_lock(&progress_lock));
        if (service_entered &&
            atomic_load(&control.request) != atomic_load(&control.acknowledgment)) {
            finish_service = true;
            assert(!pthread_cond_broadcast(&progress_changed));
        }
        assert(!pthread_mutex_unlock(&progress_lock));
    }
    sched_yield();
}

static void *launch(void *argument) {
    (void) argument;
    on_worker = true;
    executor_worker();
    return NULL;
}

void multicore_launch_core1(void (*entry)(void)) {
    assert(entry == executor_worker);
    assert(!pthread_create(&worker, NULL, launch, NULL));
}

uint64_t time_us_64(void) {
    assert_owner();
    return model_time;
}

bool sequence_running(void) {
    assert_owner();
    return run.state == SEQUENCE_RUNNING;
}

const sequence_status_t *sequence_status(void) {
    assert_owner();
    return &run;
}

const timing_pwm_t *timing_pwm(void) {
    assert_owner();
    return &pwm;
}

bool i2c_bus_busy(void) {
    assert_owner();
    return bus_busy;
}

const i2c_bus_result_t *i2c_bus_result(void) {
    assert_owner();
    return &result;
}

void i2c_bus_service(void) {
    assert(on_worker);
    assert_owner();
    if (!bus_busy)
        return;
    assert(borrowed_rx == rx && borrowed_payload == payload);
    assert(!memcmp(borrowed_payload, expected_payload, sizeof(payload)));
    borrowed_rx[0] = borrowed_payload[0] ^ 0xffu;
    if (atomic_load(&complete_transfer)) {
        result.written = sizeof(payload);
        result.read = 1;
        bus_busy = controller_active = false;
        borrowed_payload = NULL;
        borrowed_rx = NULL;
    }
}

void timing_service(void) {
    assert(on_worker);
    assert_owner();
    if (pwm.active && atomic_load(&inject_pwm_fault)) {
        pwm_fault = true;
        pwm.active = dma_active = pio_active = false;
    }
}

void sequence_service(uint8_t receive_buffer[I2C_TRANSFER_LIMIT]) {
    assert(on_worker && receive_buffer == rx);
    assert_owner();
    ++model_time;
    if (pwm_fault) {
        run.state = SEQUENCE_ERROR;
        run.run_error = "TIMING_ERROR";
        pwm_fault = false;
    } else if (run.state == SEQUENCE_RUNNING) {
        uint32_t generation = ++run.poll_attempts;
        run.poll_length = sizeof(run.poll_data);
        memset(run.poll_data, (uint8_t) generation, sizeof(run.poll_data));
        run.programmed_cycles = ((uint64_t) generation << 32) | generation;
        run.inserted_cycles = ~run.programmed_cycles;
        pwm.final_duty_basis_points = generation % 10001u;
        if (!bus_busy) {
            run.state = SEQUENCE_DONE;
            run.run_elapsed = model_time - run.run_start;
        }
    }
    publish_progress(false);
}

void sequence_abort(void) {
    assert(on_worker);
    assert_owner();
    /* The mailbox must not acknowledge any cancellation until teardown ends. */
    assert(atomic_load(&control.acknowledgment) != atomic_load(&control.request));
    dma_active = false;
    assert(!pthread_mutex_lock(&progress_lock));
    if (gate_abort) {
        abort_entered = true;
        assert(!pthread_cond_broadcast(&progress_changed));
        struct timespec until = deadline();
        while (!finish_abort)
            assert(!pthread_cond_timedwait(&progress_changed, &progress_lock, &until));
    }
    assert(!pthread_mutex_unlock(&progress_lock));
    pio_active = controller_active = bus_busy = pwm.active = false;
    borrowed_payload = NULL;
    borrowed_rx = NULL;
    result.error = "ABORTED";
    if (run.state == SEQUENCE_RUNNING)
        run.state = SEQUENCE_ABORTED;
    assert(atomic_load(&control.acknowledgment) != atomic_load(&control.request));
}

static void acquire(void) {
    assert(executor_acquire());
    atomic_store(&host_owned, true);
}

static void release(void) {
    atomic_store(&host_owned, false);
    executor_release();
}

static void stage(bool sequence, bool transfer, bool carrier) {
    assert_owner();
    assert(!bus_busy && !dma_active && !pio_active && !controller_active);
    run = (sequence_status_t) {.state = sequence ? SEQUENCE_RUNNING : SEQUENCE_IDLE,
                               .run_start = model_time};
    pwm = (timing_pwm_t) {.configured = true, .active = carrier, .pin = 7,
                         .hz = 1000000, .period_cycles = 125,
                         .final_mode = TIMING_PWM_CARRIER};
    result = (i2c_bus_result_t) {0};
    bus_busy = controller_active = transfer;
    dma_active = pio_active = carrier;
    borrowed_payload = transfer ? payload : NULL;
    borrowed_rx = transfer ? rx : NULL;
    memcpy(expected_payload, payload, sizeof(payload));
    atomic_store(&complete_transfer, false);
    atomic_store(&inject_pwm_fault, false);
}

static void check_snapshot(const executor_status_t *snapshot) {
    uint32_t generation = snapshot->sequence.poll_attempts;
    assert(snapshot->sequence.state == SEQUENCE_RUNNING);
    assert(snapshot->sequence.run_elapsed == generation);
    assert(snapshot->sequence.poll_length == sizeof(run.poll_data));
    for (size_t i = 0; i < snapshot->sequence.poll_length; ++i)
        assert(snapshot->sequence.poll_data[i] == (uint8_t) generation);
    uint64_t cycles = ((uint64_t) generation << 32) | generation;
    assert(snapshot->sequence.programmed_cycles == cycles);
    assert(snapshot->sequence.inserted_cycles == ~cycles);
    assert(snapshot->pwm.active && snapshot->pwm.final_duty_basis_points == generation % 10001u);
}

/* Hold the end of a completed service pass until core0 publishes its request.
 * The next worker iteration must service STATUS, but not ABORT, after progress. */
static uint32_t pause_service(void) {
    struct timespec until = deadline();
    assert(!pthread_mutex_lock(&progress_lock));
    gate_service = true;
    finish_service = false;
    while (!service_entered)
        assert(!pthread_cond_timedwait(&progress_changed, &progress_lock, &until));
    uint32_t generation = run.poll_attempts;
    assert(!pthread_mutex_unlock(&progress_lock));
    return generation;
}

static void *teardown_observer(void *unused) {
    (void) unused;
    struct timespec until = deadline();
    assert(!pthread_mutex_lock(&progress_lock));
    while (!abort_entered)
        assert(!pthread_cond_timedwait(&progress_changed, &progress_lock, &until));
    assert(!dma_active && pio_active && controller_active);
    assert(!atomic_load(&abort_returned));
    assert(atomic_load(&control.acknowledgment) != atomic_load(&control.request));
    finish_abort = true;
    assert(!pthread_cond_broadcast(&progress_changed));
    assert(!pthread_mutex_unlock(&progress_lock));
    return NULL;
}

int main(void) {
    executor_init(rx);
    acquire();
    uint64_t ticks = progress(false);
    wait_progress(true, progress(true) + 100);
    assert(progress(false) == ticks); /* Worker runs but never services while parked. */
    stage(true, true, true);
    release();
    assert(executor_busy());
    assert(!executor_acquire()); /* Both sequence and transfer retain buffer ownership. */

    /* Core0 performs no service calls or mailbox requests during this barrier. */
    wait_progress(false, progress(false) + 200);
    executor_status_t snapshot;
    executor_status(&snapshot);
    check_snapshot(&snapshot);
    uint32_t first_generation = snapshot.sequence.poll_attempts;
    wait_progress(false, progress(false) + 100);
    check_snapshot(&snapshot); /* Published copies are not live execution pointers. */
    assert(snapshot.sequence.poll_attempts == first_generation);
    for (unsigned i = 0; i < 1000; ++i) {
        executor_status(&snapshot);
        check_snapshot(&snapshot);
    }
    assert(snapshot.sequence.poll_attempts > first_generation);
    uint32_t before_status = pause_service();
    executor_status(&snapshot);
    check_snapshot(&snapshot);
    assert(snapshot.sequence.poll_attempts == before_status + 1);

    pthread_t observer;
    assert(!pthread_mutex_lock(&progress_lock));
    gate_abort = true;
    assert(!pthread_mutex_unlock(&progress_lock));
    assert(!pthread_create(&observer, NULL, teardown_observer, NULL));
    uint32_t before_abort = pause_service();
    executor_abort();
    atomic_store(&abort_returned, true);
    assert(!pthread_join(observer, NULL));
    assert(!executor_busy());
    acquire();
    assert(run.state == SEQUENCE_ABORTED);
    assert(run.poll_attempts == before_abort); /* ABORT precedes the next refill pass. */
    assert(!dma_active && !pio_active && !controller_active && !bus_busy);
    memset(rx, 0xe7, sizeof(rx));
    memset(payload, 0x5a, sizeof(payload));
    release();
    wait_progress(false, progress(false) + 100);
    acquire();
    for (size_t i = 0; i < sizeof(rx); ++i)
        assert(rx[i] == 0xe7); /* A canceled worker cannot touch reused buffers. */
    stage(false, true, false);
    release();
    assert(executor_busy() && !executor_acquire()); /* Direct transfer alone is busy. */
    atomic_store(&complete_transfer, true);
    wait_progress(false, progress(false) + 100);
    assert(!executor_busy());
    acquire();
    assert(!i2c_bus_result()->error && i2c_bus_result()->written == sizeof(payload));
    uint8_t expected_read = payload[0] ^ 0xffu;
    assert(i2c_bus_result()->read == 1 && rx[0] == expected_read);
    stage(false, true, false);
    release();
    executor_abort(); /* Direct cancellation needs teardown even without a sequence. */
    acquire();
    assert(!bus_busy && !controller_active && !borrowed_rx && !borrowed_payload);
    assert(!strcmp(i2c_bus_result()->error, "ABORTED"));
    stage(false, false, true);
    release();
    assert(!executor_busy()); /* A held hardware carrier does not own staging. */
    atomic_store(&inject_pwm_fault, true);
    wait_progress(false, progress(false) + 100);
    executor_status(&snapshot);
    assert(snapshot.sequence.state == SEQUENCE_ERROR && !snapshot.pwm.active);
    assert(!strcmp(snapshot.sequence.run_error, "TIMING_ERROR"));
    acquire();
    assert(!dma_active && !pio_active);
    atomic_store(&shutdown_worker, true);
    assert(!pthread_join(worker, NULL));
    puts("executor threads: parked ownership, priority service, coherent snapshots, prompt safe cancellation and idle PWM faults passed");
    return 0;
}
