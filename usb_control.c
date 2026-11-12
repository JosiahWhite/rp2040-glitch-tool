/* SPDX-License-Identifier: MIT */
/* Core-0 USB/session control. Staging requires acknowledged worker ownership;
 * core 1 owns execution and releases buffers only at executor boundaries. */
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "executor.h"
#include "i2c_bus.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "protocol.h"
#include "sequence.h"
#include "timing.h"
#include "tusb.h"
#include "usb_control.h"

static uint8_t              scratch[I2C_TRANSFER_LIMIT], rx[I2C_TRANSFER_LIMIT];
static request_t            request;
static response_t           response;
static bool                 response_ready, disconnected, clock_ok;
static bool                 direct_pending, rebooting;
static bool                 urgent_handled;
static uint32_t             pending_id;
static uint64_t             reboot_deadline;
static bool                 deadline_valid(uint32_t v) {
    return v && v <= I2C_DEADLINE_LIMIT_US;
}
static bool address_valid(uint32_t v) {
    return v >= 8 && v <= 0x77;
}
static void fail(const char *error, int step, uint32_t detail) {
    response_error(&response, request.id, error, step, detail);
    response_ready = true;
}
static void success_empty(void) {
    response_begin(&response, request.id);
    response_append(&response, "{}");
    response_end(&response);
    response_ready = true;
}
static void pwm_result(const timing_pwm_t *p) {
    if (!p->configured) {
        response_append(&response, "null");
        return;
    }
    uint32_t    high = ((uint64_t) p->period_cycles * p->final_duty_basis_points + 5000u) / 10000u;
    const char *mode = p->final_mode == TIMING_PWM_CARRIER    ? "carrier"
                       : p->final_mode == TIMING_PWM_HIGH     ? "high"
                       : p->final_mode == TIMING_PWM_FLOATING ? "floating"
                                                              : "low";
    response_append(&response,
                    "{\"pin\":%u,\"hz\":%lu,\"period_cycles\":%lu,\"active\":%s,\"final_duty_basis_"
                    "points\":%lu,\"final_high_cycles\":%lu,\"final_mode\":\"%s\"}",
                    p->pin, (unsigned long) p->hz, (unsigned long) p->period_cycles,
                    p->active ? "true" : "false", (unsigned long) p->final_duty_basis_points,
                    (unsigned long) high, mode);
}
static void status_result(void) {
    executor_status_t snapshot;
    executor_status(&snapshot);
    const sequence_status_t *status = &snapshot.sequence;
    response_append(
        &response, "{\"state\":\"%s\",\"step\":%u,\"elapsed_us\":%llu,\"poll\":{\"data\":\"",
        sequence_state_name(status->state), status->current_step,
        (unsigned long long) status->run_elapsed);
    response_hex(&response, status->poll_data, status->poll_length);
    response_append(&response,
                    "\",\"attempts\":%lu,\"last_error\":", (unsigned long) status->poll_attempts);
    if (status->last_poll_error)
        response_append(&response, "\"%s\"", status->last_poll_error);
    else
        response_append(&response, "null");
    response_append(&response, "},\"matched\":");
    const timing_trigger_t *trigger = &status->matched_trigger;
    if (!status->matched)
        response_append(&response, "null");
    else if (trigger->kind == TIMING_TRIGGER_GPIO) {
        response_append(&response, "{\"source\":\"gpio\",\"pin\":%u,\"rising\":%s}", trigger->pin,
                        trigger->rising ? "true" : "false");
    } else {
        response_append(&response, "{\"source\":\"%s\",\"address\":%u,\"read\":%s,\"prefix\":\"",
                        trigger->kind == TIMING_TRIGGER_SPI ? "spi" : "i2c", trigger->address,
                        trigger->read ? "true" : "false");
        response_hex(&response, trigger->prefix, trigger->prefix_length);
        response_append(&response, "\"}");
    }
    response_append(&response, ",\"error\":");
    if (status->run_error) {
        response_append(&response, "{\"code\":\"%s\",\"step\":%u,\"detail\":%lu}",
                        status->run_error, status->current_step,
                        (unsigned long) status->error_detail);
    } else {
        response_append(&response, "null");
    }
    response_append(&response,
                    ",\"programmed_cycles\":%llu,\"inserted_cycles\":%llu,\"hold_final\":%s,"
                    "\"token_count\":%u,\"pwm_word_count\":%u,\"pwm\":",
                    (unsigned long long) status->programmed_cycles,
                    (unsigned long long) status->inserted_cycles,
                    status->hold_final ? "true" : "false", status->compiled_tokens,
                    status->compiled_pwm_words);
    pwm_result(&snapshot.pwm);
    response_append(&response, "}");
}
static void status_reply(void) {
    response_begin(&response, request.id);
    status_result();
    response_end(&response);
    response_ready = true;
}

/* Build collision masks only from explicit, successfully installed configs. */
static uint32_t pins_except(int bus, int gpio, bool sync, bool spi, bool pwm) {
    uint32_t mask = 0;
    for (unsigned i = 0; i < I2C_BUS_COUNT; ++i) {
        const i2c_bus_config_t *c = i2c_bus_config(i);
        if ((int) i != bus && c->configured)
            mask |= (1u << c->sda) | (1u << c->scl);
    }
    uint32_t gpio_mask = timing_gpio()->configured_mask;
    if (gpio >= 0)
        gpio_mask &= ~(1u << gpio);
    mask |= gpio_mask;
    int sync_pin = timing_sync_pin();
    if (!sync && sync_pin >= 0)
        mask |= 1u << sync_pin;
    const timing_spi_t *s = timing_spi();
    if (!spi && s->configured)
        mask |= (1u << s->cs) | (1u << s->sck) | (1u << s->data);
    const timing_pwm_t *p = timing_pwm();
    if (!pwm && p->configured)
        mask |= 1u << p->pin;
    return mask;
}
static const char *configure_gpio(void) {
    uint32_t pin, idle;
    if (request.argc != 5 || !request_u32(&request, 2, &pin) || !request_u32(&request, 4, &idle))
        return "BAD_REQUEST";
    if (pin > 29 || idle > 1)
        return "RANGE";

    timing_drive_t drive;
    if (!strcmp(request.argv[3], "push_pull"))
        drive = TIMING_PUSH_PULL;
    else if (!strcmp(request.argv[3], "open_drain"))
        drive = TIMING_OPEN_DRAIN;
    else if (!strcmp(request.argv[3], "open_source"))
        drive = TIMING_OPEN_SOURCE;
    else
        return "RANGE";
    if (pins_except(-1, (int) pin, false, false, false) & (1u << pin))
        return "PIN_CONFLICT";
    return timing_configure_gpio(pin, drive, idle);
}

static const char *configure_bus(void) {
    uint32_t bus, sda, scl, hz, pullups;
    if (request.argc != 7 || !request_u32(&request, 2, &bus) || !request_u32(&request, 3, &sda) ||
        !request_u32(&request, 4, &scl) || !request_u32(&request, 5, &hz) ||
        !request_u32(&request, 6, &pullups)) {
        return "BAD_REQUEST";
    }
    const char *error = i2c_bus_validate(bus, sda, scl, hz);
    if (error)
        return error;
    if (pullups > 1)
        return "RANGE";
    if (pins_except((int) bus, -1, false, false, false) & ((1u << sda) | (1u << scl))) {
        return "PIN_CONFLICT";
    }
    return i2c_bus_configure(bus, sda, scl, hz, pullups);
}

static const char *configure_pwm(void) {
    uint32_t pin, hz;
    if (request.argc != 4 || !request_u32(&request, 2, &pin) || !request_u32(&request, 3, &hz))
        return "BAD_REQUEST";
    if (pin > 29)
        return "RANGE";
    if (pins_except(-1, -1, false, false, true) & (1u << pin))
        return "PIN_CONFLICT";
    return timing_configure_pwm(pin, hz);
}

static const char *configure_sync(void) {
    uint32_t pin;
    if (request.argc != 3 || !request_u32(&request, 2, &pin))
        return "BAD_REQUEST";
    if (pin > 29)
        return "RANGE";
    if (pins_except(-1, -1, true, false, false) & (1u << pin))
        return "PIN_CONFLICT";
    return timing_configure_sync(pin);
}

static const char *configure_spi(void) {
    uint32_t cs, sck, data, mode, lsb_first, cs_active_low, max_hz;
    if (request.argc != 9 || !request_u32(&request, 2, &cs) || !request_u32(&request, 3, &sck) ||
        !request_u32(&request, 4, &data) || !request_u32(&request, 5, &mode) ||
        !request_u32(&request, 6, &lsb_first) || !request_u32(&request, 7, &cs_active_low) ||
        !request_u32(&request, 8, &max_hz)) {
        return "BAD_REQUEST";
    }
    if (cs > 29 || sck > 29 || data > 29 || mode > 3 || lsb_first > 1 || cs_active_low > 1 ||
        !max_hz || max_hz > 10000000)
        return "RANGE";
    if (cs == sck || cs == data || sck == data)
        return "PIN_CONFLICT";
    if (pins_except(-1, -1, false, true, false) & ((1u << cs) | (1u << sck) | (1u << data))) {
        return "PIN_CONFLICT";
    }
    timing_spi_t config = {.configured    = true,
                           .cs            = cs,
                           .sck           = sck,
                           .data          = data,
                           .mode          = mode,
                           .lsb_first     = lsb_first,
                           .cs_active_low = cs_active_low,
                           .max_hz        = max_hz};
    return timing_configure_spi(&config);
}

static void info_reply(void) {
    executor_status_t snapshot;
    executor_status(&snapshot);
    response_begin(&response, request.id);
    response_append(&response,
                    "{\"protocol\":3,\"product\":\"RP2040 Glitch "
                    "Tool\",\"sys_hz\":%lu,\"requested_clock_khz\":%lu,\"clock_error\":%s,\"clock_"
                    "source\":\"SDK configured, not measured\",",
                    (unsigned long) clock_get_hz(clk_sys), (unsigned long) GLITCH_SYS_CLOCK_KHZ,
                    clock_ok ? "null" : "\"CLOCK_ERROR\"");
    response_append(
        &response,
        "\"limits\":{\"request_bytes\":%u,\"reply_bytes\":%u,\"steps\":%u,\"transfer_bytes\":"
        "%u,\"routine_bytes\":%u,\"payload_bytes\":%u,\"deadline_us\":"
        "%u,\"tokens\":%u,\"pwm_words\":%u,\"scheduled_writes\":%u,"
        "\"scheduled_write_bytes\":%u},",
        (unsigned) REQUEST_LIMIT, (unsigned) RESPONSE_LIMIT, (unsigned) SEQUENCE_STEP_LIMIT,
        (unsigned) I2C_TRANSFER_LIMIT, (unsigned) SEQUENCE_ROUTINE_BYTES,
        (unsigned) SEQUENCE_PAYLOAD_LIMIT, (unsigned) I2C_DEADLINE_LIMIT_US,
        (unsigned) TIMING_MAX_TOKENS, (unsigned) TIMING_MAX_PWM_WORDS,
        (unsigned) TIMING_WRITE_LIMIT, (unsigned) I2C_PREPARED_LIMIT);
    response_append(
        &response,
        "\"min_interval_cycles\":%u,\"i2c_trigger_latency_cycles\":%u,\"spi_trigger_latency_"
        "cycles\":%u,\"sampling_uncertainty_cycles\":%u,\"spi_cs_setup_cycles\":%u,\"spi_cs_hold_"
        "cycles\":%u,\"spi_cs_gap_cycles\":%u,\"spi_half_period_ns\":50,",
        TIMING_MIN_INTERVAL_CYCLES, TIMING_I2C_MATCH_LATENCY_CYCLES,
        TIMING_SPI_MATCH_LATENCY_CYCLES, TIMING_INPUT_UNCERTAINTY_CYCLES,
        TIMING_SPI_CS_SETUP_CYCLES, TIMING_SPI_CS_HOLD_CYCLES, TIMING_SPI_CS_GAP_CYCLES);
    response_append(&response,
                    "\"gpio_trigger_latency_cycles\":%u,\"gpio_trigger_min_level_cycles\":4,",
                    TIMING_GPIO_MATCH_LATENCY_CYCLES);
    response_append(&response,
                    "\"write_at_min_cycles\":%u,\"write_at_ready_lead_cycles\":%u,",
                    TIMING_WRITE_MIN_CYCLES, TIMING_WRITE_READY_LEAD);
    response_append(
        &response,
        "\"open_drive_latency_extra_cycles\":1,\"implicit_wait_spacing_cycles\":%u,\"trigger_"
        "latency_reference\":\"synchronized final clock or GPIO edge to push-pull OUT; add "
        "effective post-wait spacing (minimum eight) and one cycle for direction edges\",",
        TIMING_MIN_INTERVAL_CYCLES);
    response_append(&response, "\"buses\":[");
    for (unsigned b = 0; b < I2C_BUS_COUNT; ++b) {
        const i2c_bus_config_t *c = i2c_bus_config(b);
        if (b)
            response_append(&response, ",");
        if (!c->configured)
            response_append(&response, "null");
        else
            response_append(&response,
                            "{\"bus\":%u,\"sda\":%u,\"scl\":%u,\"hz\":%lu,\"pullups\":%s}", b,
                            c->sda, c->scl, (unsigned long) c->hz, c->pullups ? "true" : "false");
    }
    response_append(&response, "],\"gpio\":[");
    const timing_gpio_t *g         = timing_gpio();
    bool                 separator = false;
    for (unsigned pin = 0; pin < 30; ++pin) {
        uint32_t bit = 1u << pin;
        if (!(g->configured_mask & bit))
            continue;
        const char *drive = g->open_drain_mask & bit    ? "open_drain"
                            : g->open_source_mask & bit ? "open_source"
                                                        : "push_pull";
        response_append(&response, "%s{\"pin\":%u,\"drive\":\"%s\",\"idle\":%u}",
                        separator ? "," : "", pin, drive, !!(g->idle_high_mask & bit));
        separator = true;
    }
    response_append(&response, "],\"sync_pin\":");
    if (timing_sync_pin() < 0)
        response_append(&response, "null");
    else
        response_append(&response, "%d", timing_sync_pin());
    response_append(&response, ",\"spi\":");
    const timing_spi_t *s = timing_spi();
    if (!s->configured)
        response_append(&response, "null");
    else
        response_append(&response,
                        "{\"cs\":%u,\"sck\":%u,\"data\":%u,\"mode\":%u,\"lsb_first\":%s,\"cs_"
                        "active_low\":%s,\"max_hz\":%lu}",
                        s->cs, s->sck, s->data, s->mode, s->lsb_first ? "true" : "false",
                        s->cs_active_low ? "true" : "false", (unsigned long) s->max_hz);
    response_append(&response, ",\"pwm\":");
    pwm_result(&snapshot.pwm);
    response_append(&response,
                    ",\"trigger_arming\":\"live from block start, including preceding pulses\"}");
    response_end(&response);
    response_ready = true;
}
/* Every path through this helper runs with the worker acknowledged parked.
 * The caller releases ownership, including on validation errors/early returns. */
static void dispatch_parked(void) {
    const char *cmd = request.argv[1], *error = NULL;
    if (!strcmp(cmd, "clear")) {
        if (request.argc != 2)
            error = "BAD_REQUEST";
        else
            sequence_clear();
    } else if (!strcmp(cmd, "bus_config"))
        error = configure_bus();
    else if (!strcmp(cmd, "gpio_config"))
        error = configure_gpio();
    else if (!strcmp(cmd, "sync_config"))
        error = configure_sync();
    else if (!strcmp(cmd, "spi_config"))
        error = configure_spi();
    else if (!strcmp(cmd, "pwm_config"))
        error = configure_pwm();
    else if (!strcmp(cmd, "add"))
        error = sequence_stage(&request, scratch);
    else if (!strcmp(cmd, "start")) {
        uint32_t timeout, requested_hold = 1;
        if ((request.argc != 3 && request.argc != 4) || !request_u32(&request, 2, &timeout) ||
            (request.argc == 4 && !request_u32(&request, 3, &requested_hold)))
            error = "BAD_REQUEST";
        else if (!deadline_valid(timeout) || requested_hold > 1)
            error = "RANGE";
        else if (!(error = sequence_start(timeout, requested_hold,
                                          pins_except(-1, -1, false, false, false)))) {
            response_begin(&response, request.id);
            response_append(&response, "{\"state\":\"running\"}");
            response_end(&response);
            response_ready = true;
            return;
        }
    } else if (!strcmp(cmd, "transfer")) {
        uint32_t bus, address, timeout, nread;
        size_t   nwrite;
        if (request.argc != 7 || !request_u32(&request, 2, &bus) ||
            !request_u32(&request, 3, &address) || !request_u32(&request, 4, &timeout) ||
            !request_u32(&request, 6, &nread))
            error = "BAD_REQUEST";
        else if (bus >= I2C_BUS_COUNT || !address_valid(address) || !deadline_valid(timeout) ||
                 nread > I2C_TRANSFER_LIMIT ||
                 !parse_hex(request.argv[5], scratch, sizeof(scratch), &nwrite) ||
                 (!nwrite && !nread))
            error = "RANGE";
        else if (!(error = i2c_bus_begin(bus, address, scratch, nwrite, rx, nread,
                                         time_us_64() + timeout))) {
            direct_pending = true;
            pending_id     = request.id;
            return;
        }
    } else
        error = "BAD_REQUEST";
    if (error)
        fail(error, -1, 0);
    else
        success_empty();
}

static void dispatch(void) {
    if (request.error) {
        fail(request.error, -1, 0);
        return;
    }
    const char *cmd = request.argv[1];
    bool control = !strcmp(cmd, "info") || !strcmp(cmd, "status") ||
                   !strcmp(cmd, "abort") || !strcmp(cmd, "bootloader");
    if (!control) {
        if (!executor_acquire()) {
            fail("BUSY", -1, 0);
            return;
        }
        if (!clock_ok)
            fail("CLOCK_ERROR", -1, 0);
        else
            dispatch_parked();
        executor_release();
        return;
    }
    if (request.argc != 2) {
        fail("BAD_REQUEST", -1, 0);
        return;
    }
    if (!strcmp(cmd, "info")) {
        info_reply();
    } else if (!strcmp(cmd, "status")) {
        status_reply();
    } else if (!strcmp(cmd, "abort")) {
        executor_abort();
        status_reply();
    } else {
        executor_abort();
        rebooting = true;
        if (!reboot_deadline)
            reboot_deadline = time_us_64() + 100000;
        response_begin(&response, request.id);
        response_append(&response, "{\"rebooting\":true}");
        response_end(&response);
        response_ready = true;
    }
}

static void service_requests(void) {
    if (rebooting)
        return;
    for (unsigned budget = 0; !request.ready && budget < 256 && tud_cdc_available(); ++budget) {
        unsigned char c;
        if (tud_cdc_read(&c, 1) != 1)
            break;
        request_feed(&request, c);
    }
    if (!request.ready)
        return;
    /* Cancellation must not wait for the host to drain an earlier reply. Keep
     * that reply intact and defer only the cancellation acknowledgment. */
    if (!request.error && request.argc == 2 && !urgent_handled &&
        (!strcmp(request.argv[1], "abort") || !strcmp(request.argv[1], "bootloader"))) {
        executor_abort();
        urgent_handled = true;
        if (!strcmp(request.argv[1], "bootloader")) {
            reboot_deadline = time_us_64() + 100000;
        }
    }
    if (!response_ready && !direct_pending) {
        dispatch();
        request_reset(&request);
        urgent_handled = false;
    }
}

static void service_disconnect(void) {
    if (!disconnected)
        return;
    disconnected = false;
    executor_abort();
    direct_pending = response_ready = urgent_handled = false;
    response.length = response.sent = 0;
    request_reset(&request);
    tud_cdc_read_flush();
}

static void service_pending_replies(void) {
    if (direct_pending && executor_acquire()) {
        const i2c_bus_result_t *result = i2c_bus_result();
        if (result->error) {
            response_error(&response, pending_id, result->error, -1, result->raw_abort);
        } else {
            response_begin(&response, pending_id);
            response_append(&response, "{\"data\":\"");
            response_hex(&response, rx, result->read);
            response_append(&response, "\",\"written\":%u,\"read\":%u}", (unsigned) result->written,
                            (unsigned) result->read);
            response_end(&response);
        }
        executor_release();
        direct_pending = false;
        response_ready = true;
    }
}

static void service_response(void) {
    if (!response_ready)
        return;
    if (response.failed)
        response_error(&response, request.id, "RANGE", -1, 0);
    size_t remaining = response.length - response.sent;
    if (remaining > 256)
        remaining = 256;
    response.sent += tud_cdc_write(response.data + response.sent, remaining);
    tud_cdc_write_flush();
    if (response.sent == response.length)
        response_ready = false;
}

void tud_mount_cb(void) {}
void tud_umount_cb(void) {
    disconnected = true;
}
void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
}
void tud_resume_cb(void) {}
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void) itf;
    (void) rts;
    if (!dtr)
        disconnected = true;
}

/* One-time core-0 initialization. rx has static lifetime: the worker retains
 * its address for direct transfers and sequence reads throughout the session. */
void usb_control_init(bool clock_valid) {
    clock_ok = clock_valid;
    sequence_clear();
    executor_init(rx);
    request_reset(&request);
}

void usb_control_task(void) {
    service_disconnect();
    service_pending_replies();
    service_response();
    service_requests();
    if (reboot_deadline && time_us_64() >= reboot_deadline) {
        executor_abort();
        reset_usb_boot(0, 0);
    }
}
