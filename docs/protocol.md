# Wire protocol 3

Application release **1.0** and USB device release **1.00** are distinct from wire protocol **3**. The protocol version remains 3.


Requests are `<nonzero-u32-id> <command> [arguments]\n`, optionally CRLF. Arguments use decimal unsigned integers, booleans 0/1, lowercase even hex and `-` for empty bytes. Maximum request size is 4096 bytes including newline. Overflow discards through newline and reports `BAD_REQUEST` without execution. Replies are one JSON line of at most 32768 bytes:

```json
{"id":1,"ok":true,"result":{}}
{"id":2,"ok":false,"error":{"code":"NACK","step":null,"detail":1}}
```

One request may be outstanding; there is no unsolicited output. During a run only `info`, `status`, `abort` and `bootloader` are accepted. Other commands return `BUSY`. The host rejects incompatible protocol versions before configuration or staging.

```text
info
bus_config bus sda scl hz pullups
gpio_config pin drive idle
sync_config pin
spi_config cs sck data mode lsb_first cs_active_low max_hz
pwm_config pin hz
transfer bus address timeout_us write_hex read_length
clear
add delay cycles
add pulse pin width_cycles
add set pin low|high|floating
add pwm duty_basis_points
add write bus address timeout_us data_hex
add write_at bus address at_cycles timeout_us data_hex
add sync_writes
add poll bus address register_hex read_length mask_hex value_hex interval_us timeout_us transfer_timeout_us
add wait_i2c bus address read prefix_hex timeout_us
add wait_spi prefix_hex timeout_us
add wait_gpio pin rising timeout_us
start run_timeout_us [hold_final]
status
abort
bootloader
```

The optional `hold_final` flag is 0/1 and defaults to 1; Python sends it explicitly. Transfer results contain `{data,written,read}`; configuration/staging returns `{}`; start returns `{"state":"running"}`; abort returns status; bootloader returns `{"rebooting":true}`.

## Error codes

Error codes: `BAD_REQUEST`, `RANGE`, `PIN_CONFLICT`, `UNCONFIGURED`, `BUSY`, `RESOURCE_BUSY`, `NACK`, `ARBITRATION_LOST`, `BUS_TIMEOUT`, `WAIT_TIMEOUT`, `RUN_TIMEOUT`, `MISSED_DEADLINE`, `TRIGGER_OVERFLOW`, `TIMING_ERROR`, `ABORTED`, `CLOCK_ERROR`. Applicable `detail` contains the raw I2C abort mask. A rejected command is distinct from a retained terminal run error.


## Resource limits

| Resource | Limit |
|---|---|
| Request / reply | 4096 / 32768 bytes including newline |
| I2C buses / addresses | Two buses (0/1); addresses 0x08–0x77 |
| Sequence | 64 steps; 8192 staged payload bytes |
| Direct transfer | 1024 write bytes and 1024 read bytes; at least one byte total |
| Routine Write/Poll fields | 256 bytes; Poll reads 1–256 bytes with equal mask/value lengths |
| Scheduled writes | Two per group; 1–16 payload bytes each |
| Hardware waits | One per run |
| Deadlines | Positive, at most 60 seconds |
| Waveform storage | 1024 u32 words; three words per frame |
| PWM storage | 64 segments; 1024 words |

Hardware-wait prefixes contain zero–two bytes for I2C and one–four bytes for SPI. Scheduled offsets are u32 cycles, at least eight, with four-cycle readiness lead time. GPIOs are 0–29; see [safety](safety.md) for mux and collision constraints and [timing](timing.md) for minimum pulse/spacing, PWM alignment and trigger constraints.

`info` reports protocol, configured `sys_hz`, capabilities/limits and current configuration. `status` and abort results describe the coherent worker state (`idle`, `running`, `done`, `error`, `aborted`), step, elapsed time, polling, match/error details and compiled GPIO/PWM metadata; see [status and cancellation](timing.md#status-and-cancellation). Terminal run errors are retained status data, not a failed status command. `CLOCK_ERROR` gates non-control commands, not `info`, `status`, `abort` or `bootloader`.
