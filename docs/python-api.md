# Python API

See [environment setup](building.md#python-environment) and the [glitch notebook guide](notebook.md).

`Device(port, *, timeout=2.0)` opens CDC and checks protocol 3. Use it as a context manager; context exit aborts an active synchronized run and closes the connection. `close()` drops DTR and closes the port, restoring safe GPIO/PWM states even after successful completion.

```python
from rp2040_glitch import Device, Pulse, WriteAt, seconds_to_cycles

# All fixture values must be supplied and electrically verified by the caller.
with Device(port) as device:
    device.configure_bus(bus, sda=sda_pin, scl=scl_pin, hz=1_000_000)
    device.configure_gpio(pin=reset_pin, drive="open_drain", idle=1)
    clock_hz = device.info()["sys_hz"]
    device.run([
        Pulse(reset_pin, seconds_to_cycles(reset_seconds, clock_hz=clock_hz)),
        WriteAt(bus, address, lower_payload,
                seconds_to_cycles(lower_offset_seconds, clock_hz=clock_hz)),
        WriteAt(bus, address, restore_payload,
                seconds_to_cycles(restore_offset_seconds, clock_hz=clock_hz)),
    ])
```

See [scheduled-write timing](timing.md#scheduled-i2c-writes). Both write offsets in this example are relative to reset release. They are not voltage-settling timestamps, and the second offset is not a delay after the first write.

## Device methods

```text
Device(port, *, timeout=2.0)
close()
info()
configure_bus(bus, *, sda, scl, hz=100000, pullups=False)
configure_gpio(*, pin, drive="push_pull", idle=0)
configure_sync(*, pin)
configure_spi(*, cs, sck, data, mode=0, lsb_first=False,
              cs_active_low=True, max_hz=10000000)
configure_pwm(*, pin, hz=500000)
transfer(bus, address, *, write=b'', read=0, timeout_us=10000)
write_register(bus, address, register: bytes, data: bytes, *, timeout_us=10000)
read_register(bus, address, register: bytes, length=1, *, timeout_us=10000)
start(steps, *, timeout_us=1000000, hold_final=True)
status()
wait(*, timeout=None, poll_interval=0.01)
run(steps, *, timeout_us=1000000, hold_final=True)
set_pin(pin, state, *, hold_final=True)
abort()
bootloader()
```

`transfer` and `read_register` return bytes. Write-then-read uses a repeated START and ends with STOP. Register pointers are explicit bytes; no endian conversion or PEC is implicit. `set_pin` requires an already configured GPIO or PWM pin.

`seconds_to_cycles(seconds, *, clock_hz)` rounds to the nearest integer, ties to even. It rejects negative/nonfinite times, nonpositive frequencies and counts above u32. Supply a measured clock when calibration matters.

`microseconds_to_cycles(microseconds, *, clock_hz)` uses floor division of `microseconds * clock_hz` by 1,000,000 before conversion to an integer cycle count. It validates a positive finite clock, nonnegative finite converted time and u32 result. It is not equivalent to the nearest/ties-even seconds helper; preserve that distinction when choosing units.

## Sequence steps

Immutable dataclasses, with positional order shown:

```text
Delay(cycles)
Pulse(pin, width_cycles)
SetPin(pin, state)
Pwm(duty_percent)
Write(bus, address, data, timeout_us=10000)
WriteAt(bus, address, data, at_cycles, timeout_us=10000)
SyncWrites()
Poll(bus, address, register, length, mask, value,
     interval_us=1000, timeout_us=1000000, transfer_timeout_us=10000)
WaitI2C(bus, address, read, prefix=b'', timeout_us=1000000)
WaitSPI(prefix, timeout_us=1000000)
WaitGPIO(pin, rising=True, timeout_us=1000000)
```

`SetPin` accepts `"low"`, `"high"` or `"floating"`. Low/high respect drive mode; floating disables output with no internal pulls. GPIO `idle` is integer 0/1, not boolean. `Pulse` asserts the opposite of configured idle, then restores that idle rather than a preceding `SetPin` state. Other pins retain their held states.

By default, successful runs hold final GPIO states and PWM mode/duty. `hold_final=False` restores configured GPIO idle and PWM low. Abort, run error, disconnect/DTR loss and BOOTSEL always restore those safe states. `clear` resets staging/results without changing held outputs; `abort()` explicitly clears holds.

`Poll` repeats a pointer-write/read and compares every byte using `(received & mask) == (value & mask)`. Its interval is start-to-start without catch-up bursts. With `interval_us=0`, the next attempt starts after the previous attempt completes. NACK means not-ready only for polling. Arbitration loss and bus timeout terminate the run. Ordinary writes never retry automatically or force bus-recovery clocks.

## Limits and errors

See the canonical [resource limits](protocol.md#resource-limits) and [error codes](protocol.md#error-codes). Invalid local arguments raise `ValueError`; no staging begins until local sequence validation succeeds.

I2C addresses are 0x08–0x77. Bytes remain bytes until wire encoding. The host validates the complete sequence before staging; firmware performs authoritative validation and compiles waveform blocks before accepting `start`. Failed add leaves earlier staging intact; failed host staging never sends start.

`DeviceError` exposes `code`, `step` and `detail`; terminal `wait`/`run` failures raise it. `ProtocolError` covers framing, version, ID and malformed-result failures; `TransportError` covers I/O/deadlines. Transport/protocol failure closes the port and drops DTR: reopen explicitly, never automatically replay a mutation. KeyboardInterrupt cancels when synchronized. Host `wait(timeout=...)` cancels before raising `TimeoutError`. `BootloaderUncertainError` means acknowledgment was not received, not that ROM enumeration was observed.
