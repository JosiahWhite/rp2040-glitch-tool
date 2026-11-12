# Execution and timing

See [ownership and execution](development.md#ownership-and-execution) for core-0 preparation, immutable run buffers and core-1 worker control. **Status polling does not advance the program:** execution continues independently of USB response formatting. Block startup uses bounded register/FIFO setup without a PWM-prefill spin; CPU setup and peripheral readiness still contribute runtime latency. Cancellation is acknowledged only after event producers, DMA and controller use are torn down.

Native writes fill available FIFO space in batches. Consecutive ordinary writes advance in the same servicing pass that observes completion. Same-target FIFO preparation does not wait for another USB/main-loop pass. Different targets, peripheral reset/disable acknowledgment and bus conditions can require additional worker polls. Completion requires confirmed data, STOP and local master idle; queued bytes are not counted as acknowledged. This is not a guarantee of zero STOP-to-START time.

### GPIO waveforms

Pulse holds and delays use u32 system cycles. Pulse widths and positive delays below eight cycles are rejected; zero delay is removed. The compiler supplies missing eight-cycle spacing between output transitions, including after hardware waits. Explicit delays count toward this minimum.

Adjacent timed operations compile into preloaded PIO/DMA blocks. `programmed_cycles` includes requested holds/delays and inserted spacing, but excludes variable waits, I2C wire time and CPU preparation. `inserted_cycles` reports padding; `token_count` reports words, not frames. Initial host-command launch and CPU-observed I2C completion to a following timing block are not cycle-deterministic.

### Scheduled I2C writes

`WriteAt.at_cycles` is relative to completion of the **preceding non-WriteAt command**. Consecutive declarations share that reference and do not accumulate offsets.

| Preceding command | Reference |
|---|---|
| `Pulse` | Observed return-to-idle edge after observing the asserted level |
| `WaitGPIO`, `WaitI2C`, `WaitSPI` | Hardware match latch on the sync GPIO |
| `Delay`, `SetPin`, `Pwm` | Compiled hardware completion boundary, counted by PIO from synchronized waveform launch |
| `Write`, `Poll`, `SyncWrites` | Firmware-observed completion, including CPU service/setup latency |

- A group has one or two contiguous declarations with strictly increasing u32 offsets of at least eight cycles. `WriteAt` cannot be the first command. Join pending writes with `SyncWrites`, ordinary `Write` or `Poll` before another reference/group.
- Declarations are nonblocking: foreground delays, GPIO/PWM commands and waits can continue while writes are pending. `SyncWrites`, ordinary `Write`, `Poll` and sequence end join pending writes. Separate waveform blocks still have CPU setup overhead.
- The first write is preloaded behind `IC_ENABLE.TX_CMD_BLOCK` before the reference command starts, except after native `Write`/`Poll`, which must finish using the controller first. Those software references start before payload preparation.
- After write 1 completes, core 1 prepares write 2 in the same sequence service pass. Its offset does not move. A readiness permit must arrive four system cycles before the PIO release event; otherwise the run fails with `MISSED_DEADLINE`, without a late fallback transfer.
- `timeout_us` covers preparation, waiting for the reference, waiting for the offset and transmission. Include pulse/wait duration in the first write's budget. Validation rejects nominally impossible spacing; actual stretching, bus timing and preparation can still cause a deadline miss. Scheduled failures report the scheduled step even if foreground execution has advanced.
- The scheduled timestamp selects a PIO RX PUSH event. DMA then clears `TX_CMD_BLOCK`. Input synchronization, electrical rise time, DMA arbitration and controller launch add latency. **It is not an exact first-SCL or voltage-settling timestamp.**
- Event engines use PIO0 SM0/1, sharing ten instructions at offsets 0–9 and a two-instruction hardware-origin prelude at 16–17. GPIO waveform and PWM use SM2/3; matchers remain on PIO1. Up to two release DMA channels are reserved before execution and retained across groups until run teardown. Resource conflicts return `RESOURCE_BUSY`; teardown quiesces DMA before controller reset.

For `Delay`, `SetPin` and `Pwm` references, the timers start synchronously with the isolated reference block and count to its compiled completion boundary before counting `at_cycles`. Delayed CPU observation of block completion cannot move that reference. A generic `SetPin` reference is its output-latch/direction command, not a measured external rise; `Delay(0)` and unchanged `SetPin` still have a hardware command boundary. Pulse references continue observing the physical pin, and hardware waits continue using the matcher latch.

For a delay measured **after transmission completes**, join first:

```python
from rp2040_glitch import SyncWrites, Delay, Write

# Append after a WriteAt declaration in the staged program:
completion_relative_restore = [
    SyncWrites(),
    Delay(restore_delay_cycles),
    Write(bus, address, restore_payload),
]
```

A `Delay` directly after `WriteAt` does not wait for that write. There is no hardware `WriteAfter` command. `info()` reports scheduled-write limits, minimum offset and readiness lead cycles.

### Hardware waits

Hardware waits require a configured sync pin. Matchers are live from timing-block launch: a match during an earlier pulse in that block is retained until the wait is reached. To begin matching only after that pulse, use a separate run or an actual compiled-block boundary; do not assume the wait's position delays arming.

- **I2C:** matches address/direction and zero–two initial data bytes, including preceding ACK bits but excluding the final ACK. The reference is the final matched bit's sampled rising SCL edge, not STOP. STOP/repeated START reset partial matches through hardware boundary DMA.
- **SPI:** matches the first one–four bytes after CS assertion, modes 0–3 and either bit order. Arming during selected CS waits for a fresh deselect/select. SCK high/low must each be at least 50 ns; CS setup/hold at least eight system cycles; inactive CS gap at least 64 cycles. The supported configuration limit is 10 MHz, subject to electrical validation.
- **GPIO:** rising waits for low then high; falling waits for high then low. An already-active input does not trigger. Hold each level at least four system cycles. The input needs no prior GPIO configuration, remains input-only after completion and has no debounce.

```python
from rp2040_glitch import WaitGPIO, Pulse

device.configure_gpio(pin=output_pin, drive="push_pull", idle=0)
device.configure_sync(pin=sync_pin)  # Leave this pin unconnected.
device.run([WaitGPIO(trigger_pin, rising=True), Pulse(output_pin, width_cycles)])
```

Instruction-model base edge-to-push-pull latencies are seven cycles for GPIO and eleven for I2C/SPI, plus the post-wait interval of at least eight cycles. Open-drain/open-source direction edges add one cycle. Input synchronization is additional; these counts are not measured pin latency. Matcher boundary-delivery overflow returns `TRIGGER_OVERFLOW`.

### PWM

`configure_pwm` selects an unused push-pull pin and leaves the carrier off. PWM-only routines need neither generic GPIO nor sync configuration. Stop an active carrier with `abort()` before reconfiguring it.

`Pwm` accepts finite percentages in 0..100, rounded to integer basis points, ties to even. Frequency rounds to an integer period of 16..32768 system cycles; actual frequency is `sys_hz / period_cycles`. High cycles round to nearest, ties upward. Intermediate duty requires at least three high cycles and ten low cycles. 0%/100% are constant levels.

PIO/DMA applies duty changes and PWM-pin `SetPin` changes at whole-period boundaries on the GPIO timeline, including inserted spacing. Every PWM event must align to that grid. A timed block cannot combine PWM events with a hardware wait. PWM can continue through bus operations and non-PWM blocks, but each new PWM-containing block restarts carrier phase; gapless phase continuity is not guaranteed.

`SetPin` on the PWM pin selects low/high/floating without generic GPIO configuration. A final static mode stops the stream engine while retaining the level or high impedance; a later `Pwm` resumes it. See the [PWM notebook example](notebook.md#sequenced-500-khz-pwm).

## Status and cancellation

States are `idle`, `running`, `done`, `error` and `aborted`. Terminal results persist until clear/new start. Status includes step, elapsed time, poll bytes/attempts, matched condition, terminal error, compiled cycle/word counts and PWM metadata.

`info` and `status` report PWM configuration and its programmed final mode/duty, not instantaneous duty during a run. Modes are `carrier`, `low`, `high` and `floating`; `active` indicates the stream engine. Run elapsed time and PWM state come from a coherent worker snapshot.

Abort, deadlines, errors, DTR loss and unmount stop triggers/waveforms, restore GPIO idle/PWM low and cancel local I2C without recovery clocks. An abort received while an earlier reply is backpressured shuts down execution before its acknowledgment can be sent.

`bootloader` waits for worker cancellation, acknowledges, drains CDC for at most 100 ms, then enters ROM USB programming mode. The bound starts at command receipt even if an earlier response cannot drain. It does not flash or reboot the target; confirm ROM USB `2e8a:0003` or `picotool info` separately.
