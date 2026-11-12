# Development and verification

## Source layout

| File | Responsibility |
|---|---|
| `main.c` | High-Z startup, clock/timing initialization and USB task loop |
| `usb_control.c` / `usb_control.h` | Core-0 commands, configuration/staging, JSON replies, CDC session lifecycle and TinyUSB callbacks |
| `executor.c` / `executor.h` | Core-1 worker, ownership handoff, snapshots and acknowledged cancellation |
| `protocol.c` / `protocol.h` | Bounded framing, parsing and response encoding |
| `sequence.c` / `sequence.h` | Staged program, compilation, execution state, polling and deadlines |
| `timing.c` / `timing.h` | GPIO/PWM compiler, trigger packing and timing contracts |
| `timing_hw.c` / `timing.pio` | GPIO, PIO/DMA waveform execution, matchers, scheduled releases and shutdown |
| `i2c_bus.c` / `i2c_bus.h` | Transfer state machine and native-driver contract |
| `i2c_hw.c` | Native RP2040 controllers, FIFO preparation, completion and cancellation |
| `python/rp2040_glitch.py` | Synchronous host API, validation and error handling |


## Ownership and execution

Core 0 handles USB, parsing and response formatting. Configuration, staging, compilation and direct-transfer submission acquire exclusive ownership while core 1 is acknowledged parked and idle. Before accepting a run, firmware resolves schedule groups and trigger metadata, encodes scheduled I2C FIFO commands, reserves the maximum simultaneous event resources, and builds PWM DMA descriptors and FIFO-prefill words. These buffers remain immutable during execution. A held PWM carrier retains private tail storage, independent of subsequent staging.

After release, core 1 continuously services native I2C, PIO/DMA state, sequence advancement, deadlines and PWM faults. It performs completion/refill work before status snapshots; abort and ownership requests take priority over servicing. Block startup performs bounded register/FIFO setup without a PWM-prefill spin. Core 1 never waits for USB response formatting. Cancellation acknowledgment follows teardown of event producers, DMA and controller use. **Status polling does not advance the program.**

Native writes fill available FIFO space in batches. Consecutive ordinary writes advance in the same servicing pass that observes completion. Same-target FIFO preparation does not wait for another USB/main-loop pass. Different targets, peripheral reset/disable acknowledgment and bus conditions can require additional worker polls. Completion requires confirmed data, STOP and local master idle; queued bytes are not counted as acknowledged. This is not a guarantee of zero STOP-to-START time.


`usb_control_init(bool clock_valid)` runs once on core 0 after timing initialization. It clears sequence state, initializes the executor with the module's static receive buffer, then resets request framing. That receive buffer lives for the entire session/worker lifetime. `usb_control_task()` services disconnects, pending replies, outgoing response bytes, incoming requests and the BOOTSEL deadline in that order; `main` calls `tud_task()` separately first. Startup intentionally avoids `board_init()` so board-specific LED or UART setup cannot drive fixture pins.

Core-0 staging and compilation occur only under acknowledged exclusive ownership; they are not concurrent edits to core-1 run state. `sequence.c` owns staging/compiled buffers and worker-owned execution state; `executor.c` owns the handoff. USB disconnect/DTR loss cancels execution and clears pending input/replies. Urgent abort/BOOTSEL teardown precedes draining a backpressured reply, and direct-transfer replies retain their original request IDs.

## Native and model tests

Use the [build prerequisites and environment](building.md): Pico SDK >=2.3.1, ARM toolchain, CMake, Python >=3.10 with pyserial and a native C compiler with pthread support. From the repository root, build the firmware and assembled PIO data first:

```sh
PICO_BOARD=pico cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target rp2040_glitch
PYTHONPATH=python python3 -m unittest discover -s tests -p 'test_*.py'
```

`tests/test_pio.py` requires `build/timing.pio.json`; set `PIO_JSON` to the generated file when using a different build directory. Missing assembled PIO data is not a successful timing-model verification. Use a separate build directory rather than overwriting a configuration for another board.

Offline tests exercise production transfer, sequence and protocol code; a threaded executor harness checks ownership, progress and cancellation. Register/FIFO models and assembled PIO tests cover timing-engine behavior. The firmware harness includes `usb_control.c` before `main.c` with the clock definition supplied first, allowing its native model to exercise the real startup/control lifecycle without production test hooks.

For a standalone native startup/control smoke:

```sh
smoke_dir="$(mktemp -d -t rp2040-glitch-smoke.XXXXXX)"
cc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
  -I. -Itests/native protocol.c sequence.c i2c_bus.c timing.c \
  tests/firmware_harness.c -Wl,--gc-sections -o "$smoke_dir/firmware"
"$smoke_dir/firmware"
```

**Native/model checks establish neither electrical timing nor safety.** CDC smoke checks communication/lifecycle, not waveform certification. Release 1.0 does not certify analog glitching, timing under load or completed hardware acceptance. Do not use `make install`, flashing or permission installation as software verification.

## Authorized CDC hardware smoke


The smoke script never flashes and does not generate bus stimulus. Missing port/config or fixture authorization returns exit 77 with `SKIPPED HARDWARE`, not a passing waveform test:

```sh
PYTHONPATH=python python3 tests/hardware_smoke.py --port PORT --config fixture.json
```

Create a JSON config with **your actual verified fixture values** using this shape (the nulls deliberately cannot drive hardware):

```json
{
  "authorized": false,
  "voltage_compatible": false,
  "stimulus_available": false,
  "bootloader_authorized": false,
  "buses": [
    {"bus": 0, "sda": null, "scl": null, "hz": 100000,
     "address": null, "register": "", "expected": ""},
    {"bus": 1, "sda": null, "scl": null, "hz": 100000,
     "address": null, "register": "", "expected": ""}
  ],
  "gpio": [{"pin": null, "drive": "open_drain", "idle": 1},
           {"pin": null, "drive": "push_pull", "idle": 0}],
  "sync_pin": null,
  "wait": {"bus": 0, "address": null, "read": false, "prefix": "",
           "quiet_during_smoke": false}
}
```

`gpio` is a list of objects with exactly `pin`, `drive` and `idle`; use `[]` if no generic outputs are required for this wait-only smoke. `sync_pin` is required because the smoke arms a hardware wait, and must be an unused, unconnected GPIO. Every pin must be an explicit GPIO0–29 integer, and all pins must be distinct; generic GPIOs have no adjacency constraint. Drives are `push_pull`, `open_drain` or `open_source`; idle is integer 0/1, not boolean. The script validates fixture values before opening CDC. `register`, `expected` and `prefix` are hex strings. Both expected read payloads must be nonempty and register accesses read-safe. Authorize GPIOs only after checking external inactive bias; keep the glitch power stage disabled. The wait fixture must guarantee that its prefix does not occur during cancellation testing. Stimulus availability is explicit, but this script does not claim to have exercised all stimulus traces.

The script checks protocol version 3, malformed-command recovery, reads on both configured buses, status while waiting, explicit abort and an accepted BOOTSEL command. Confirm ROM USB enumeration `2e8a:0003` or `picotool info` yourself. The external physical acceptance checklist below is separate from this CDC check. An acknowledgment does not prove ROM enumeration, waveform accuracy or analog glitch behavior.

Without fixture arguments, the retained script safely reports `SKIPPED HARDWARE` and exits 77:

```sh
PYTHONPATH=python python3 tests/hardware_smoke.py
```

## External physical acceptance

On an explicitly authorized, electrically verified fixture, use an external analyzer/scope to measure:

- Actual I2C SCL rate and STOP-to-next-START distributions under USB load, including clock stretching and arbitration behavior.
- I2C/SPI/GPIO trigger arming, edge-to-output latency and boundary behavior.
- GPIO pulse widths, output spacing, drive polarity and held/floating states.
- PWM frequency, synchronized duty/mode boundaries, floating behavior and reset edges.
- Abort, DTR loss, disconnect and BOOTSEL shutdown, including when USB responses are backpressured.

Keep the power stage disabled for transport/digital checks before calibrating analog voltage response and restoration. No minimum physical turnaround time is guaranteed by software models. Follow [electrical safety](safety.md) and the [timing contracts](timing.md); physical acceptance requires measurements, not a successful native or CDC run.
