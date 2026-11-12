# RP2040 Glitch Tool

![Slop Em Up](https://media3.giphy.com/media/v1.Y2lkPTc5MGI3NjExYTV5cjJ3NTVxZWU1Znd6ODQ1dnpqdml4bjE2OG05ZDVuaGZzMTQxNSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/5NiitcM3zBWFgT3m9j/giphy.gif)

USB CDC firmware and a Python API for fixture-controlled I2C transfers,
GPIO pulse sequences, scheduled writes, hardware-triggered waits and PWM.
The host stages a complete program; core 1 executes it independently of
core 0's USB handling, with PIO/DMA producing waveform and release events.

## Capabilities

- Two native I2C buses, register access and masked active polling.
- GPIO pulse/delay/state sequences with push-pull, open-drain or open-source drive.
- Sequenced PWM duty and static pin states at hardware period boundaries.
- Nonblocking scheduled I2C writes with explicit completion barriers.
- Hardware I2C-prefix, SPI-prefix and GPIO-edge waits.
- Bounded protocol, worker-owned execution and acknowledged cancellation.

Firmware does not provide passive capture; use an external analyzer.
Requested clocks and programmed cycles are not measured wire timing.

## Build without flashing

**Bare `make` only builds; `make install` builds and flashes hardware.**
See the [build and setup guide](docs/building.md). CMake also builds without flashing:

```sh
PICO_BOARD=pico cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target rp2040_glitch
```

Outputs are `build/rp2040_glitch.uf2`, `.bin` and `.elf`.

## Documentation

- [Building, clock configuration, USB permissions and Python setup](docs/building.md)
- [Electrical safety, pin constraints and the internal sync latch](docs/safety.md)
- [Python API and sequence steps](docs/python-api.md)
- [Execution, scheduled writes, trigger timing and PWM](docs/timing.md)
- [Protocol 3, command grammar, limits and errors](docs/protocol.md)
- [Architecture, native/model tests and authorized hardware smoke](docs/development.md)
- [Example glitch notebook guide and fixture authorization](docs/notebook.md)

The [example workflow](example/notebook.ipynb) is provided as a Jupyter notebook.
It requires explicit fixture values and opt-in authorization before driving hardware.

## Identity and safety

Application release 1.0 uses **wire protocol 3**. Runtime USB is `2e8a:000a`,
manufacturer `Glitch Tool`, product `RP2040 Glitch Tool`, with a board-specific
serial number and Linux alias `/dev/rp2040-glitch-SERIAL`.

GPIO controls an external glitch circuit, never a target power rail directly.
Use compatible levels and safe external bias; keep the power stage disabled
until digital timing is measured. Failed voltage restoration can leave a VRM low.
Native/model and CDC tests do not certify electrical safety, physical timing,
analog glitching or hardware acceptance; authorized external measurements are required.
