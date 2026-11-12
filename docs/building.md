# Building and setup

**Bare `make` only builds. `make install` builds, flashes and reboots the board.** CMake also supports build-only work.

## Build-only firmware

Requires Pico SDK >=2.3.1, an ARM toolchain, CMake, Python >=3.10 and a native C compiler with pthread support for the test harnesses. Set the board to match your hardware; `pico` below selects the standard Pico.

```sh
export PICO_SDK_PATH=/usr/share/pico-sdk
export PICO_BOARD=pico
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target rp2040_glitch
```

Outputs: `build/rp2040_glitch.uf2`, `build/rp2040_glitch.bin` and `build/rp2040_glitch.elf`. The build also generates `build/timing.pio.json` for instruction-model tests. See [development and testing](development.md) before interpreting software checks as hardware evidence.

The SDK `copy_to_ram` binary type copies the firmware into SRAM at boot, including SDK helpers used during execution. Flash still stores the image; normal UF2/BOOTSEL loading is unchanged. SRAM execution removes XIP fetch variability.

## Clock configuration

CMake enables `PICO_USE_FASTEST_SUPPORTED_CLOCK` by default and sets `GLITCH_SYS_CLOCK_KHZ=SYS_CLK_KHZ`, using the SDK's clock selection. Startup frequency and core voltage depend on the board/SDK configuration; there is no universal frequency/voltage promise across boards. USB runs at 48 MHz.

Use `info()["sys_hz"]` for configured time-to-cycle conversion. This value and the configured I2C rate are **not calibrated measurements**. For example, a 200-MHz system clock gives 5 ns per cycle; that arithmetic is not a measurement of your board. Verify physical timing and retain measured-clock calibration where required.

To disable the SDK's fastest-supported default and use its ordinary startup configuration:

```sh
cmake -S . -B build -DPICO_USE_FASTEST_SUPPORTED_CLOCK=OFF \
  -DGLITCH_SYS_CLOCK_KHZ=SYS_CLK_KHZ
cmake --build build --target rp2040_glitch
```

To restore the default, set `PICO_USE_FASTEST_SUPPORTED_CLOCK=ON` with `GLITCH_SYS_CLOCK_KHZ=SYS_CLK_KHZ`. CMake caches these settings. An explicit numeric `GLITCH_SYS_CLOCK_KHZ` override (in kHz) applies after SDK clock initialization; it does not by itself change the startup clock/voltage setting. An unattainable PLL setting reports `CLOCK_ERROR`; an unstable clock may require physical BOOTSEL recovery.

## Linux USB permissions

Install local-login USB permissions once, then reconnect the Pico:

```sh
sudo make install_rules
```

This installs [tools/70-pico.rules](../tools/70-pico.rules) and reloads udev without building or flashing. The rules grant runtime and BOOTSEL access to the active local login, not world access or an SSH-only session. Select the serial-specific `/dev/rp2040-glitch-SERIAL` symlink rather than assuming a stable `/dev/ttyACM` number. Runtime identity is `2e8a:000a`, manufacturer `Glitch Tool`, product `RP2040 Glitch Tool`, with a board-specific serial number; ROM BOOTSEL is `2e8a:0003`.

## Python environment

From the repository root:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python/requirements.txt
export PYTHONPATH="$PWD/python"
```

The requirements currently contain only pyserial. JupyterLab is optional and installed separately:

```sh
python -m pip install jupyterlab
PYTHONPATH="$PWD/python" jupyter lab example/notebook.ipynb
```

Read the [notebook guide](notebook.md) and [safety requirements](safety.md) before supplying fixture values or enabling any hardware cell. See the [Python API](python-api.md) for scripts and [development](development.md) for native/model tests and authorized CDC smoke requirements.
