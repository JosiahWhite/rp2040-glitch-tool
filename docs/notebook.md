# Example glitch notebook guide

Open [notebook.ipynb](../example/notebook.ipynb) after following [Python environment setup](building.md#python-environment). Python >=3.10 and protocol 3 are required; JupyterLab is optional and installed separately from the pyserial requirements. From the repository root:

```sh
PYTHONPATH="$PWD/python" jupyter lab example/notebook.ipynb
```

Opening the notebook does not connect or drive hardware. Its fixture defaults are deliberately unset, its authorization flags are false, and code-cell outputs are empty. Fill and verify the fixture before executing hardware cells. Software models do not certify physical timing or electrical safety.

## Fixture fields and authorization

- `PORT`: serial-number-specific CDC port; `BUSES`: both hardware-I2C pin maps; `TARGETS`: explicit target addresses and register bytes. No default address/pinout is treated as safe.
- `GPIO_CONFIG`: explicitly chosen pins mapped to `{"drive": ..., "idle": ...}`. The logic-analyzer example uses two nonadjacent pins; `reset_pin`/`glitch_pin` are fixture labels, not firmware roles. GPIO0–29 need not otherwise be adjacent.
- `AUTHORIZED = False`: enable only after verifying wiring, compatible voltage, external pull-ups/bias and a disabled glitch power stage. It gates opening the device and target access.
- `SYNC_PIN = None`: reserve an unused, **unconnected** internal PIO1-to-PIO0 match latch only for hardware waits. It is not a target signal or analyzer marker; no jumper is required. Pure pulse/PWM routines need no sync pin.
- `SPI`: explicit CS/SCK/data map, mode and bit order. `TRIGGER_SOURCE` chooses `"spi"` or `"gpio"`; supply `SPI_PREFIX` or a separate free input `GPIO_PIN`. Set `GPIO_RISING=False` for falling edges. `I2C_PREFIX` separately configures I2C matching.
- `WRITE_DATA`, `POLL_MASK` and `POLL_VALUE`: explicit bytes for read-safe/authorized register operations and masked polling.
- `MEASURED_CLOCK_HZ`: optional measured clock for conversion; `info["sys_hz"]` is configured, not measured. Preserve calibration when comparing physical events.
- PWM, static PWM states, idle restoration, sweep and BOOTSEL each have separate opt-ins described below; general fixture authorization does not enable them automatically.

All bus/GPIO/sync/SPI/PWM/trigger pins must be distinct and compatible with the [mux and safety constraints](safety.md). Idle is integer 0/1, not boolean. External bias must remain safe through power loss and ROM BOOTSEL. Never connect GPIO directly to a target power rail; failed voltage restoration can leave a VRM low.

The notebook covers both-bus register access, masked polling, pulse/delay, I2C/SPI matching, GPIO rising/falling edges, PWM and a parameter sweep. Firmware does not provide passive capture; use an external analyzer. Read the [Python API](python-api.md) for drive/Pulse/SetPin semantics, validation and errors, and [timing](timing.md) for implicit eight-cycle spacing, trigger arming and held states. A hardware matcher is live from block launch, not only when execution reaches its wait step.

Interrupt a waiting cell to cancel; explicitly abort held outputs and close the device when finished. `start`/`run` hold successful final states by default; `hold_final=False` restores idle/PWM low. Abort, error, disconnect/DTR loss and BOOTSEL restore those safe states. `clear` resets staging/results without clearing held outputs. See [status and cancellation](timing.md#status-and-cancellation).

## Sequenced 500-kHz PWM

The PWM cell requires separate `PWM_AUTHORIZED = False` and `PWM_PIN = None` guards. Choose an unused, exposed, voltage-compatible pin, never a presumed safe default. The example configures `reset_pin` as open-drain idle=1 and PWM at `hz=500000`, then runs:

```text
Pwm(60)
Pulse(reset_pin, seconds_to_cycles(0.100, clock_hz=clock_hz))
Delay(seconds_to_cycles(10e-6, clock_hz=clock_hz))
Pwm(10)
Delay(seconds_to_cycles(100e-6, clock_hz=clock_hz))
Pwm(60)
```

The runnable cell supplies its clock explicitly (`pwm_clock_hz`, optionally measured). The generic GPIO is needed only for the fixture-reset pulse; PWM-only routines require no generic GPIO or sync configuration.

Duty and PWM-pin SetPin changes use PIO/DMA whole-period boundaries (2 µs at exactly 500 kHz), not USB timing or CPU dispatch. Events must align on the GPIO timeline including inserted spacing. Each PWM-containing block restarts carrier phase; a hardware wait must be in a separate block, for example across Write/Poll. Frequency/duty quantization and minimum high/low counts are specified in [PWM timing](timing.md#pwm); unsupported frequency/duty/grid combinations are rejected, not stretched.

The pin-state cell additionally requires `PWM_STATES_AUTHORIZED = False` and demonstrates carrier → floating → low → high → carrier → floating on the same explicitly selected PWM pin. No generic GPIO configuration is needed on that pin. A final static mode stops the stream engine; a later Pwm resumes it. `info`/`status` expose `pwm.final_mode` (`carrier`, `low`, `high`, `floating`) and `active`; final fields describe the endpoint, not instantaneous mode during a run.

The separate `hold_final=False` cell additionally requires `RESTORE_IDLE_AUTHORIZED = False`. Successful final duty/static state otherwise persists while the device stays open; `close()` and context exit drop DTR even after success. Use external inactive bias through power loss/ROM release. Physical use requires scope measurements of frequency, synchronized duty/mode changes, floating behavior, reset edges and shutdown. Software models do not certify physical timing.

## Sweep and BOOTSEL

`SWEEP_AUTHORIZED = False` separately gates the parameter sweep after a digital-only fixture check. Each call waits for on-device completion; it does not use USB-timed edges or automatically retry mutations.

`BOOTSEL_AUTHORIZED = False` separately gates the final ROM USB programming request. It does not flash firmware or reset the target. Confirm ROM enumeration using `2e8a:0003` or `picotool info`; an uncertain or accepted CDC acknowledgment is not enumeration proof. The separately authorized [CDC hardware smoke](development.md#authorized-cdc-hardware-smoke) uses its own fixture JSON and authorization requirements.
