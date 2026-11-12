# Wiring and safety

- At firmware entry, GPIO0–29 are initialized as inputs with internal pulls disabled. Pins remain high-impedance until configured. This does not cover ROM boot, dedicated flash/USB/debug pins or external board circuitry.
- I2C bus IDs select hardware I2C0/I2C1. SDA must be even and SCL must equal SDA+1. I2C0 requires SDA modulo 4 = 0; I2C1 requires SDA modulo 4 = 2. Check board exposure and other uses before choosing pins.
- I2C accepts **10 kHz–1.5 MHz**, default 100 kHz, with internal pulls off. Rates above 1 MHz are experimental overclocking beyond the RP2040's specified Fast-mode Plus rate—not I2C High-speed mode. Fit external pull-ups and suitable level translation; verify actual SCL timing and slave behavior on the fixture. The requested rate is not a measured wire rate.
- GPIO drive modes are `push_pull`, `open_drain` and `open_source`. Open-drain drives low or releases; open-source drives high or releases. Supply external bias for released states. PWM is push-pull.
- Configured bus, GPIO, sync, SPI, PWM and GPIO-trigger input pins must not collide. SPI inputs and GPIO-trigger inputs have internal pulls disabled.
- Use safe external bias through power loss, reset and BOOTSEL. Firmware cannot protect an unpowered or ROM-controlled board. GPIO controls an external glitch circuit, never a target power rail directly.
- Keep the power stage disabled during digital timing checks. **A successful voltage-lowering write followed by failed restoration can leave the VRM low.** GPIO idle restoration does not undo an I2C command; recovery must be explicit.

### Sync pin

Hardware waits require `configure_sync(pin=...)`. Choose an unused GPIO and **leave it disconnected from the target**.

This pin bridges the PIO1 matcher to the PIO0 waveform engine: low while armed, latched high on a match, cleared when the block completes or stops. Both PIO blocks access the same GPIO; no jumper is needed. It is not a programmable analyzer marker. Pure pulse, delay and PWM programs need no sync pin.
