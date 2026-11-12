"""Opt-in CDC smoke. Does not flash, generate stimulus, or certify waveforms."""
import argparse
import json
import sys
import time
from pathlib import Path

from rp2040_glitch import Device, DeviceError, WaitI2C


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    if not args.port or not args.config:
        print("SKIPPED HARDWARE: provide --port and --config with explicitly authorized fixture wiring")
        return 77
    try:
        config = json.loads(args.config.read_text())
    except (OSError, ValueError) as error:
        print(f"SKIPPED HARDWARE: unavailable/invalid fixture config: {error}")
        return 77
    if not isinstance(config, dict):
        print("SKIPPED HARDWARE: fixture config must be an object")
        return 77
    if config.get("authorized") is not True or config.get("voltage_compatible") is not True:
        print("SKIPPED HARDWARE: fixture authorization and voltage compatibility are required")
        return 77
    buses = config.get("buses", [])
    if not isinstance(buses, list) or len(buses) != 2 or any(not isinstance(b, dict) for b in buses) or {b.get("bus") for b in buses} != {0, 1}:
        print("SKIPPED HARDWARE: configure both buses and read-safe fixture addresses/registers")
        return 77
    if "gpio" not in config or "sync_pin" not in config or config.get("stimulus_available") is not True or config.get("bootloader_authorized") is not True:
        print("SKIPPED HARDWARE: gpio, sync_pin, stimulus_available and bootloader_authorized are required")
        return 77
    # Decode all fixture inputs before opening a port or changing pins.
    try:
        reads = [(b["bus"], b["address"], bytes.fromhex(b["register"]), bytes.fromhex(b["expected"])) for b in buses]
        wait = config["wait"]
        wait_step = WaitI2C(wait["bus"], wait["address"], wait["read"], bytes.fromhex(wait["prefix"]), 5_000_000)
        from rp2040_glitch import _compile_steps, _target, _uint
        pins = []
        for bus in buses:
            sda, scl = _uint(bus["sda"], 29), _uint(bus["scl"], 29)
            _uint(bus.get("hz", 100000), 1500000, 10000)
            if sda % 4 != 2 * bus["bus"] or scl != sda + 1:
                raise ValueError("invalid I2C mux pins")
            pins.extend((sda, scl))
        gpio = config["gpio"]
        if not isinstance(gpio, list):
            raise ValueError("gpio must be a list of pin/drive/idle objects")
        for entry in gpio:
            if not isinstance(entry, dict) or set(entry) != {"pin", "drive", "idle"}:
                raise ValueError("each gpio entry requires exactly pin, drive and idle")
            _uint(entry["pin"], 29)
            if entry["drive"] not in ("push_pull", "open_drain", "open_source"):
                raise ValueError("invalid GPIO drive")
            _uint(entry["idle"], 1)
            pins.append(entry["pin"])
        pins.append(_uint(config["sync_pin"], 29))
        if len(set(pins)) != len(pins):
            raise ValueError("colliding fixture pins")
        _compile_steps([wait_step])
        for bus, address, register, expected in reads:
            _target(bus, address)
            _uint(len(expected), 1024, 1)
            _uint(len(register), 1024)
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        print(f"SKIPPED HARDWARE: incomplete/invalid fixture configuration: {error}")
        return 77
    if wait.get("quiet_during_smoke") is not True:
        print("SKIPPED HARDWARE: wait fixture must guarantee no matching prefix during cancellation check")
        return 77
    with Device(args.port) as device:
        print(json.dumps(device.info(), sort_keys=True))
        try:
            device._request("invalid_command")
        except DeviceError as error:
            assert error.code == "BAD_REQUEST", error
        else:
            raise AssertionError("malformed command was accepted")
        assert device.info()["protocol"] == 3
        for bus in buses:
            device.configure_bus(bus["bus"], sda=bus["sda"], scl=bus["scl"], hz=bus.get("hz", 100000), pullups=False)
        for entry in gpio:
            device.configure_gpio(**entry)
        device.configure_sync(pin=config["sync_pin"])
        for bus, address, register, expected in reads:
            assert expected, "expected read data cannot be empty"
            actual = device.read_register(bus, address, register, len(expected))
            assert actual == expected, (bus, actual.hex(), expected.hex())
        device.start([WaitI2C(wait["bus"], wait["address"], wait["read"], bytes.fromhex(wait["prefix"]), 5_000_000)], timeout_us=6_000_000)
        started = time.monotonic()
        assert device.status()["state"] == "running"
        assert time.monotonic() - started < 1.0, "status stalled during hardware wait"
        assert device.abort()["state"] == "aborted"
        device.bootloader()
    print("CDC SMOKE PASSED; BOOTSEL acknowledgment received. Verify 2e8a:0003/picotool separately.")
    print("UNMEASURED: physical waveform timing, arbitration, SPI 10 MHz, load integrity and disconnect safety")
    return 0


if __name__ == "__main__":
    sys.exit(main())
