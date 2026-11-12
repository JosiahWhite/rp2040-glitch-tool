"""Synchronous, bounded CDC API. No retries: reopen after transport/protocol failure."""
from dataclasses import dataclass
import json
import math
import re
import time

import serial

U32 = 0xffffffff
MAX_TIMEOUT_US = 60_000_000


class DeviceError(Exception):
    def __init__(self, code, step=None, detail=None):
        self.code, self.step, self.detail = code, step, detail
        super().__init__(f"{code}" + (f" at step {step}" if step is not None else ""))


class ProtocolError(Exception):
    pass


class TransportError(Exception):
    pass


class BootloaderUncertainError(TransportError):
    """Command may have rebooted the board; confirm BOOTSEL enumeration externally."""


def _uint(value, maximum=U32, minimum=0):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"expected integer in {minimum}..{maximum}")
    return value


def _bool(value):
    if type(value) is not bool:
        raise ValueError("expected bool")
    return int(value)


def _bytes(value, maximum, minimum=0):
    if not isinstance(value, bytes) or not minimum <= len(value) <= maximum:
        raise ValueError(f"expected bytes of length {minimum}..{maximum}")
    return value.hex() or "-"


def _deadline(value):
    return _uint(value, MAX_TIMEOUT_US, 1)


def _target(bus, address):
    return _uint(bus, 1), _uint(address, 0x77, 0x08)


def _positive_seconds(value, *, zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (not zero and value == 0):
        raise ValueError("expected finite positive seconds")
    return value


def seconds_to_cycles(seconds, *, clock_hz):
    _positive_seconds(seconds, zero=True)
    _positive_seconds(clock_hz)
    product = seconds * clock_hz
    if not math.isfinite(product):
        raise ValueError("cycle count exceeds u32")
    return _uint(round(product))

def microseconds_to_cycles(microseconds, *, clock_hz):
    _positive_seconds(clock_hz)
    _positive_seconds(microseconds * 1e-6, zero=True)
    product = microseconds * clock_hz // 1_000_000
    if not math.isfinite(product):
        raise ValueError("cycle count exceeds u32")
    return _uint(round(product))

@dataclass(frozen=True)
class Delay:
    cycles: int


@dataclass(frozen=True)
class Pulse:
    pin: int
    width_cycles: int


@dataclass(frozen=True)
class SetPin:
    """Set a configured GPIO or PWM pin low, high, or floating."""
    pin: int
    state: str


@dataclass(frozen=True)
class Pwm:
    """Duty percent, rounded to basis points (nearest, ties to even) on staging."""
    duty_percent: float


@dataclass(frozen=True)
class Write:
    bus: int
    address: int
    data: bytes
    timeout_us: int = 10000


@dataclass(frozen=True)
class WriteAt:
    """Nonblocking write offset from the preceding command's completion.

    Consecutive declarations share that reference (at most two per group).
    Pulse release, hardware-wait match, and compiled Delay/SetPin/Pwm completion
    provide hardware references. Write/Poll/SyncWrites use firmware-observed
    completion. Payloads fit the 16-entry FIFO; timeout_us includes preparation,
    reference waiting and transmission. DMA/controller latency remains extra;
    this is not first-SCL timing.
    """
    bus: int
    address: int
    data: bytes
    at_cycles: int
    timeout_us: int = 10000


@dataclass(frozen=True)
class SyncWrites:
    """Wait for scheduled writes to complete successfully before continuing."""


@dataclass(frozen=True)
class Poll:
    bus: int
    address: int
    register: bytes
    length: int
    mask: bytes
    value: bytes
    interval_us: int = 1000
    timeout_us: int = 1000000
    transfer_timeout_us: int = 10000


@dataclass(frozen=True)
class WaitI2C:
    bus: int
    address: int
    read: bool
    prefix: bytes = b""
    timeout_us: int = 1000000


@dataclass(frozen=True)
class WaitSPI:
    prefix: bytes
    timeout_us: int = 1000000

@dataclass(frozen=True)
class WaitGPIO:
    pin: int
    rising: bool = True
    timeout_us: int = 1000000



def _compile_steps(steps):
    commands, payload, waits = [], 0, 0
    for step in steps:
        if len(commands) == 64:
            raise ValueError("at most 64 steps")
        if type(step) in (Delay, Pulse):
            cycles = step.width_cycles if type(step) is Pulse else step.cycles
            _uint(cycles)
            if cycles != 0 or type(step) is not Delay:
                _uint(cycles, U32, 8)
            args = (("pulse", _uint(step.pin, 29), cycles) if type(step) is Pulse
                    else ("delay", cycles))
        elif type(step) is SetPin:
            if type(step.state) is not str or step.state not in ("low", "high", "floating"):
                raise ValueError("expected low, high, or floating state")
            args = ("set", _uint(step.pin, 29), step.state)
        elif type(step) is Pwm:
            duty = step.duty_percent
            if isinstance(duty, bool) or not isinstance(duty, (int, float)) or not 0 <= duty <= 100 or not math.isfinite(duty):
                raise ValueError("expected finite duty percent in 0..100")
            args = ("pwm", round(duty * 100))
        elif type(step) is Write:
            args = ("write", *_target(step.bus, step.address), _deadline(step.timeout_us), _bytes(step.data, 256, 1))
            payload += len(step.data)
        elif type(step) is WriteAt:
            args = ("write_at", *_target(step.bus, step.address),
                    _uint(step.at_cycles, U32, 8), _deadline(step.timeout_us),
                    _bytes(step.data, 16, 1))
            payload += len(step.data)
        elif type(step) is SyncWrites:
            args = ("sync_writes",)
        elif type(step) is Poll:
            _uint(step.length, 256, 1)
            args = ("poll", *_target(step.bus, step.address), _bytes(step.register, 256), step.length,
                    _bytes(step.mask, step.length, step.length), _bytes(step.value, step.length, step.length),
                    _uint(step.interval_us, MAX_TIMEOUT_US), _deadline(step.timeout_us), _deadline(step.transfer_timeout_us))
            payload += len(step.register) + len(step.mask) + len(step.value)
        elif type(step) is WaitI2C:
            args = ("wait_i2c", *_target(step.bus, step.address), _bool(step.read), _bytes(step.prefix, 2), _deadline(step.timeout_us))
            payload += len(step.prefix)
            waits += 1
        elif type(step) is WaitSPI:
            args = ("wait_spi", _bytes(step.prefix, 4, 1), _deadline(step.timeout_us))
            payload += len(step.prefix)
            waits += 1
        elif type(step) is WaitGPIO:
            args = ("wait_gpio", _uint(step.pin, 29), _bool(step.rising), _deadline(step.timeout_us))
            waits += 1
        else:
            raise ValueError("unknown routine step")
        commands.append(args)
    if not commands or waits > 1 or payload > 8192:
        raise ValueError("empty routine, multiple hardware waits, or payload exceeds 8192 bytes")
    references = set()
    pending = False
    group_count = 0
    for i, args in enumerate(commands):
        op = args[0]
        if op in ("sync_writes", "write", "poll"):
            pending = False
        if op != "write_at":
            group_count = 0
            continue
        if not i:
            raise ValueError("WriteAt requires a preceding command")
        if not group_count:
            if pending:
                raise ValueError("join pending writes before another WriteAt reference command")
            references.add(i - 1)
            pending = True
        elif args[3] <= commands[i - 1][3]:
            raise ValueError("WriteAt offsets within a group must increase")
        group_count += 1
        if group_count > 2:
            raise ValueError("at most two WriteAt declarations per reference")
    block_pwm = block_wait = False
    for i, args in enumerate(commands):
        op = args[0]
        if i in references or op in ("write", "write_at", "poll", "sync_writes"):
            block_pwm = block_wait = False
        block_pwm |= op == "pwm"
        block_wait |= op in ("wait_i2c", "wait_spi", "wait_gpio")
        if block_pwm and block_wait:
            raise ValueError("PWM and hardware wait cannot share a timed block")
    # Firmware knows held states, drive modes and the PWM pin. Count only a
    # lower bound: SetPin may collapse or target PWM, and implicit spacing can
    # add frames. Firmware enforces the exact state-dependent FIFO capacity.
    frames, elapsed, in_block = 0, 0, False
    maximum_span = 0x0fffffff + 6
    for i, args in enumerate(commands + [("end",)]):
        op = args[0]
        if i in references and in_block:
            frames += ((elapsed + maximum_span - 1) // maximum_span if elapsed else 0) + 1
            elapsed, in_block = 0, False
        if op in ("write", "write_at", "poll", "sync_writes", "end"):
            if in_block:
                frames += ((elapsed + maximum_span - 1) // maximum_span if elapsed else 0) + 1
            elapsed, in_block = 0, False
        elif op == "delay":
            elapsed += args[1]
            in_block = True
        elif op in ("pwm", "set"):
            in_block = True
        elif op in ("wait_i2c", "wait_spi", "wait_gpio"):
            frames += (elapsed + maximum_span - 1) // maximum_span if elapsed else 0
            frames += 1
            elapsed, in_block = 0, True
        else:
            if elapsed:
                frames += (elapsed - 1 + maximum_span - 1) // maximum_span
            frames += (args[2] + maximum_span - 1) // maximum_span
            elapsed, in_block = 0, True
    if frames * 3 > 1024:
        raise ValueError("compiled waveform exceeds 1024 FIFO words")
    return commands


class Device:
    def __init__(self, port, *, timeout=2.0):
        self.timeout = _positive_seconds(timeout)
        self._serial = None
        self._id = 0
        self._active = False
        self._synchronized = True
        try:
            self._serial = serial.Serial(port, timeout=min(timeout, 0.05), write_timeout=timeout)
            self.info()
        except (OSError, serial.SerialException) as error:
            self.close()
            raise TransportError(str(error)) from error
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            if self._active and self._synchronized:
                self.abort()
        finally:
            self.close()

    def close(self):
        port, self._serial = self._serial, None
        self._synchronized = False
        if port is not None:
            try:
                port.dtr = False
            except (OSError, serial.SerialException):
                pass
            finally:
                port.close()

    def _invalid(self, message):
        self.close()
        raise ProtocolError(message)

    def _request(self, command, *args):
        if not self._synchronized or self._serial is None:
            raise TransportError("session closed; reopen before issuing another command")
        next_id = self._id % U32 + 1
        request = (" ".join(map(str, (next_id, command, *args))) + "\n").encode("ascii")
        if len(request) > 4096:
            raise ValueError("request exceeds 4096 bytes")
        self._id = next_id
        deadline = time.monotonic() + self.timeout
        try:
            offset = 0
            while offset < len(request):
                count = self._serial.write(request[offset:])
                if not count or time.monotonic() >= deadline:
                    raise TransportError("request write timed out; not retried")
                offset += count
            response = bytearray()
            while not response.endswith(b"\n"):
                if time.monotonic() >= deadline:
                    raise TransportError("response timed out; session closed")
                chunk = self._serial.read_until(b"\n", size=32769 - len(response))
                response.extend(chunk)
                if len(response) > 32768:
                    self._invalid("response exceeds 32768 bytes")
            try:
                reply = json.loads(response)
            except (ValueError, UnicodeError):
                self._invalid("invalid JSON response")
            if not isinstance(reply, dict) or type(reply.get("id")) is not int or reply["id"] != self._id or type(reply.get("ok")) is not bool:
                self._invalid("invalid response envelope or mismatched request ID")
            if not reply["ok"]:
                error = reply.get("error")
                if not isinstance(error, dict) or not isinstance(error.get("code"), str) or (error.get("step") is not None and type(error["step"]) is not int):
                    self._invalid("invalid device error")
                raise DeviceError(error["code"], error.get("step"), error.get("detail", reply.get("detail")))
            result = reply.get("result")
            if not isinstance(result, dict):
                self._invalid("result must be an object")
            return result
        except DeviceError:
            raise
        except ProtocolError:
            raise
        except KeyboardInterrupt:
            # A partially exchanged frame cannot safely be followed by abort.
            self.close()
            raise
        except (OSError, serial.SerialException, TransportError) as error:
            self.close()
            raise TransportError(str(error)) from error

    def info(self):
        result = self._request("info")
        if type(result.get("protocol")) is not int or result["protocol"] != 3:
            self._invalid("unsupported protocol version")
        return result

    def configure_bus(self, bus, *, sda, scl, hz=100000, pullups=False):
        _uint(bus, 1)
        _uint(sda, 29)
        _uint(scl, 29)
        if sda % 4 != bus * 2 or scl != sda + 1:
            raise ValueError("bus pins do not match hardware I2C mux")
        self._request("bus_config", bus, sda, scl, _uint(hz, 1500000, 10000), _bool(pullups))

    def configure_gpio(self, *, pin, drive="push_pull", idle=0):
        _uint(pin, 29)
        if type(drive) is not str or drive not in ("push_pull", "open_drain", "open_source"):
            raise ValueError("expected push_pull, open_drain, or open_source drive")
        self._request("gpio_config", pin, drive, _uint(idle, 1))

    def configure_sync(self, *, pin):
        self._request("sync_config", _uint(pin, 29))

    def configure_pwm(self, *, pin, hz=500000):
        """Configure an initially off carrier; firmware validates clock constraints."""
        self._request("pwm_config", _uint(pin, 29), _uint(hz, U32, 1))

    def configure_spi(self, *, cs, sck, data, mode=0, lsb_first=False, cs_active_low=True, max_hz=10_000_000):
        for pin in (cs, sck, data):
            _uint(pin, 29)
        if len({cs, sck, data}) != 3:
            raise ValueError("SPI pins must be distinct")
        self._request("spi_config", cs, sck, data, _uint(mode, 3), _bool(lsb_first), _bool(cs_active_low), _uint(max_hz, 10_000_000, 1))

    def transfer(self, bus, address, *, write=b"", read=0, timeout_us=10000):
        _target(bus, address)
        encoded = _bytes(write, 1024)
        _uint(read, 1024)
        _deadline(timeout_us)
        if not write and not read:
            raise ValueError("transfer must contain at least one byte")
        result = self._request("transfer", bus, address, timeout_us, encoded, read)
        data = result.get("data")
        if (not isinstance(data, str) or re.fullmatch(r"(?:[0-9a-f]{2})*", data) is None
                or len(data) != 2 * read or type(result.get("written")) is not int
                or type(result.get("read")) is not int or result["written"] != len(write) or result["read"] != read):
            self._invalid("short transfer or malformed transfer result")
        return bytes.fromhex(data)

    def write_register(self, bus, address, register, data, *, timeout_us=10000):
        _bytes(register, 1024)
        _bytes(data, 1024)
        self.transfer(bus, address, write=register + data, timeout_us=timeout_us)

    def read_register(self, bus, address, register, length=1, *, timeout_us=10000):
        _uint(length, 1024, 1)
        return self.transfer(bus, address, write=register, read=length, timeout_us=timeout_us)

    def start(self, steps, *, timeout_us=1_000_000, hold_final=True):
        """Stage and start; successful completion holds final states by default."""
        _deadline(timeout_us)
        hold = _bool(hold_final)
        commands = _compile_steps(steps)
        # Validate every request before clear: no partial staging for local errors.
        for args in commands:
            if len(("4294967295 add " + " ".join(map(str, args)) + "\n").encode("ascii")) > 4096:
                raise ValueError("step exceeds request limit")
        self._request("clear")
        for args in commands:
            self._request("add", *args)
        result = self._request("start", timeout_us, hold)
        if result.get("state") != "running":
            self._invalid("start did not acknowledge running state")
        self._active = True

    def status(self):
        result = self._request("status")
        if result.get("state") not in ("idle", "running", "done", "error", "aborted"):
            self._invalid("invalid run state")
        self._active = result["state"] == "running"
        return result

    def abort(self):
        result = self._request("abort")
        if result.get("state") not in ("idle", "done", "error", "aborted"):
            self._invalid("abort did not terminate run")
        self._active = False
        return result

    def _cancel(self):
        if self._synchronized and self._serial is not None:
            try:
                self.abort()
            except BaseException:
                self.close()
                raise
        else:
            self.close()

    def wait(self, *, timeout=None, poll_interval=0.01):
        _positive_seconds(poll_interval, zero=True)
        if timeout is not None:
            _positive_seconds(timeout, zero=True)
        deadline = None if timeout is None else time.monotonic() + timeout
        try:
            while True:
                if deadline is not None and time.monotonic() >= deadline:
                    self._cancel()
                    raise TimeoutError("host wait deadline expired; run aborted")
                result = self.status()
                if result["state"] != "running":
                    if result["state"] in ("error", "aborted"):
                        error = result.get("error")
                        if isinstance(error, dict):
                            raise DeviceError(error.get("code", "ABORTED"), error.get("step", result.get("step")), error.get("detail"))
                        raise DeviceError(error or "ABORTED", result.get("step"), result.get("detail"))
                    return result
                delay = poll_interval if deadline is None else min(poll_interval, max(0, deadline - time.monotonic()))
                time.sleep(delay)
        except KeyboardInterrupt:
            self._cancel()
            raise

    def run(self, steps, *, timeout_us=1_000_000, hold_final=True):
        try:
            self.start(steps, timeout_us=timeout_us, hold_final=hold_final)
            return self.wait()
        except KeyboardInterrupt:
            if self._active:
                self._cancel()
            raise

    def set_pin(self, pin, state, *, hold_final=True):
        """Run one state change on an already configured GPIO or PWM pin."""
        return self.run([SetPin(pin, state)], hold_final=hold_final)

    def bootloader(self):
        try:
            result = self._request("bootloader")
            if result.get("rebooting") is not True:
                self._invalid("invalid bootloader acknowledgment")
        except TransportError as error:
            raise BootloaderUncertainError("BOOTSEL acknowledgment unavailable; confirm USB enumeration") from error
        finally:
            self.close()
