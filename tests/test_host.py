from dataclasses import FrozenInstanceError
import json
import unittest
from unittest.mock import patch

import rp2040_glitch as api


def reply(request_id, result=None, error=None):
    envelope = {"id": request_id, "ok": error is None}
    envelope["result" if error is None else "error"] = result if error is None else error
    return json.dumps(envelope).encode() + b"\n"


class Port:
    """Scripted byte stream; replies are independent of transmitted commands."""
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.closed = False
        self.dtr = True

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def read_until(self, expected, size):
        if not self.chunks:
            return b""
        chunk = self.chunks.pop(0)
        if len(chunk) > size:
            self.chunks.insert(0, chunk[size:])
        return chunk[:size]

    def close(self):
        self.closed = True


class HostTests(unittest.TestCase):
    def device(self, *chunks):
        port = Port([reply(1, {"protocol": 3}), *chunks])
        with patch.object(api.serial, "Serial", return_value=port):
            device = api.Device("unused", timeout=0.01)
        self.addCleanup(device.close)
        return device, port

    def test_partial_reply_and_register_encoding(self):
        response = reply(2, {"data": "8042", "written": 1, "read": 2})
        device, port = self.device(response[:9], response[9:])
        self.assertEqual(device.read_register(1, 0x50, b"\x10", 2), b"\x80\x42")
        self.assertEqual(port.writes[-1], b"2 transfer 1 80 10000 10 2\n")

    def test_validation_has_no_io(self):
        device, port = self.device()
        invalid = [lambda: device.transfer(0, 0x50),
                   lambda: device.transfer(0, 0x50, write=b"x" * 1025),
                   lambda: device.configure_bus(1, sda=4, scl=5),
                   lambda: device.configure_bus(0, sda=4, scl=5, hz=9999),
                   lambda: device.configure_bus(0, sda=4, scl=5, hz=1500001),
                   lambda: device.start([api.Delay(8), api.Pulse(10, 7)]),
                   lambda: device.start([api.WaitSPI(b"a"), api.WaitSPI(b"b")]),
                   lambda: device.start([api.WaitGPIO(30)]),
                   lambda: device.start([api.WaitGPIO(-1)]),
                   lambda: device.start([api.WaitGPIO(21, rising=1)]),
                   lambda: device.start([api.WaitGPIO(21, timeout_us=0)]),
                   lambda: device.start([api.WaitGPIO(21), api.WaitI2C(0, 80, False)]),
                   lambda: device.start([api.Poll(0, 80, b"", 2, b"a", b"ab")]),
                   lambda: device.start([api.Delay(1)]),
                   lambda: device.start([api.Delay(7)]),
                   lambda: device.start([api.Delay(0xffffffff)] * 64),
                   lambda: device.start([api.Write(0, 80, b"x" * 256)] * 33),
                   lambda: device.start([api.Delay(8)] * 65),
                   lambda: device.start([api.Delay(8)], timeout_us=60_000_001)]
        for operation in invalid:
            with self.assertRaises(ValueError):
                operation()
        self.assertEqual(len(port.writes), 1)

    def test_scheduled_write_rejects_invalid_program_before_io(self):
        device, port = self.device()
        pulse = api.Pulse(8, 125000)
        first = api.WriteAt(1, 0x25, b"\x23\xd4\x00", 75000)
        second = api.WriteAt(1, 0x25, b"\x23\x01\x00", 100000)
        invalid = [
            [first],
            [pulse, first, api.Delay(8), second],
            [pulse, first, second, api.WriteAt(1, 0x25, b"x", 125000)],
            [pulse, second, first], [pulse, first, first],
            [pulse, first, api.WaitGPIO(21), second],
            [pulse, first, api.SetPin(9, "high"), second],
        ]
        for field, value in (("data", b""), ("data", b"x" * 17),
                             ("at_cycles", 7), ("at_cycles", True),
                             ("at_cycles", 0x100000000), ("timeout_us", 0),
                             ("bus", 2), ("address", 0x78)):
            args = dict(bus=1, address=0x25, data=b"x", at_cycles=75000)
            args[field] = value
            invalid.append([pulse, api.WriteAt(**args)])
        for program in invalid:
            with self.subTest(program=program), self.assertRaises(ValueError):
                device.start(program)
        self.assertEqual(len(port.writes), 1)

    def test_scheduled_deadline_failure_is_not_retried(self):
        device, port = self.device(reply(2, {
            "state": "error", "step": 2,
            "error": {"code": "MISSED_DEADLINE", "detail": 0}}))
        with self.assertRaises(api.DeviceError) as caught:
            device.wait(poll_interval=0)
        self.assertEqual((caught.exception.code, caught.exception.step),
                         ("MISSED_DEADLINE", 2))
        self.assertEqual(len(port.writes), 2)
        self.assertFalse(device._active)

    def test_id_mismatch_oversize_bad_json_close(self):
        for frame in (reply(99, {}), b"x" * 32769, b"not-json\n"):
            device, port = self.device(frame)
            with self.assertRaises(api.ProtocolError):
                device.status()
            self.assertTrue(port.closed)
            self.assertFalse(port.dtr)
            with self.assertRaises(api.TransportError):
                device.info()
            self.assertEqual(len(port.writes), 2)

    def test_protocol_three_accepted(self):
        device, port = self.device(reply(2, {"protocol": 3}))
        self.assertEqual(device.info()["protocol"], 3)
        self.assertEqual(port.writes, [b"1 info\n", b"2 info\n"])

    def test_version_rejection(self):
        for version in (1, 2, 4, True, 3.0, "3", None):
            with self.subTest(version=version):
                port = Port([reply(1, {"protocol": version})])
                with patch.object(api.serial, "Serial", return_value=port):
                    with self.assertRaises(api.ProtocolError):
                        api.Device("unused")
                self.assertTrue(port.closed)
                self.assertFalse(port.dtr)
                self.assertEqual(port.writes, [b"1 info\n"])

    def test_timeout_does_not_resend(self):
        device, port = self.device()
        with self.assertRaises(api.TransportError):
            device.transfer(0, 80, write=b"a")
        self.assertTrue(port.closed)
        self.assertFalse(port.dtr)
        self.assertEqual(len(port.writes), 2)

    def test_transfer_errors_are_distinct(self):
        for code in ("NACK", "ARBITRATION_LOST", "BUS_TIMEOUT"):
            device, port = self.device(reply(2, error={"code": code, "step": None, "detail": 4096}))
            with self.assertRaises(api.DeviceError) as caught:
                device.transfer(0, 80, write=b"x")
            self.assertEqual(caught.exception.code, code)
            self.assertEqual(caught.exception.detail, 4096)
            self.assertFalse(port.closed)
        device, port = self.device(reply(2, {"data": "", "written": 0, "read": 0}))
        with self.assertRaises(api.ProtocolError):
            device.transfer(0, 80, read=1)
        self.assertTrue(port.closed)

    def test_staging_failure_prevents_start(self):
        device, port = self.device(reply(2, {}), reply(3, error={"code": "RANGE", "step": None}))
        with self.assertRaises(api.DeviceError):
            device.start([api.Delay(8)])
        self.assertEqual(port.writes[-1], b"3 add delay 8\n")
        self.assertEqual(len(port.writes), 3)

    def test_wait_timeout_aborts_and_context_cancels(self):
        device, port = self.device(reply(2, {"state": "aborted"}))
        device._active = True
        with self.assertRaises(TimeoutError):
            device.wait(timeout=0)
        self.assertEqual(port.writes[-1], b"2 abort\n")
        device, port = self.device(reply(2, {"state": "aborted"}))
        with device:
            device._active = True
        self.assertEqual(port.writes[-1], b"2 abort\n")
        self.assertTrue(port.closed)

    def test_interrupt_between_requests_aborts(self):
        device, port = self.device(reply(2, {"state": "running"}), reply(3, {"state": "aborted"}))
        with patch.object(api.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                device.wait()
        self.assertEqual(port.writes[-1], b"3 abort\n")

    def test_interrupted_exchange_closes_without_abort_frame(self):
        device, port = self.device()
        device._active = True
        with patch.object(port, "read_until", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                device.wait()
        self.assertTrue(port.closed)
        self.assertFalse(port.dtr)
        self.assertEqual(port.writes[-1], b"2 status\n")

    def test_failed_cancellation_closes(self):
        device, port = self.device(reply(2, error={"code": "BUSY", "step": None}))
        with self.assertRaises(api.DeviceError):
            device.wait(timeout=0)
        self.assertTrue(port.closed)

    def test_wait_reports_terminal_error_and_result(self):
        device, port = self.device(
            reply(2, {"state": "running", "poll_attempts": 1}),
            reply(3, {"state": "done", "poll_attempts": 2, "poll_data": "80"}))
        self.assertEqual(device.wait(poll_interval=0)["poll_attempts"], 2)
        device, port = self.device(reply(2, {"state": "error", "step": 3,
                                                 "error": {"code": "WAIT_TIMEOUT", "detail": 7}}))
        with self.assertRaises(api.DeviceError) as caught:
            device.wait()
        self.assertEqual((caught.exception.code, caught.exception.step, caught.exception.detail),
                         ("WAIT_TIMEOUT", 3, 7))

    def test_partial_write_continues_without_duplicate_prefix(self):
        device, port = self.device(reply(2, {"state": "idle"}))
        transmitted = bytearray()
        def partial(data):
            transmitted.extend(data[:2])
            return min(2, len(data))
        with patch.object(port, "write", side_effect=partial):
            device.status()
        self.assertEqual(transmitted, b"2 status\n")

    def test_bootloader_uncertain_and_accepted(self):
        device, port = self.device()
        with self.assertRaises(api.BootloaderUncertainError):
            device.bootloader()
        self.assertTrue(port.closed)
        device, port = self.device(reply(2, {"rebooting": True}))
        device.bootloader()
        self.assertTrue(port.closed)

    def test_cycles_and_steps(self):
        self.assertEqual(api.seconds_to_cycles(0.5, clock_hz=5), 2)
        self.assertEqual(api.seconds_to_cycles(0.7, clock_hz=5), 4)
        for seconds, hz in ((float("nan"), 125e6), (-1, 1), (1, 0), (1, 2**32)):
            with self.assertRaises(ValueError):
                api.seconds_to_cycles(seconds, clock_hz=hz)
        commands = api._compile_steps([api.Pulse(0, 12500), api.Delay(2500), api.Pulse(29, 25),
                                     api.Write(1, 80, b"\x10\x42"), api.WaitI2C(0, 80, False, b"\x10\x42")])
        self.assertEqual(commands[:3], [("pulse", 0, 12500), ("delay", 2500), ("pulse", 29, 25)])
        self.assertEqual(commands[-1], ("wait_i2c", 0, 80, 0, "1042", 1000000))
        self.assertEqual(api._compile_steps([api.WaitSPI(b"\x80", 12)])[0], ("wait_spi", "80", 12))

    def test_pwm_bounds_and_block_validation_before_staging(self):
        device, port = self.device()
        for duty in (True, False, None, "60", complex(1), float("nan"),
                     float("inf"), -float("inf"), -0.01, 100.01, 10**1000):
            with self.subTest(duty=duty), self.assertRaises(ValueError):
                device.start([api.Delay(250), api.Pwm(duty)])
        for config in ({"pin": -1}, {"pin": 30}, {"pin": True},
                       {"pin": 2, "hz": 0}, {"pin": 2, "hz": 2**32},
                       {"pin": 2, "hz": True}, {"pin": 2, "hz": 500000.0}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                device.configure_pwm(**config)
        for wait in (api.WaitGPIO(21), api.WaitSPI(b"a"), api.WaitI2C(0, 80, False)):
            for steps in ([api.Pwm(60), api.Delay(250), wait],
                          [wait, api.Delay(250), api.Pwm(60)]):
                with self.assertRaises(ValueError):
                    device.start(steps)
        for steps in ([api.Pwm(60)] * 65,
                      [api.WaitGPIO(21), api.Write(0, 80, b"a"),
                       api.Pwm(60), api.Write(0, 80, b"a"), api.WaitGPIO(21)]):
            with self.assertRaises(ValueError):
                device.start(steps)
        self.assertEqual(port.writes, [b"1 info\n"])

    def test_gpio_bounds_and_electrical_modes_before_io(self):
        device, port = self.device()
        for pin in (-1, 30, True, 1.0, "1"):
            with self.subTest(pin=pin):
                with self.assertRaises(ValueError):
                    device.configure_gpio(pin=pin)
                with self.assertRaises(ValueError):
                    device.configure_sync(pin=pin)
                with self.assertRaises(ValueError):
                    device.start([api.Pulse(0, 8), api.Delay(8), api.Pulse(pin, 8)])
        for drive in ("", "open-drain", "PUSH_PULL", None, True, []):
            with self.subTest(drive=drive), self.assertRaises(ValueError):
                device.configure_gpio(pin=10, drive=drive)
        for idle in (-1, 2, True, False, 0.0, "0"):
            with self.subTest(idle=idle), self.assertRaises(ValueError):
                device.configure_gpio(pin=10, idle=idle)
        for width in (0, 7, -1, 2**32, True, 8.0):
            with self.subTest(width=width), self.assertRaises(ValueError):
                device.start([api.Delay(8), api.Pulse(29, width)])
        self.assertEqual(port.writes, [b"1 info\n"])


    def test_fifo_word_budget_and_unknown_drive_offset(self):
        span = 0x0fffffff + 6
        # 340 duration frames plus the final idle frame fit in 1024 words.
        steps = [api.Delay(span * 15)] * 22 + [api.Delay(span * 10)]
        self.assertEqual(len(api._compile_steps(steps)), 23)
        device, port = self.device()
        with self.assertRaises(ValueError):
            device.start(steps + [api.Delay(8)])
        self.assertEqual(port.writes, [b"1 info\n"])
        # A direction-driven pulse can absorb this extra cycle before its edge.
        # Firmware knows the drive mode and makes the final capacity decision.
        steps[-1] = api.Delay(span * 9 + 1)
        self.assertEqual(len(api._compile_steps(steps + [api.Pulse(29, 8)])), 24)

    def test_implicit_spacing_is_firmware_owned(self):
        for middle in ([], [api.Delay(0)], [api.WaitGPIO(21)],
                       [api.WaitSPI(b"a")], [api.WaitI2C(0, 80, False)],
                       [api.Pwm(60)]):
            steps = [api.Pulse(0, 8), *middle, api.Pulse(29, 8)]
            self.assertEqual(len(api._compile_steps(steps)), len(steps))
        for wait in (api.WaitGPIO(21), api.WaitSPI(b"a"), api.WaitI2C(0, 80, False)):
            for steps in ([api.SetPin(0, "high"), wait, api.SetPin(29, "floating")],
                          [wait, api.SetPin(0, "low"), api.Pulse(29, 8)]):
                self.assertEqual(len(api._compile_steps(steps)), len(steps))

    def test_set_pin_and_hold_validation_before_io(self):
        device, port = self.device()
        for pin in (-1, 30, True, 0.0, "0", None):
            with self.subTest(pin=pin), self.assertRaises(ValueError):
                device.set_pin(pin, "low")
        for state in ("", "LOW", "float", "high\n", 0, True, None, []):
            with self.subTest(state=state), self.assertRaises(ValueError):
                device.set_pin(0, state)
        for hold in (0, 1, None, "true", [], 1.0):
            for operation in (
                    lambda: device.start([api.SetPin(0, "high")], hold_final=hold),
                    lambda: device.run([api.SetPin(0, "high")], hold_final=hold),
                    lambda: device.set_pin(0, "high", hold_final=hold)):
                with self.subTest(hold=hold), self.assertRaises(ValueError):
                    operation()
        self.assertEqual(port.writes, [b"1 info\n"])
        step = api.SetPin(29, "floating")
        with self.assertRaises(FrozenInstanceError):
            step.state = "high"
        self.assertEqual(api._compile_steps([
            api.SetPin(0, "low"), api.SetPin(29, "high"), step]),
            [("set", 0, "low"), ("set", 29, "high"), ("set", 29, "floating")])

    def test_set_pin_capacity_defers_unknown_state_and_pin_role(self):
        span = 0x0fffffff + 6
        steps = [api.Delay(span * 15)] * 22 + [api.Delay(span * 10)]
        # Redundant GPIO states or same-time PWM modes need not add FIFO words.
        self.assertEqual(len(api._compile_steps(
            [api.SetPin(29, "low"), *steps, api.SetPin(29, "low")])), 25)
        device, port = self.device()
        with self.assertRaises(ValueError):
            device.start([api.SetPin(29, "floating"), *steps, api.Delay(8)])
        self.assertEqual(port.writes, [b"1 info\n"])

    def test_set_pin_interrupt_aborts_and_failed_start_stays_inactive(self):
        device, port = self.device(
            reply(2, {}), reply(3, {}), reply(4, {"state": "running"}),
            reply(5, {"state": "running"}), reply(6, {"state": "aborted"}))
        with patch.object(api.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                device.set_pin(29, "floating", hold_final=False)
        self.assertEqual(port.writes[-1], b"6 abort\n")
        self.assertFalse(device._active)
        self.assertFalse(port.closed)
        device, port = self.device(
            reply(2, {}), reply(3, {}), reply(4, error={"code": "PIN_CONFLICT"}))
        with self.assertRaises(api.DeviceError):
            device.set_pin(29, "high")
        self.assertFalse(device._active)
        self.assertFalse(port.closed)
        self.assertEqual(len(port.writes), 4)

    def test_pwm_percentage_quantization(self):
        commands = api._compile_steps([api.Pwm(12.345), api.Pwm(12.355)])
        self.assertEqual(commands, [("pwm", 1234), ("pwm", 1236)])

    def test_pwm_success_keeps_connection_until_close(self):
        device, port = self.device(
            reply(2, {}), reply(3, {}), reply(4, {}), reply(5, {}),
            reply(6, {"state": "running"}), reply(7, {"state": "done"}))
        with device:
            device.configure_pwm(pin=22)
            device.start([api.Pwm(60), api.Pwm(12.345)])
            self.assertEqual(device.wait()["state"], "done")
            self.assertTrue(port.dtr)
            self.assertFalse(port.closed)
        self.assertFalse(port.dtr)
        self.assertTrue(port.closed)



if __name__ == "__main__":
    unittest.main()
