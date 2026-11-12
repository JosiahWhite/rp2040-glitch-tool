"""Execute assembled RP2040 instructions and the production C timing compiler.

Requires build/timing.pio.json from the firmware build, not source-text matching.
PIO_JSON can select an alternate build. This is a digital instruction trace model,
not evidence of electrical timing, metastability, or shared-bus arbitration.
"""
import ctypes
from collections import deque
import json
import math
from itertools import product
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MASK = 0xFFFFFFFF
DELAY, PULSE, WAIT, PWM, SET = range(5)
LOW, HIGH, FLOATING = range(3)
CARRIER, PWM_LOW, PWM_HIGH, PWM_FLOATING = range(4)
PUSH_PULL, OPEN_DRAIN, OPEN_SOURCE = range(3)


class Gpio(ctypes.Structure):
    _fields_ = [("configured_mask", ctypes.c_uint32), ("open_drain_mask", ctypes.c_uint32),
                ("open_source_mask", ctypes.c_uint32), ("idle_high_mask", ctypes.c_uint32),
                ("out_base", ctypes.c_uint8), ("current_levels", ctypes.c_uint32),
                ("current_dirs", ctypes.c_uint32)]


def gpio_config(*pins, pwm_pin=None):
    gpio = Gpio()
    for pin, drive, idle in pins:
        gpio.configured_mask |= 1 << pin
        gpio.open_drain_mask |= (drive == OPEN_DRAIN) << pin
        gpio.open_source_mask |= (drive == OPEN_SOURCE) << pin
        gpio.idle_high_mask |= idle << pin
    gpio.out_base = 0 if pwm_pin is None else (pwm_pin + 1) & 31
    gpio.current_levels, gpio.current_dirs = idle_state(gpio)
    return gpio


def idle_state(gpio):
    levels = (gpio.idle_high_mask & ~(gpio.open_drain_mask | gpio.open_source_mask)) | gpio.open_source_mask
    dirs = gpio.configured_mask & ~(gpio.open_drain_mask & gpio.idle_high_mask) & \
        ~(gpio.open_source_mask & ~gpio.idle_high_mask)
    return levels, dirs


def fixture_gpio(*, pwm_pin=None):
    return gpio_config((10, OPEN_DRAIN, 1), (11, PUSH_PULL, 0), pwm_pin=pwm_pin)


class Pwm(ctypes.Structure):
    _fields_ = [("configured", ctypes.c_bool), ("active", ctypes.c_bool),
                ("pin", ctypes.c_uint8), ("hz", ctypes.c_uint32),
                ("period_cycles", ctypes.c_uint32), ("final_duty_basis_points", ctypes.c_uint32),
                ("final_mode", ctypes.c_int)]


class Instruction(ctypes.Structure):
    _fields_ = [("op", ctypes.c_int), ("cycles", ctypes.c_uint32),
                ("duty_basis_points", ctypes.c_uint32), ("pin", ctypes.c_uint8), ("state", ctypes.c_int)]


class Segment(ctypes.Structure):
    _fields_ = [("words", ctypes.c_uint32 * 8), ("periods", ctypes.c_uint32),
                ("word_count", ctypes.c_uint32), ("mode", ctypes.c_int),
                ("duty_basis_points", ctypes.c_uint32), ("padding", ctypes.c_uint32 * 4)]


class Block(ctypes.Structure):
    _fields_ = [("tokens", ctypes.POINTER(ctypes.c_uint32)), ("count", ctypes.c_size_t),
                ("waits", ctypes.c_bool), ("programmed_cycles", ctypes.c_uint64),
                ("pwm_segments", ctypes.POINTER(Segment)), ("pwm_count", ctypes.c_size_t),
                ("pwm_final_duty", ctypes.c_uint32), ("wave_start_countdown", ctypes.c_uint32),
                ("inserted_cycles", ctypes.c_uint64), ("final_levels", ctypes.c_uint32),
                ("final_dirs", ctypes.c_uint32), ("pwm_final_mode", ctypes.c_int),
                ("completion_cycles", ctypes.c_uint64),
                ("pwm_descriptor", ctypes.c_uint16), ("pwm_tail_descriptor", ctypes.c_uint16),
                ("pwm_prefill", ctypes.c_uint32 * 8)]


class SM:
    """One enabled divider-one SM, using only instructions emitted by timing.pio."""
    def __init__(self, world, program, *, in_base=0, out_base=0, set_base=0,
                 jmp_pin=0, fifo_capacity=4, out_count=0, autopull=False, pull_threshold=32,
                 output_mask=MASK, sideset_pindirs=False):
        self.world = world
        self.origin = program["origin"]
        self.code = [int(i["hex"], 16) for i in program["instructions"]]
        # SDK pio_add_program relocates JMP targets, including fixed-origin programs.
        self.code = [word + self.origin if word >> 13 == 0 else word for word in self.code]
        self.wrap = self.origin + program["wrap"]
        self.wrap_target = self.origin + program["wrapTarget"]
        self.pc = self.origin
        self.in_base, self.out_base, self.set_base = in_base, out_base, set_base
        self.jmp_pin = jmp_pin
        self.out_count, self.output_mask = out_count, output_mask
        self.sideset_pindirs = sideset_pindirs
        self.autopull, self.pull_threshold = autopull, pull_threshold
        self.osr_count = 32
        self.x = self.y = self.isr = self.osr = 0
        self.tx, self.rx = deque(), deque()
        self.fifo_capacity = fifo_capacity
        self.tx_stall = self.rx_stall = False
        self.enabled = True
        self.forced = None
        self.forced_system = False
        self.delay = 0
        self.samples = []
        self.wait_edges = []
        world.sms.append(self)

    def load_program(self, program):
        """Install a second assembled program in this SM's instruction memory."""
        start = program["origin"] - self.origin
        words = [int(i["hex"], 16) for i in program["instructions"]]
        words = [word + program["origin"] if word >> 13 == 0 else word for word in words]
        self.code.extend([0] * max(0, start + len(words) - len(self.code)))
        self.code[start:start + len(words)] = words

    def source(self, index):
        return {0: self.world.synced >> self.in_base, 1: self.x, 2: self.y,
                3: 0, 6: self.isr, 7: self.osr}[index]

    def destination(self, index, value):
        value &= MASK
        if index == 1:
            self.x = value
        elif index == 2:
            self.y = value
        elif index == 6:
            self.isr = value
        elif index == 7:
            self.osr = value
            self.osr_count = 0
        else:
            raise AssertionError(f"unsupported MOV destination {index}")

    def prefetch(self):
        if self.autopull and self.osr_count >= self.pull_threshold and self.tx:
            self.osr = self.tx.popleft()
            self.osr_count = 0

    def exec_instruction(self, instruction):
        """SMx_INSTR replaces a stalled forced instruction, even while disabled."""
        self.forced = instruction
        self.forced_system = True

    def step(self):
        if not self.enabled and not self.forced_system:
            return
        if self.delay and not self.forced_system:
            self.prefetch()
            self.delay -= 1
            return
        forced = self.forced
        instruction = forced if forced is not None else self.code[self.pc - self.origin]
        opcode, argument = instruction >> 13, instruction & 0xFF
        delay = 0 if self.forced_system else (instruction >> 8) & (7 if self.sideset_pindirs else 31)
        if self.sideset_pindirs and instruction & 0x1000:
            self.world.drive(self.set_base, 1, bool(instruction & 0x0800), True)
        next_pc = self.pc if forced is not None else \
            self.wrap_target if self.pc == self.wrap else self.pc + 1
        # RP2040 datasheet 3.5.4.2: non-OUT cycles prefetch, except a write to
        # OSR takes precedence. A NOP (MOV Y,Y) does not reset the OSR counter.
        # https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
        if opcode != 3 and not (opcode == 4 and argument & 128) and \
                not (opcode == 5 and argument >> 5 == 7):
            self.prefetch()
        if opcode == 0:  # JMP; a forced JMP replaces a previously stalled WAIT.
            condition, target = argument >> 5, argument & 31
            taken = {0: True, 1: self.x == 0, 2: self.x != 0,
                     3: self.y == 0, 4: self.y != 0, 5: self.x != self.y,
                     6: bool(self.world.synced & (1 << self.jmp_pin))}.get(condition)
            assert taken is not None
            if condition == 2:
                self.x = (self.x - 1) & MASK
            elif condition == 4:
                self.y = (self.y - 1) & MASK
            if taken:
                next_pc = target
        elif opcode == 1:  # WAIT GPIO / input-relative PIN.
            source, pin = (argument >> 5) & 3, argument & 31
            assert source in (0, 1)
            if source == 1:
                pin = (pin + self.in_base) & 31
            if bool(self.world.synced & (1 << pin)) != bool(argument & 128):
                return
            self.wait_edges.append((self.world.cycle, pin))
        elif opcode == 2:  # IN, left-shifting, no autopush.
            count = (argument & 31) or 32
            self.isr = ((self.isr << count) | (self.source(argument >> 5) & ((1 << count) - 1))) & MASK
            self.samples.append((self.world.cycle, self.isr))
        elif opcode == 3:  # OUT bit count shifts OSR; OUT_COUNT independently maps pins.
            count, destination = (argument & 31) or 32, argument >> 5
            if self.autopull and self.osr_count >= self.pull_threshold:
                # Empty OUT pulls but ALWAYS stalls that cycle, even with FIFO data.
                self.prefetch()
                self.tx_stall = True
                return
            value = self.osr & ((1 << count) - 1)
            self.osr >>= count
            self.osr_count = min(32, self.osr_count + count)
            if destination == 0:
                self.world.drive(self.out_base, self.out_count, value, False, self.output_mask)
            elif destination == 4:
                self.world.drive(self.out_base, self.out_count, value, True, self.output_mask)
            elif destination == 1:
                self.x = value
            elif destination == 2:
                self.y = value
            elif destination == 7:
                self.forced = value & 0xFFFF  # OUT EXEC executes on the following cycle.
                self.forced_system = False
            else:
                raise AssertionError(f"unsupported OUT destination {destination}")
            self.prefetch()
        elif opcode == 4:  # PUSH/PULL, including the nonblocking readiness cutoff.
            assert not argument & 64
            if argument & 128:
                if self.autopull and self.osr_count < self.pull_threshold:
                    self.pc, self.delay = next_pc, delay
                    self.forced = None
                    self.forced_system = False
                    return  # Explicit PULL is a barrier/no-op while autopull OSR is full.
                if not self.tx and argument & 32:
                    self.tx_stall = True
                    return
                self.osr = self.tx.popleft() if self.tx else self.x
                self.osr_count = 0
            else:
                if len(self.rx) == self.fifo_capacity:
                    self.rx_stall = True
                    return
                self.rx.append(self.isr)
                self.isr = 0
        elif opcode == 5:  # MOV, with no reverse/invert used by these programs.
            assert not argument & 24
            self.destination(argument >> 5, self.source(argument & 7))
        elif opcode == 7:  # SET pins or scratch register.
            destination, value = argument >> 5, argument & 31
            if destination == 0:
                self.world.drive(self.set_base, 1, value, False)
            else:
                self.destination(destination, value)
        else:
            raise AssertionError(f"unsupported instruction 0x{instruction:04x}")
        if forced is not None:
            self.forced = None
            self.forced_system = False
        self.pc = next_pc
        self.delay = delay


class PwmDMA:
    """RP2040 T ring reader + four-write AL3 control descriptor model.

    One bus word per tick and four control writes expose descriptor crossings;
    this checks scheduling/headroom, not real shared-bus arbitration latency.
    """
    def __init__(self, sm, segments):
        self.sm = sm
        self.tail = len(segments) - 1
        self.retained = list(segments[-1].words[:segments[-1].word_count])
        self.descriptors = [(s.words if i != self.tail else self.retained,
                             s.word_count, s.periods * s.word_count)
                            for i, s in enumerate(segments)]
        self.descriptors.append(self.descriptors[-1])
        self.next_descriptor = 0
        self.control_remaining = 4
        self.current = -1
        self.remaining = self.offset = 0
        self.enabled = True
        self.boundaries = 0

    @property
    def tail_owned(self):
        return self.current >= self.tail and not self.control_remaining

    def tick(self):
        if not self.enabled:
            return
        if self.control_remaining:
            self.control_remaining -= 1
            if not self.control_remaining:
                self.current = self.next_descriptor
                self.next_descriptor += 1
                self.packet, self.word_count, self.remaining = self.descriptors[self.current]
                self.offset = 0
                self.boundaries += 1
            return
        if self.remaining and len(self.sm.tx) < 8:
            self.sm.tx.append(self.packet[self.offset])
            self.offset = (self.offset + 1) & (self.word_count - 1)
            self.remaining -= 1
            if not self.remaining:
                self.control_remaining = 4

    def service(self):
        if self.tail_owned:
            self.next_descriptor = self.tail


class World:
    def __init__(self, programs):
        self.programs = programs
        self.cycle = 0
        self.external = 0
        self.levels = 0
        self.directions = 0
        self.synced = 0
        self.synchronizers = deque([0, 0])
        self.sms = []
        self.edges = []
        self.injections = []
        self.boundary = self.matcher = self.wave = None
        self.pwm = None
        self.pwm_dma_enabled = True
        self.pwm_dma = None
        self.pwm_final_mode = CARRIER
        self.pwm_error = None
        self.boundary_delay = 4
        self.boundary_budget = MASK
        self.wave_dma_enabled = True
        self.tokens = deque()
        self.snapshot = False
        self.gpio = Gpio()
        self.sync_pin = None
        self.pwm_pin = None

    def physical(self):
        return (self.external & ~self.directions) | (self.levels & self.directions)

    def drive(self, base, count, value, directions, output_mask=MASK):
        # OUT mappings wrap modulo 32, even when OUT_COUNT < instruction bit count.
        window = (1 << count) - 1
        value &= window
        mask = ((window << base) | (window >> (32 - base))) & MASK & output_mask
        value = ((value << base) | (value >> (32 - base))) & mask
        previous = self.directions if directions else self.levels
        changed = (previous ^ value) & mask
        if directions:
            self.directions = (previous & ~mask) | value
        else:
            self.levels = (previous & ~mask) | value
        for pin in range(32):
            if changed & (1 << pin):
                self.edges.append((self.cycle, "direction" if directions else "level", pin,
                                   bool(value & (1 << pin))))

    def set_pin(self, pin, value):
        self.external = (self.external & ~(1 << pin)) | (int(value) << pin)

    def tick(self, cycles=1):
        for _ in range(cycles):
            self.synced = self.synchronizers.popleft()
            self.synchronizers.append(self.physical())
            for due, instruction in self.injections[:]:
                if due <= self.cycle:
                    self.matcher.exec_instruction(instruction)
                    self.injections.remove((due, instruction))
            for sm in self.sms:
                sm.step()
            if self.boundary and self.boundary.rx and self.boundary_budget:
                self.injections.append((self.cycle + self.boundary_delay, self.boundary.rx.popleft()))
                self.boundary_budget -= 1
            if self.wave_dma_enabled and self.wave and self.tokens and len(self.wave.tx) < 8:
                self.wave.tx.append(self.tokens.popleft())
                if not self.tokens:
                    # Chained DMA snapshots FDEBUG after the final FIFO write.
                    self.snapshot = self.wave.tx_stall
            if self.pwm_dma_enabled and self.pwm_dma:
                self.pwm_dma.tick()
            self.cycle += 1

    def advance_hold(self, count):
        """Execute the encoded decrement loop analytically for u32-length tests."""
        while count:
            sm = self.wave
            word = sm.code[sm.pc - sm.origin]
            if word >> 13 == 0 and (word & 0xE0) == 0x40 and (word & 31) == sm.pc and sm.x:
                skip = min(count, sm.x)
                sm.x -= skip
                self.cycle += skip
                count -= skip
            else:
                self.tick()
                count -= 1

    def waveform(self, tokens, gpio=None, pwm_pin=None):
        self.gpio = fixture_gpio() if gpio is None else gpio
        if pwm_pin is not None:
            self.pwm_pin = pwm_pin
        levels, dirs = self.gpio.current_levels, self.gpio.current_dirs
        self.levels = (self.levels & ~self.gpio.configured_mask) | levels
        self.directions = (self.directions & ~self.gpio.configured_mask) | dirs
        pio0_mask = self.gpio.configured_mask | (0 if self.pwm_pin is None else 1 << self.pwm_pin)
        self.wave = SM(self, self.programs["waveform"], out_base=self.gpio.out_base,
                       out_count=30 if self.pwm_pin is None else 31, autopull=True,
                       output_mask=pio0_mask, fifo_capacity=8)
        self.wave.code[6] = 0x2000 | 128 | 12
        self.tokens = deque(tokens[8:])
        self.wave.tx.extend(tokens[:8])
        return self.wave

    def pwm_waveform(self, tokens, segments, warmup, gpio=None, pin=13, initial_mode=PWM_LOW):
        self.waveform(tokens, fixture_gpio(pwm_pin=pin) if gpio is None else gpio, pin)
        self.wave.osr, self.wave.osr_count = warmup, 0
        self.wave.pc = self.wave.origin + 3
        self.pwm = SM(self, self.programs["pwm_sequence"], set_base=pin, autopull=True,
                      fifo_capacity=8, output_mask=1 << pin, sideset_pindirs=True)
        if initial_mode == PWM_FLOATING:
            self.directions &= ~(1 << pin)
        else:
            self.directions |= 1 << pin
        if initial_mode == PWM_HIGH:
            self.levels |= 1 << pin
        self.pwm_dma = PwmDMA(self.pwm, segments)
        while len(self.pwm.tx) < 8:
            self.pwm_dma.tick()  # Disabled-SM prefill is entirely DMA-owned.
        self.pwm_final_mode = segments[-1].mode
        self.pwm_error = None

    def advance_pwm(self, cycles):
        """Skip only simultaneous assembled X-- self-loops; execute all edges."""
        assert set(self.sms) == {self.wave, self.pwm}
        end = self.cycle + cycles
        while self.cycle < end:
            skip, decrement = end - self.cycle, []
            for sm in self.sms:
                if not sm.enabled:
                    continue
                word = sm.code[sm.pc - sm.origin]
                if sm.delay or sm.forced is not None or (self.pwm_dma_enabled and self.pwm_dma and
                    (self.pwm_dma.control_remaining or len(self.pwm.tx) < 8)):
                    skip = 0
                    break
                if word >> 13 == 0 and (word & 0xE0) == 0x40 and (word & 31) == sm.pc and sm.x:
                    skip = min(skip, sm.x)
                    decrement.append(sm)
                elif sm.tx_stall and not sm.tx and sm is self.wave and not self.tokens:
                    continue
                else:
                    skip = 0
                    break
            if skip:
                for sm in decrement:
                    sm.x -= skip
                self.cycle += skip
            else:
                self.tick()

    def service_pwm(self):
        if not self.pwm or not self.pwm.enabled:
            return
        if self.pwm.tx_stall:
            self.shutdown()
            self.pwm_error = "TIMING_ERROR"
        else:
            self.pwm_dma.service()
            if self.wave_complete and self.pwm_final_mode != CARRIER:
                self.pwm.enabled = self.pwm_dma.enabled = False
                self.pwm.tx.clear()

    @property
    def wave_complete(self):
        # Completion protects all staged packets, not just the waveform source.
        return self.wave.tx_stall and self.wave.pc == self.wave.origin + 1 and \
            not self.wave.tx and not self.tokens and not self.snapshot and \
            (self.pwm_dma is None or self.pwm_dma.tail_owned)

    def trigger(self, kind, expected, bit_count, *, bus=0, mode=0, active_low=True,
                initially_selected=False):
        self.sync_pin = 12
        self.directions |= 1 << self.sync_pin
        if kind == "i2c":
            sda = 4 + bus * 2
            self.set_pin(sda, 1)
            self.set_pin(sda + 1, 1)
            # Input synchronizers run even while state machines are disabled.
            self.tick(4)
            self.matcher = SM(self, self.programs["i2c_matcher"], in_base=sda, set_base=12)
            self.boundary = SM(self, self.programs["i2c_boundary"], in_base=sda, jmp_pin=sda + 1)
            self.boundary.x = 1  # Encoded JMP matcher_start.
            self.boundary.y = 0  # Encoded JMP matcher_idle.
        else:
            self.set_pin(18, not active_low if initially_selected else active_low)
            self.set_pin(19, bool(mode & 2))
            # The fixture is already selected before arm, not an assertion
            # coincident with enabling an SM and a freshly zeroed synchronizer.
            self.tick(4)
            self.matcher = SM(self, self.programs["spi_matcher"], in_base=20, set_base=12)
            self.boundary = SM(self, self.programs["spi_boundary"])
            sample_high = (mode & 1) == (mode >> 1)
            for index, pin, high in ((0, 18, active_low), (1, 18, not active_low),
                                     (4, 19, not sample_high), (5, 19, sample_high)):
                self.matcher.code[index] = 0x2000 | (int(high) << 7) | pin
            self.boundary.code[0] = 0x2000 | (int(not active_low) << 7) | 18
            self.boundary.code[1] = 0x2000 | (int(active_low) << 7) | 18
            self.boundary.x = 0
        self.matcher.y = expected
        self.matcher.osr = bit_count - 1
        self.tick(32)

    def gpio_trigger(self, pin, rising, initial):
        self.sync_pin = 12
        self.directions |= 1 << self.sync_pin
        self.set_pin(pin, initial)
        self.tick(4)  # Synchronizers keep running while the matcher is disabled.
        self.matcher = SM(self, self.programs["gpio_matcher"], set_base=12)
        self.matcher.code[0] = 0x2000 | (int(not rising) << 7) | pin
        self.matcher.code[1] = 0x2000 | (int(rising) << 7) | pin
        self.tick(16)

    def shutdown(self):
        # Model the ordered stop contract: cancel hardware before changing pads.
        for sm in (self.boundary, self.matcher, self.wave, self.pwm):
            if sm:
                sm.enabled = False
                sm.tx.clear()
                sm.rx.clear()
        self.injections.clear()
        self.tokens.clear()
        levels, dirs = idle_state(self.gpio)
        self.drive(0, 32, levels, False, self.gpio.configured_mask)
        self.drive(0, 32, dirs, True, self.gpio.configured_mask)
        if self.sync_pin is not None:
            self.drive(self.sync_pin, 1, 0, False)
        if self.pwm_dma:
            self.pwm_dma.enabled = False
        self.pwm_error = None
        if self.pwm:
            self.drive(self.pwm_pin, 1, 0, False)
            self.drive(self.pwm_pin, 1, 1, True)

    @property
    def matched(self):
        return bool(self.levels & (1 << 12))


def bits_of(data, lsb=False):
    return [(byte >> bit) & 1 for byte in data for bit in (range(8) if lsb else range(7, -1, -1))]


def i2c_start(world, bus=0):
    sda = 4 + bus * 2
    world.set_pin(sda, 1)
    world.set_pin(sda + 1, 1)
    world.tick(32)
    world.set_pin(sda, 0)
    world.tick(32)


def i2c_bits(world, bits, bus=0, stretch=0):
    sda = 4 + bus * 2
    for bit in bits:
        world.set_pin(sda + 1, 0)
        world.tick(16)
        world.set_pin(sda, bit)
        world.tick(140 + stretch)
        world.set_pin(sda + 1, 1)
        world.tick(157)


def i2c_boundary(world, restart, bus=0):
    # SCL remains high. Depending on the last data bit, the first transition
    # may itself be a boundary; neither helper invents an extra sampling edge.
    sda = 4 + bus * 2
    world.set_pin(sda, not restart)
    world.tick(32)
    world.set_pin(sda, restart)
    world.tick(32)
    world.set_pin(sda, not restart)
    world.tick(32)


def spi_bits(world, bits, mode, *, select=True, deselect=True, active_low=True):
    cpol, cpha = bool(mode & 2), bool(mode & 1)
    world.set_pin(19, cpol)
    if select:
        world.set_pin(18, not active_low)
    # 125 MHz / 10 MHz: 6.25 cycles each half-period. Integer sampling of
    # fractional physical edges intentionally alternates 6/7-cycle waits.
    # First relevant sampling edge is exactly eight cycles after CS assertion,
    # including CPHA=1's earlier leading edge.
    when = float(world.cycle) + (8 - (6.25 if cpha else 0) if select else 6.25)
    for bit in bits:
        if not cpha:
            world.set_pin(20, bit)
        world.tick(math.ceil(when) - world.cycle)
        if cpha:
            world.set_pin(20, bit)
        world.set_pin(19, not cpol)
        when += 6.25
        world.tick(math.ceil(when) - world.cycle)
        world.set_pin(19, cpol)
        when += 6.25
    world.tick(8)
    if deselect:
        world.set_pin(18, active_low)
        world.tick(64)


class PioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        library = Path(cls.temporary.name) / "timing.so"
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-shared", "-fPIC", str(ROOT / "timing.c"),
                        "-o", str(library)], check=True)
        cls.native = ctypes.CDLL(str(library))
        cls.native.timing_compile.restype = ctypes.c_char_p
        cls.native.timing_compile.argtypes = [ctypes.POINTER(Gpio), ctypes.POINTER(Instruction),
            ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t, ctypes.POINTER(Block)]
        cls.native.timing_compile_pwm.restype = ctypes.c_char_p
        cls.native.timing_compile_pwm.argtypes = [ctypes.POINTER(Gpio), ctypes.POINTER(Pwm),
            ctypes.POINTER(Instruction), ctypes.c_size_t, ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t, ctypes.POINTER(Segment),
            ctypes.c_size_t, ctypes.POINTER(Block)]
        cls.native.timing_pwm_parameters.restype = cls.native.timing_pwm_encode.restype = ctypes.c_char_p
        cls.native.timing_pwm_parameters.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
                                                    ctypes.POINTER(ctypes.c_uint32)]
        cls.native.timing_pwm_encode.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
        cls.native.timing_pack_i2c.restype = cls.native.timing_pack_spi.restype = ctypes.c_char_p
        cls.native.timing_pack_i2c.argtypes = [ctypes.c_uint8, ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
        cls.native.timing_pack_spi.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.c_bool,
            ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
        assembled = Path(os.environ.get("PIO_JSON", str(ROOT / "build/timing.pio.json")))
        if not assembled.exists():
            raise RuntimeError("Build firmware first: assembled build/timing.pio.json is required")
        cls.programs = {p["name"]: p for p in json.loads(assembled.read_text())["programs"]}

    def compile(self, steps, *, capacity=1024, gpio=None, error=None):
        gpio = fixture_gpio() if gpio is None else gpio
        instructions = (Instruction * len(steps))(*(Instruction(*s) for s in steps))
        tokens = (ctypes.c_uint32 * capacity)()
        block = Block()
        result = self.native.timing_compile(ctypes.byref(gpio), instructions, len(steps),
                                            tokens, capacity, ctypes.byref(block))
        self.assertEqual(result, error)
        self.last_block = block
        return list(tokens[:block.count]) if not result else []

    def compile_pwm(self, steps, *, period=250, initial=0, initial_mode=CARRIER, capacity=64, token_capacity=1024,
                    configured=True, gpio=None, pin=13, error=None):
        gpio = fixture_gpio(pwm_pin=pin) if gpio is None else gpio
        pwm = Pwm(configured, False, pin, 500000, period, initial, initial_mode)
        instructions = (Instruction * len(steps))(*(Instruction(*s) for s in steps))
        tokens = (ctypes.c_uint32 * token_capacity)()
        words = (Segment * capacity)()
        block = Block()
        result = self.native.timing_compile_pwm(ctypes.byref(gpio), ctypes.byref(pwm),
            instructions, len(steps), initial, tokens, token_capacity, words, capacity, ctypes.byref(block))
        self.assertEqual(result, error)
        return list(tokens[:block.count]), list(words[:block.pwm_count]), block

    def pwm_world(self, steps, **kwargs):
        tokens, words, block = self.compile_pwm(steps, **kwargs)
        world = World(self.programs)
        world.pwm_waveform(tokens, words, block.wave_start_countdown,
                           kwargs.get("gpio"), kwargs.get("pin", 13), kwargs.get("initial_mode", PWM_LOW))
        return world

    def pack(self, kind, prefix, lsb=False, address=0x50, read=False):
        payload = (ctypes.c_uint8 * len(prefix))(*prefix)
        expected, count = ctypes.c_uint32(), ctypes.c_uint32()
        if kind == "i2c":
            result = self.native.timing_pack_i2c(address, read, payload, len(prefix),
                                                ctypes.byref(expected), ctypes.byref(count))
        else:
            result = self.native.timing_pack_spi(payload, len(prefix), lsb,
                                                ctypes.byref(expected), ctypes.byref(count))
        self.assertIsNone(result)
        return expected.value, count.value

    def world(self, steps, *, gpio=None):
        world = World(self.programs)
        world.waveform(self.compile(steps, gpio=gpio), gpio)
        return world

    def assert_pulse(self, world, width):
        edges = [cycle for cycle, kind, pin, _ in world.edges if kind == "level" and pin == 11]
        self.assertEqual(len(edges), 2, world.edges)
        self.assertEqual(edges[1] - edges[0], width)

    def test_production_timing_runtime(self):
        directory = Path(self.temporary.name)
        declarations = []
        for name, program in self.programs.items():
            instructions = ",".join("0x" + i["hex"] for i in program["instructions"])
            declarations += [
                f"static const uint16_t {name}_instructions[] = {{{instructions}}};",
                f"static const pio_program_t {name}_program = {{"
                f".instructions={name}_instructions,.length={len(program['instructions'])},"
                f".origin={program['origin']},.wrap={program['wrap']},"
                f".wrap_target={program['wrapTarget']},.sideset={int(bool(program['sideset']))}" "};",
                f"static inline pio_sm_config {name}_program_get_default_config(uint offset) {{"
                f"return (pio_sm_config){{.wrap=offset+{program['wrap']},"
                f".wrap_target=offset+{program['wrapTarget']},.sideset={int(bool(program['sideset']))}" "};}"]
            declarations += [f"#define {name}_offset_{label} {offset}u"
                             for label, offset in program["publicLabels"].items()]
        (directory / "timing_test_programs.h").write_text("\n".join(declarations) + "\n")
        executable = directory / "timing-runtime"
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fno-pie", "-no-pie", "-I", str(directory),
                        str(ROOT / "tests/timing_runtime_harness.c"), str(ROOT / "timing.c"),
                        "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


    def scheduled_world(self, times, rising):
        world = World(self.programs)
        world.set_pin(11, rising)
        world.tick(4)
        engines = []
        for at in times:
            sm = SM(world, self.programs["scheduled_write"])
            sm.code[0] = 0x2000 | (int(rising) << 7) | 11
            sm.x = at - 7
            sm.exec_instruction(0x2000 | (int(not rising) << 7) | 11)
            engines.append(sm)
        world.tick(8)
        self.assertTrue(all(sm.pc == sm.origin and sm.forced is not None and not sm.rx for sm in engines))
        world.set_pin(11, not rising)
        world.tick(8)
        self.assertTrue(all(sm.pc == sm.origin and sm.forced is None for sm in engines))
        world.set_pin(11, rising)
        reference = world.cycle + 2  # Two-stage input synchronizer.
        return world, engines, reference

    def test_scheduled_cutoff_and_one_shot(self):
        program = self.programs["scheduled_write"]
        for rising in (False, True):
            for at in (8, 9, 40, 127):
                for relative in (-1, 0, 1):
                    with self.subTest(rising=rising, at=at, relative=relative):
                        world, (sm,), reference = self.scheduled_world([at], rising)
                        cutoff = reference + at - 4
                        world.tick(cutoff + relative - world.cycle)
                        # At cutoff means visible before executing that cycle's PULL.
                        sm.tx.append(1)
                        world.tick(reference + at - world.cycle)
                        self.assertFalse(sm.rx)
                        world.tick()
                        if relative <= 0:
                            self.assertEqual(list(sm.rx), [1])
                            self.assertEqual(sm.pc, sm.origin + program["publicLabels"]["done"])
                            sm.rx.clear()
                        else:
                            self.assertEqual(sm.pc, sm.origin + program["publicLabels"]["missed"])
                        self.assertEqual(sm.wait_edges[-1], (reference, 11))
                        sm.tx.append(1)  # Neither success nor failure can fire twice.
                        for level in (not rising, rising):
                            world.set_pin(11, level)
                            world.tick(100)
                        self.assertFalse(sm.rx)

    def test_scheduled_absolute_events_without_cpu_service(self):
        for rising in (False, True):
            world, engines, reference = self.scheduled_world([16, 100], rising)
            engines[0].tx.append(1)
            world.tick(reference + 16 + 1 - world.cycle)
            self.assertEqual(list(engines[0].rx), [1])
            self.assertFalse(engines[1].rx)
            world.tick(30)
            engines[1].tx.append(1)  # Preparation finishes between fixed timestamps.
            world.tick(reference + 100 - world.cycle)
            self.assertFalse(engines[1].rx)
            world.tick()
            self.assertEqual(list(engines[1].rx), [1])
            world.tick(1000)  # No CPU callback is involved in either event.
            self.assertEqual([list(sm.rx) for sm in engines], [[1], [1]])

    def test_scheduled_forced_wait_while_disabled(self):
        for rising in (False, True):
            world = World(self.programs)
            world.set_pin(11, rising)
            world.tick(4)
            sm = SM(world, self.programs["scheduled_write"])
            sm.enabled = False
            sm.code[0] = 0x2000 | (int(rising) << 7) | 11
            sm.x = 1  # Earliest supported release: cycle 8.
            sm.exec_instruction(0x2000 | (int(not rising) << 7) | 11)
            world.tick(20)
            self.assertIsNotNone(sm.forced)
            self.assertEqual(sm.pc, sm.origin)
            world.set_pin(11, not rising)
            world.tick(8)
            self.assertIsNone(sm.forced)
            self.assertEqual(sm.pc, sm.origin)  # Forced WAIT never advances PC.
            sm.tx.append(1)
            world.tick(20)
            self.assertFalse(sm.rx)
            sm.enabled = True
            world.tick(4)
            world.set_pin(11, rising)
            reference = world.cycle + 2
            world.tick(reference + 8 - world.cycle)
            self.assertFalse(sm.rx)
            world.tick()
            self.assertEqual(list(sm.rx), [1])

    def test_scheduled_software_origin_shared_by_both_events(self):
        world = World(self.programs)
        times = (8, 40)
        engines = []
        for at in times:
            sm = SM(world, self.programs["scheduled_write"])
            sm.enabled = False
            sm.code[0] = 0xa042  # NOP replaces hardware reference WAIT.
            sm.x = at - 7
            sm.tx.append(1)
            engines.append(sm)
        world.tick(100)
        self.assertTrue(all(sm.pc == sm.origin and not sm.rx for sm in engines))
        reference = world.cycle
        for sm in engines:
            sm.enabled = True  # One mask write, not sequential execution cycles.
        for sm, at in zip(engines, times):
            world.tick(reference + at - world.cycle)
            self.assertFalse(sm.rx)
            world.tick()
            self.assertEqual(list(sm.rx), [1])
        world.tick(100)
        self.assertEqual([list(sm.rx) for sm in engines], [[1], [1]])

    def hardware_timer(self, world, completion, at):
        sm = SM(world, self.programs["scheduled_write"])
        sm.load_program(self.programs["scheduled_origin"])
        sm.code[17] = 1  # Runtime patches the SDK-relocated cross-program JMP.
        sm.pc, sm.y, sm.x = 16, completion - 1, at - 7
        return sm

    def test_scheduled_hardware_completion_without_cpu(self):
        self.assertEqual(self.programs["scheduled_write"]["origin"], 0)
        self.assertEqual(len(self.programs["scheduled_write"]["instructions"]), 10)
        self.assertEqual(self.programs["scheduled_origin"]["origin"], 16)
        self.assertEqual(len(self.programs["scheduled_origin"]["instructions"]), 2)
        cases = [([(DELAY, 0)], 1), ([(DELAY, 8)], 9), ([(DELAY, 127)], 128),
                 ([(SET, 0, 0, 11, HIGH)], 1), ([(SET, 0, 0, 11, LOW)], 1),
                 ([(SET, 0, 0, 10, LOW)], 2), ([(SET, 0, 0, 11, FLOATING)], 2),
                 ([(PWM, 0, 6000)], 6), ([(SET, 0, 0, 13, FLOATING)], 6)]
        for steps, completion in cases:
            for relative in (-1, 0, 1):
                with self.subTest(steps=steps, relative=relative):
                    world = World(self.programs)
                    if steps[0][0] == PWM or (steps[0][0] == SET and steps[0][3] == 13):
                        tokens, segments, block = self.compile_pwm(steps)
                        world.pwm_waveform(tokens, segments, block.wave_start_countdown)
                    else:
                        world.waveform(self.compile(steps))
                        block = self.last_block
                    self.assertEqual(block.completion_cycles, completion)
                    first = self.hardware_timer(world, completion, 8)
                    second = self.hardware_timer(world, completion, 80)
                    first.tx.append(1)
                    world.tick(completion + 8)
                    self.assertFalse(first.rx)
                    world.tick()
                    self.assertEqual(list(first.rx), [1])
                    # All subsequent servicing is absent except one readiness
                    # publication before, at, or after the hardware cutoff.
                    world.tick(completion + 80 - 4 + relative - world.cycle)
                    second.tx.append(1)
                    world.tick(completion + 80 - world.cycle)
                    self.assertFalse(second.rx)
                    world.tick()
                    self.assertEqual(list(second.rx), [1] if relative <= 0 else [])
                    world.tick(200)
                    self.assertEqual(list(first.rx), [1])
                    self.assertEqual(list(second.rx), [1] if relative <= 0 else [])

    def test_hardware_completion_u32_maximum(self):
        self.compile([(DELAY, MASK)])
        self.assertEqual(self.last_block.completion_cycles, 1 << 32)
        world = World(self.programs)
        sm = self.hardware_timer(world, 1 << 32, 8)
        self.assertEqual(sm.y, MASK)
        self.assertEqual(sm.code[16], 0x80 | 16)  # Assembled Y-- self-loop.
        sm.tx.append(1)
        # Analytically execute the taken Y-- iterations, then execute the real
        # fallthrough and cross-program branch rather than a synthetic origin.
        world.cycle += sm.y
        sm.y = 0
        world.tick(9)
        self.assertFalse(sm.rx)
        world.tick()
        self.assertEqual((world.cycle, list(sm.rx)), ((1 << 32) + 9, [1]))
        self.compile([(DELAY, 8), (SET, 0, 0, 11, HIGH)])
        self.assertEqual(self.last_block.completion_cycles, 0)

    def test_pwm_parameters_and_duty_encoding(self):
        for sys_hz, hz, expected in ((125000000, 500000, 250), (1020, 40, 26),
                                     (1019, 40, 25), (16, 1, 16), (32768, 1, 32768)):
            period = ctypes.c_uint32()
            self.assertIsNone(self.native.timing_pwm_parameters(sys_hz, hz, ctypes.byref(period)))
            self.assertEqual(period.value, expected)
        for sys_hz, hz in ((0, 500000), (125000000, 0), (15, 1), (32769, 1)):
            period = ctypes.c_uint32(123)
            self.assertEqual(self.native.timing_pwm_parameters(sys_hz, hz, ctypes.byref(period)), b"RANGE")
            self.assertEqual(period.value, 123)
        for period, duty, expected in ((250, 0, 0), (250, 10000, 250), (250, 6000, 150),
                                      (250, 1000, 25), (250, 100, 3), (16, 1875, 3),
                                      (20, 5000, 10), (32768, 5000, 16384)):
            words, count, high = (ctypes.c_uint32 * 8)(), ctypes.c_uint32(), ctypes.c_uint32()
            self.assertIsNone(self.native.timing_pwm_encode(period, duty, words, ctypes.byref(count), ctypes.byref(high)))
            self.assertEqual(high.value, expected)
            self.assertEqual(count.value, 4 if expected <= 9 or duty == 10000 else 8)
            world = self.pwm_world([(PWM, 0, duty)], period=period)
            world.advance_pwm(period * 4 + 6)
            edges = [(c, value) for c, kind, pin, value in world.edges if kind == "level" and pin == 13]
            self.assertEqual(edges, [] if duty == 0 else [(6, True)] if duty == 10000 else
                             [(6 + p * period + offset, value) for p in range(4)
                              for offset, value in ((0, True), (expected, False))])
            self.assertFalse(world.pwm.tx_stall)
        for period, duty in ((15, 6000), (32769, 6000), (250, 10001), (250, 1),
                             (250, 80), (250, 9640), (16, 5000)):
            words, count, high = (ctypes.c_uint32 * 8)(), ctypes.c_uint32(), ctypes.c_uint32()
            self.assertEqual(self.native.timing_pwm_encode(period, duty, words,
                ctypes.byref(count), ctypes.byref(high)), b"RANGE")

    def test_pwm_requested_sequence_exact_boundaries(self):
        steps = [(PWM, 0, 6000), (PULSE, 12500000, 0, 10), (DELAY, 1250),
                 (PWM, 0, 1000), (DELAY, 12500), (PWM, 0, 6000)]
        tokens, words, block = self.compile_pwm(steps)
        self.assertEqual([s.periods for s in words], [50005, 50, MASK // 8])
        self.assertEqual(block.programmed_cycles, 12513750)
        self.assertEqual(block.pwm_final_duty, 6000)
        world = World(self.programs)
        world.pwm_waveform(tokens, words, block.wave_start_countdown)
        world.advance_pwm(12513750 + 6 + 10 * 250)
        reset = [c for c, kind, pin, _ in world.edges if kind == "direction" and pin == 10]
        self.assertEqual(reset, [6, 12500006])
        pwm = [(c, value) for c, kind, pin, value in world.edges if kind == "level" and pin == 13]
        rises = [c for c, value in pwm if value]
        self.assertEqual(rises[0], 6)
        self.assertTrue(all(b - a == 250 for a, b in zip(rises, rises[1:])))
        for (rise, high), (fall, low) in zip(pwm[::2], pwm[1::2]):
            self.assertTrue(high)
            self.assertFalse(low)
            expected = 25 if 12501256 <= rise < 12513756 else 150
            self.assertEqual(fall - rise, expected)
        self.assertTrue(world.wave_complete)
        self.assertFalse(world.pwm.tx_stall)
        world.service_pwm()
        world.pwm_dma.remaining = world.pwm_dma.word_count - world.pwm_dma.offset
        start = world.cycle
        world.advance_pwm(2500)
        continued = [(c, value) for c, kind, pin, value in world.edges
                     if kind == "level" and pin == 13 and c >= start]
        self.assertEqual(continued, [(start + period * 250 + offset, value)
                                    for period in range(10)
                                    for offset, value in ((0, True), (150, False))])

    def test_pwm_phase_zero_full_and_zero_delay_reset(self):
        for duty in (0, 10000):
            world = self.pwm_world([(PWM, 0, duty)], gpio=gpio_config(pwm_pin=13))
            world.advance_pwm(2500)
            edges = [(c, value) for c, kind, pin, value in world.edges if kind == "level" and pin == 13]
            self.assertEqual(edges, [] if duty == 0 else [(6, True)])
            self.assertTrue(world.wave_complete)
            self.assertFalse(world.pwm.tx_stall)
        world = self.pwm_world([(PWM, 0, 0), (DELAY, 250), (PWM, 0, 10000),
                                (DELAY, 250), (PWM, 0, 0)])
        world.advance_pwm(2000)
        self.assertEqual([(c, value) for c, kind, pin, value in world.edges
                          if kind == "level" and pin == 13], [(256, True), (506, False)])
        for prefix, first_reset in (([], 6), ([(DELAY, 0)], 6), ([(DELAY, 250)], 256)):
            world = self.pwm_world([(PWM, 0, 6000), *prefix, (PULSE, 500, 0, 10),
                                    (DELAY, 250), (PULSE, 25, 0, 11)])
            world.advance_pwm(2000)
            reset = [c for c, kind, pin, _ in world.edges if kind == "direction" and pin == 10]
            self.assertEqual(reset, [first_reset, first_reset + 500])
            pulse = [c for c, kind, pin, _ in world.edges if kind == "level" and pin == 11]
            self.assertEqual(pulse, [first_reset + 750, first_reset + 775])
        world = self.pwm_world([(PWM, 0, 1000), (PULSE, 25, 0, 11)])
        world.advance_pwm(1000)
        self.assertEqual([c for c, kind, pin, value in world.edges
                          if kind == "level" and pin in (11, 13) and value][:2], [6, 6])

    def test_pwm_compiler_rejections_and_coalescing(self):
        for steps in ([(PWM, 0, 6000), (DELAY, 249), (PWM, 0, 1000)],
                      [(PWM, 0, 6000), (WAIT, 0)], [(WAIT, 0), (PWM, 0, 6000)],
                      [(PWM, 1, 6000)], [(PWM, 0, 10001)], [(PWM, 0, 1)],
                      [(PWM, 0, 6000), (PULSE, 250, 0, 10), (PULSE, 25, 0, 11), (PWM, 0, 1000)]):
            self.compile_pwm(steps, error=b"RANGE")
        self.compile_pwm([(PWM, 0, 6000)], configured=False, error=b"UNCONFIGURED")
        self.compile_pwm([(PWM, 0, 6000)], initial=10001, error=b"RANGE")
        self.compile_pwm([(PWM, 0, 6000)], capacity=0, error=b"RANGE")
        self.compile_pwm([(PWM, 0, 6000), (DELAY, 250)], token_capacity=1, error=b"RANGE")
        self.compile_pwm([(PWM, 0, 6000), (9, 0)], error=b"BAD_REQUEST")
        _, words, block = self.compile_pwm([(PWM, 0, 1000), (DELAY, 0), (PWM, 0, 0),
                                            (PWM, 0, 6000), (DELAY, 250), (PWM, 0, 10000),
                                            (PWM, 0, 1000)])
        self.assertEqual([s.periods for s in words], [1, MASK // 8])
        self.assertEqual(block.pwm_final_duty, 1000)
        tokens, words, block = self.compile_pwm([(DELAY, 250), (PWM, 0, 6000)], initial=1000)
        self.assertEqual(words[0].periods, 1)
        self.assertEqual(words[0].duty_basis_points, 1000)
        tokens, words, block = self.compile_pwm([(PULSE, 25, 0, 11)], configured=False,
                                                gpio=fixture_gpio(), capacity=0)
        self.assertEqual(tokens, self.compile([(PULSE, 25, 0, 11)]))
        self.assertEqual(words, [])
        self.assertEqual(block.wave_start_countdown, 0)
        steps = [(PWM, 0, 6000)] + [step for i in range(10)
                                  for step in ((DELAY, 250), (PWM, 0, 1000 if i % 2 else 6000))]
        self.compile_pwm(steps, capacity=10, error=b"RANGE")

    def test_pwm_long_repeat_splitting(self):
        # Seventeen near-u32 delays exceed 2**32 periods at the minimum period.
        duration = MASK - 15
        steps = [(PWM, 0, 0)] + [(DELAY, duration)] * 17 + [(PWM, 0, 10000)]
        _, words, block = self.compile_pwm(steps, period=16)
        periods = duration * 17 // 16
        maximum = MASK // 4
        whole, rest = divmod(periods, maximum)
        self.assertEqual([s.periods for s in words], [maximum] * whole + [rest, maximum])
        self.assertTrue(all(list(s.words) == list(words[0].words) for s in words[:-1]))
        self.assertNotEqual(list(words[-2].words), list(words[-1].words))
        self.assertEqual(block.programmed_cycles, duration * 17)
        self.compile_pwm(steps, period=16, capacity=len(words) - 1, error=b"RANGE")

    def test_pwm_dma_final_storage_abort_and_underflow(self):
        steps = [(PWM, 0, 6000)] + [step for i in range(8)
                                  for step in ((DELAY, 250), (PWM, 0, 1000 if i % 2 else 6000))]
        tokens, words, block = self.compile_pwm(steps)
        self.assertGreater(len(words), 8)
        world = World(self.programs)
        world.pwm_waveform(tokens, words, block.wave_start_countdown)
        world.advance_pwm(3000)
        self.assertTrue(world.pwm_dma.tail_owned)
        self.assertTrue(world.wave_complete)
        world.service_pwm()
        for segment in words:
            for i in range(8):
                segment.words[i] = 0
        world.pwm_dma.remaining = world.pwm_dma.word_count - world.pwm_dma.offset
        world.advance_pwm(1000)
        self.assertFalse(world.pwm.tx_stall)
        world.shutdown()
        edges = len(world.edges)
        world.advance_pwm(1000)
        self.assertEqual(len(world.edges), edges)
        self.assertFalse(world.levels & (1 << 13))
        failed = self.pwm_world(steps)
        failed.pwm_dma_enabled = False
        failed.advance_pwm(3000)
        self.assertTrue(failed.pwm.tx_stall)
        self.assertFalse(failed.wave_complete, "DMA still owns staged source storage")
        failed.service_pwm()
        self.assertEqual(failed.pwm_error, "TIMING_ERROR")
        self.assertFalse(failed.levels & (1 << 13))
        self.assertFalse(failed.pwm.enabled)
        failed.shutdown()
        self.assertIsNone(failed.pwm_error)
        # A stall after normal waveform completion is still a PWM failure.
        final = self.pwm_world([(PWM, 0, 10000)])
        final.advance_pwm(1000)
        self.assertTrue(final.wave_complete)
        final.pwm_dma_enabled = False
        final.pwm.tx.clear()
        final.advance_pwm(500)
        self.assertTrue(final.pwm.tx_stall)
        final.service_pwm()
        self.assertEqual(final.pwm_error, "TIMING_ERROR")
        self.assertFalse(final.levels & (1 << 13))

    def test_open_drain_delay_push_pull_exact_edges(self):
        world = self.world([(PULSE, 12500, 0, 10), (DELAY, 2500), (PULSE, 25, 0, 11)])
        world.tick(16000)
        reset = [c for c, kind, pin, _ in world.edges if kind == "direction" and pin == 10]
        glitch = [c for c, kind, pin, _ in world.edges if kind == "level" and pin == 11]
        self.assertEqual(reset[1] - reset[0], 12500)
        self.assertEqual(glitch[0] - reset[1], 2500)
        self.assert_pulse(world, 25)
        self.assertTrue(world.wave.tx_stall)
        self.assertFalse(world.snapshot)

    def test_widths_and_compiler_rejection(self):
        for width in (8, 9, 25):
            world = self.world([(DELAY, 0), (PULSE, width, 0, 11), (DELAY, 0)])
            world.tick(100)
            self.assert_pulse(world, width)
        for steps in ([(PULSE, 7, 0, 11)], [(PULSE, 7, 0, 10)], [(DELAY, 1)],
                      [(SET, 0, 0, 11, 3)],
                      [(WAIT, 0), (WAIT, 0)], [(WAIT, 1)]):
            self.compile(steps, error=b"RANGE")
        self.compile([(PULSE, 8, 0, 11)], capacity=5, error=b"RANGE")
        self.assertEqual(self.compile([(PULSE, 8, 0, 11)]),
                         self.compile([(DELAY, 0), (PULSE, 8, 0, 11), (DELAY, 0)]))
        self.compile([(-1, 8)], error=b"BAD_REQUEST")
        self.compile([], error=b"RANGE")
        self.compile([(PULSE, 8, 0, 30)], error=b"RANGE")
        self.compile([(PULSE, 8, 0, 0)], error=b"UNCONFIGURED")

    def test_long_count_splits(self):
        maximum = 0x0FFFFFFF + 6
        for width in (maximum - 1, maximum, maximum + 1, maximum + 6, 0xFFFFFFFF):
            world = self.world([(PULSE, width, 0, 11)])
            world.advance_hold(width + 64)
            self.assert_pulse(world, width)
            self.assertTrue(world.wave.tx_stall)
            self.assertFalse(world.snapshot)

    def test_autopull_prefetch_empty_out_and_pin_window(self):
        gpio = gpio_config((29, PUSH_PULL, 0), pwm_pin=13)
        world = World(self.programs)
        sm = world.waveform(self.compile([(PULSE, 8, 0, 29)], gpio=gpio), gpio, 13)
        world.tick()  # NOP prefetches without spending an OUT cycle.
        self.assertEqual((sm.pc, sm.osr_count), (25, 0))
        self.assertFalse(sm.tx_stall)
        world.tick()
        self.assertTrue(world.levels & (1 << 29))
        self.assertEqual(sm.osr_count, 0, "successful OUT refills concurrently")
        world.tick(30)
        self.assertEqual(sm.pc, 25, "empty stream completes at OUT PINS, not NOP")
        self.assertTrue(world.wave_complete)

        # No NOP: an initially empty OSR must stall even with a full TX FIFO.
        stalled = World(self.programs)
        sm = SM(stalled, self.programs["waveform"], out_base=30, out_count=31,
                autopull=True, fifo_capacity=8)
        sm.pc = 25
        sm.tx.extend([MASK, MASK, 2])
        stalled.directions = stalled.levels = 1 << 29  # Excluded carrier.
        stalled.tick()
        self.assertEqual((sm.pc, sm.osr_count), (25, 0))
        self.assertEqual(stalled.levels, 1 << 29)
        stalled.tick(2)
        self.assertEqual(stalled.levels, MASK)
        self.assertEqual(stalled.directions, MASK)
        self.assertFalse(any(pin == 29 for _, _, pin, _ in stalled.edges))
        self.assertEqual(sm.osr, 2, "OUT_COUNT31 still consumes all 32 OSR bits")

        # An explicit PULL cannot overwrite a full autopull OSR (warmup fence).
        sm.exec_instruction(0x80A0)  # PULL BLOCK
        sm.tx.append(123)
        stalled.tick()
        self.assertEqual(sm.osr, 2)
        self.assertEqual(list(sm.tx), [123])

    def test_arbitrary_gpio_modes_idle_and_abort(self):
        for pin, drive, idle in product((0, 10, 21, 29), range(3), (0, 1)):
            with self.subTest(pin=pin, drive=drive, idle=idle):
                configs = [(p, (drive + i) % 3, (idle + i) % 2)
                           for i, p in enumerate((0, 10, 21, 29)) if p != pin]
                gpio = gpio_config(*configs, (pin, drive, idle))
                world = self.world([(PULSE, 9, 0, pin)], gpio=gpio)
                world.external |= gpio.open_drain_mask
                # An unrelated peripheral's pad is never affected by waveform/stop.
                world.levels |= 1 << 17
                world.directions |= 1 << 17
                levels, dirs = idle_state(gpio)
                first = 1 + (drive != PUSH_PULL)
                for cycle in range(30):
                    world.tick()
                    active = first <= cycle < first + 9
                    self.assertEqual(bool(world.physical() & (1 << pin)), bool(idle ^ active))
                    self.assertEqual(world.levels & gpio.open_drain_mask, 0)
                    self.assertEqual(world.levels & gpio.open_source_mask, gpio.open_source_mask)
                    other = gpio.configured_mask & ~(1 << pin)
                    self.assertEqual(world.levels & other, levels & other)
                    self.assertEqual(world.directions & other, dirs & other)
                kind = "level" if drive == PUSH_PULL else "direction"
                self.assertEqual([c for c, k, p, _ in world.edges if p == pin and k == kind],
                                 [first, first + 9])
                self.assertEqual((world.levels & gpio.configured_mask,
                                  world.directions & gpio.configured_mask), (levels, dirs))
                active = self.world([(PULSE, 100, 0, pin)], gpio=gpio)
                active.levels |= 1 << 17
                active.directions |= 1 << 17
                active.tick(8)
                active.shutdown()
                self.assertEqual((active.levels, active.directions),
                                 (levels | (1 << 17), dirs | (1 << 17)))
                edges = list(active.edges)
                active.tick(120)
                self.assertEqual(active.edges, edges)

    def test_nonadjacent_mixed_modes_minimum_spacing(self):
        for first_drive, second_drive, first_idle, second_idle in product(range(3), range(3), (0, 1), (0, 1)):
            gpio = gpio_config((0, first_drive, first_idle), (29, second_drive, second_idle),
                               (10, OPEN_DRAIN, 1), (21, OPEN_SOURCE, 0))
            world = self.world([(PULSE, 8, 0, 0), (DELAY, 8), (PULSE, 8, 0, 29)], gpio=gpio)
            world.tick(64)
            first = 1 + (first_drive != PUSH_PULL)
            self.assertEqual([(c, p) for c, _, p, _ in world.edges],
                             [(first, 0), (first + 8, 0), (first + 16, 29), (first + 24, 29)])
            self.assertTrue(world.wave_complete)

    def test_zero_gpio_and_three_word_budget(self):
        for steps in ([(DELAY, 0)], [(DELAY, 100)], [(WAIT, 0)]):
            world = self.world(steps, gpio=Gpio())
            world.levels, world.directions = 0x25122456, 0x12345678
            world.tick(150)
            self.assertEqual((world.levels, world.directions), (0x25122456, 0x12345678))
            self.assertFalse(world.edges)
            self.assertEqual(world.wave_complete, steps != [(WAIT, 0)])
        self.compile([(PULSE, 8, 0, 0)], gpio=Gpio(), error=b"UNCONFIGURED")
        frame = self.compile([(DELAY, 0)], gpio=Gpio(), capacity=3)
        self.assertEqual(frame, [0, 0, 1])
        self.compile([(DELAY, 0)], gpio=Gpio(), capacity=2, error=b"RANGE")
        self.assertEqual(len(self.compile([(DELAY, 8)], capacity=6)), 6)
        # Maximum output is 341 frames/1023 words, never 1024 frames.
        maximum = 0x0FFFFFFF + 6
        steps = [(DELAY, maximum)] * 64
        tokens = self.compile(steps)
        self.assertEqual(len(tokens), 65 * 3)
        self.compile(steps, capacity=len(tokens) - 1, error=b"RANGE")
        full = [(DELAY, maximum * 15)] * 22 + [(DELAY, maximum * 10)]
        self.assertEqual(len(self.compile(full)), 1023)
        self.compile(full + [(DELAY, 8)], error=b"RANGE")
        self.compile([(DELAY, MASK)] * 64, capacity=4096, error=b"RANGE")

    def test_pwm_protected_between_nonadjacent_gpio_and_wrapped_base(self):
        for pwm_pin, drive, idle in product((0, 13, 22, 29), range(3), (0, 1)):
            pins = [p for p in (0, 10, 21, 29) if p != pwm_pin]
            gpio = gpio_config(*[(p, drive, idle) for p in pins], pwm_pin=pwm_pin)
            steps = [(PWM, 0, 6000), (PULSE, 8, 0, pins[0]), (DELAY, 8),
                     (PULSE, 9, 0, pins[-1])]
            world = self.pwm_world(steps, gpio=gpio, pin=pwm_pin)
            world.tick(1000)
            self.assertEqual([(c, value) for c, kind, pin, value in world.edges
                              if kind == "level" and pin == pwm_pin],
                             [(6 + period * 250 + offset, value) for period in range(4)
                              for offset, value in ((0, True), (150, False))])
            self.assertFalse(any(kind == "direction" and pin == pwm_pin for _, kind, pin, _ in world.edges))
            self.assertTrue(world.directions & (1 << pwm_pin))
            self.assertTrue(world.wave_complete)
            self.assertEqual(gpio.out_base, (pwm_pin + 1) & 31)

    def test_continuing_pwm_through_generic_trigger_blocks(self):
        for trigger in ("i2c", "spi", "gpio"):
            gpio = gpio_config((0, OPEN_DRAIN, 1), (29, OPEN_SOURCE, 0), pwm_pin=13)
            world = self.pwm_world([(PWM, 0, 6000)], gpio=gpio)
            world.tick(1000)
            world.wave.enabled = False
            steps = [(PULSE, 8, 0, 0), (DELAY, 8), (WAIT, 0), (PULSE, 9, 0, 29)]
            world.waveform(self.compile(steps, gpio=gpio), gpio)
            if trigger == "gpio":
                world.gpio_trigger(21, True, False)
                world.set_pin(21, True)
                world.tick(64)
            else:
                prefix = b"\xa5"
                world.trigger(trigger, *self.pack(trigger, prefix))
                if trigger == "i2c":
                    i2c_start(world)
                    i2c_bits(world, bits_of(b"\xa0") + [0] + bits_of(prefix))
                else:
                    spi_bits(world, bits_of(prefix), 0)
            world.tick(500)
            self.assertTrue(world.matched)
            self.assertTrue(world.wave_complete)
            carrier = [(c, v) for c, k, p, v in world.edges if k == "level" and p == 13]
            rises = [c for c, v in carrier if v]
            self.assertTrue(all(b - a == 250 for a, b in zip(rises, rises[1:])))
            self.assertTrue(all(fall[0] - rise[0] == 150 for rise, fall in zip(carrier[::2], carrier[1::2])))
            self.assertFalse(any(k == "direction" and p == 13 for _, k, p, _ in world.edges))

    def test_implicit_spacing_and_postwait_share_explicit_delays(self):
        for separator in ([], [(DELAY, 0)], [(DELAY, 8)]):
            steps = [(PULSE, 8, 0, 10), *separator, (PULSE, 8, 0, 11)]
            world = self.world(steps)
            block = self.last_block
            world.tick(64)
            self.assertEqual([(c, p) for c, _, p, _ in world.edges], [(2, 10), (10, 10), (18, 11), (26, 11)])
            self.assertEqual(block.programmed_cycles, 24)
            self.assertEqual(block.inserted_cycles, 0 if separator == [(DELAY, 8)] else 8)
        implicit = self.compile([(WAIT, 0), (PULSE, 8, 0, 11)])
        self.assertEqual((self.last_block.programmed_cycles, self.last_block.inserted_cycles), (16, 8))
        explicit = self.compile([(WAIT, 0), (DELAY, 8), (PULSE, 8, 0, 11)])
        self.assertEqual(implicit, explicit)
        self.assertEqual((self.last_block.programmed_cycles, self.last_block.inserted_cycles), (16, 0))
        for steps in ([(WAIT, 0), (PULSE, 8, 0, 11)],
                      [(PULSE, 8, 0, 10), (WAIT, 0), (PULSE, 8, 0, 11)]):
            world = self.world(steps)
            world.set_pin(12, True)  # Trigger already latched before reaching WAIT.
            world.tick(96)
            self.assert_pulse(world, 8)
            self.assertTrue(world.wave_complete)
            first = next(c for c, k, p, value in world.edges if p == 11 and value)
            wait_cycle = world.wave.wait_edges[0][0]
            self.assertGreaterEqual(first - wait_cycle, 8)

    def test_set_modes_held_across_blocks_and_float_pulse_offsets(self):
        for drive, idle, state in product(range(3), (0, 1), range(3)):
            gpio = gpio_config((29, drive, idle), (11, PUSH_PULL, 0))
            tokens = self.compile([(SET, 0, 0, 29, state)], gpio=gpio)
            block = self.last_block
            world = World(self.programs)
            world.waveform(tokens, gpio)
            world.tick(32)
            bit = 1 << 29
            self.assertEqual((world.levels, world.directions), (block.final_levels, block.final_dirs))
            self.assertEqual(bool(world.directions & bit), state != FLOATING and
                             (drive == PUSH_PULL or (state == LOW if drive == OPEN_DRAIN else state == HIGH)))
            self.assertEqual(world.levels & gpio.open_drain_mask, 0)
            self.assertEqual(world.levels & gpio.open_source_mask, gpio.open_source_mask)
            gpio.current_levels, gpio.current_dirs = block.final_levels, block.final_dirs
            held = (world.levels & bit, world.directions & bit)
            world.wave.enabled = False
            world.waveform(self.compile([(DELAY, 8), (WAIT, 0), (PULSE, 8, 0, 11)], gpio=gpio), gpio)
            world.set_pin(12, True)
            world.tick(80)
            self.assertEqual((world.levels & bit, world.directions & bit), held)
            world.shutdown()
            self.assertEqual((world.levels, world.directions), idle_state(gpio))
        for idle in (0, 1):
            gpio = gpio_config((29, PUSH_PULL, idle))
            world = self.world([(SET, 0, 0, 29, FLOATING), (PULSE, 8, 0, 29)], gpio=gpio)
            self.assertEqual((self.last_block.programmed_cycles, self.last_block.inserted_cycles), (16, 8))
            world.tick(40)
            self.assertEqual([(c, k, value) for c, k, p, value in world.edges if p == 29],
                             [(2, "direction", False), (9, "level", not idle),
                              (10, "direction", True), (18, "level", bool(idle))])
        gpio = gpio_config((11, PUSH_PULL, 0))
        world = self.world([(SET, 0, 0, 11, HIGH), (SET, 0, 0, 11, HIGH), (PULSE, 8, 0, 11)], gpio=gpio)
        world.tick(40)
        self.assertEqual(self.last_block.inserted_cycles, 0)
        self.assertEqual(world.edges, [(1, "level", 11, True), (9, "level", 11, False)])

    def test_pwm_static_modes_resume_and_shared_gpio_timeline(self):
        for state, mode in ((LOW, PWM_LOW), (HIGH, PWM_HIGH), (FLOATING, PWM_FLOATING)):
            steps = [(PWM, 0, 6000), (DELAY, 250), (SET, 0, 0, 13, state),
                     (DELAY, 250), (PWM, 0, 6000)]
            world = self.pwm_world(steps)
            world.advance_pwm(1000)
            self.assertFalse(world.pwm.tx_stall)
            self.assertEqual([(c, value) for c, k, p, value in world.edges
                              if p == 13 and k == "direction"],
                             [(256, False), (506, True)] if state == FLOATING else [])
            self.assertFalse(any(256 < c < 506 for c, _, p, _ in world.edges if p == 13))
            if state == FLOATING:
                self.assertEqual({k for c, k, p, value in world.edges if c == 506 and p == 13 and value},
                                 {"level", "direction"})
            final = self.pwm_world([(PWM, 0, 6000), (DELAY, 250), (SET, 0, 0, 13, state)])
            final.advance_pwm(400)
            final.service_pwm()
            self.assertEqual(final.pwm_final_mode, mode)
            self.assertFalse(final.pwm.enabled)
            self.assertFalse(final.pwm_dma.enabled)
            held = (final.levels, final.directions, list(final.edges))
            final.tick(1000)
            self.assertEqual((final.levels, final.directions, final.edges), held)
            self.assertEqual(bool(final.levels & (1 << 13)), state == HIGH)
            self.assertEqual(bool(final.directions & (1 << 13)), state != FLOATING)
            final.shutdown()
            self.assertFalse(final.levels & (1 << 13))
            self.assertTrue(final.directions & (1 << 13))
        # Two pulses insert eight cycles: 8+8+8+226 lands exactly on period 250.
        steps = [(PWM, 0, 6000), (PULSE, 8, 0, 11), (PULSE, 8, 0, 11),
                 (DELAY, 226), (SET, 0, 0, 13, FLOATING), (DELAY, 250), (PWM, 0, 1000)]
        tokens, segments, block = self.compile_pwm(steps)
        self.assertEqual((block.programmed_cycles, block.inserted_cycles), (500, 8))
        self.assertEqual([s.periods for s in segments], [1, 1, MASK // 8])
        world = World(self.programs)
        world.pwm_waveform(tokens, segments, block.wave_start_countdown)
        world.advance_pwm(1000)
        self.assertEqual([(c, value) for c, k, p, value in world.edges if p == 13 and k == "direction"],
                         [(256, False), (506, True)])
        self.compile_pwm(steps[:3] + [(DELAY, 234), (SET, 0, 0, 13, LOW)], error=b"RANGE")

    def test_pwm_descriptor_packet_size_crossings(self):
        # Every descriptor boundary alternates 16-byte and 32-byte read rings.
        steps = [(PWM, 0, 1500)]
        for index in range(12):
            steps += [(DELAY, 20), (PWM, 0, 5000 if index % 2 == 0 else 1500)]
        world = self.pwm_world(steps, period=20)
        world.advance_pwm(400)
        self.assertGreaterEqual(world.pwm_dma.boundaries, 13)
        self.assertTrue(world.wave_complete)
        self.assertFalse(world.pwm.tx_stall)
        edges = [(c, v) for c, k, p, v in world.edges if p == 13 and k == "level"]
        for index, (rise, fall) in enumerate(zip(edges[::2], edges[1::2])):
            self.assertEqual(rise[0], 6 + index * 20)
            self.assertEqual(fall[0] - rise[0], 10 if index < 12 and index % 2 else 3)

    def test_trigger_latency_all_drive_modes(self):
        for kind, drive, idle in product(("i2c", "spi", "gpio"), range(3), (0, 1)):
            gpio = gpio_config((29, drive, idle))
            world = self.world([(WAIT, 0), (PULSE, 9, 0, 29)], gpio=gpio)
            if kind == "gpio":
                world.gpio_trigger(0, True, False)
                world.set_pin(0, True)
                world.tick(64)
                sample = world.matcher.wait_edges[-1][0]
                latency = 7
            else:
                world.trigger(kind, *self.pack(kind, b"\xa5"))
                if kind == "i2c":
                    i2c_start(world)
                    i2c_bits(world, bits_of(b"\xa0") + [0] + bits_of(b"\xa5"))
                    clock = 5
                else:
                    spi_bits(world, bits_of(b"\xa5"), 0)
                    clock = 19
                final_sample = world.matcher.samples[-1][0]
                sample = max(c for c, p in world.matcher.wait_edges if p == clock and c < final_sample)
                latency = 11
            edges = [c for c, _, pin, _ in world.edges if pin == 29]
            # The implicit post-WAIT hold counts physical offsets, rather than
            # adding a second direction offset to the delayed output.
            self.assertEqual(edges, [sample + latency + 8, sample + latency + 17])


    def test_i2c_prefix_pack(self):
        self.assertEqual(self.pack("i2c", b""), (0xA0, 8))
        self.assertEqual(self.pack("i2c", b"\x10"), ((0xA0 << 9) | 0x10, 17))
        self.assertEqual(self.pack("i2c", b"\x10\x42"), ((0xA0 << 18) | (0x10 << 9) | 0x42, 26))
        for lsb in (False, True):
            word, count = self.pack("spi", b"\x01\x80\x55\xaa", lsb)
            expected = int("".join(map(str, bits_of(b"\x01\x80\x55\xaa", lsb))), 2)
            self.assertEqual((word, count), (expected, 32))
        payload = (ctypes.c_uint8 * 5)(1, 2, 3, 4, 5)
        word, count = ctypes.c_uint32(0xDEADBEEF), ctypes.c_uint32(123)
        self.assertEqual(self.native.timing_pack_spi(payload, 5, False, ctypes.byref(word),
                                                   ctypes.byref(count)), b"RANGE")
        self.assertEqual(self.native.timing_pack_i2c(0x50, False, payload, 3, ctypes.byref(word),
                                                   ctypes.byref(count)), b"RANGE")
        self.assertEqual((word.value, count.value), (0xDEADBEEF, 123))

    def test_i2c_address_and_single_byte_prefix(self):
        for bus, read, prefix in product((0, 1), (False, True), (b"", b"\x10")):
            world = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
            expected, count = self.pack("i2c", prefix, read=read)
            world.trigger("i2c", expected, count, bus=bus)
            i2c_start(world, bus)
            wire = bits_of(bytes([0xA0 | read]))
            if prefix:
                wire += [0] + bits_of(prefix)
            i2c_bits(world, wire, bus)
            self.assertTrue(world.matched)
            self.assert_pulse(world, 8)

    def test_i2c_matching_and_stretch(self):
        expected, count = self.pack("i2c", b"\x10\x42")
        for bus in (0, 1):
            for value, address_ack, match in ((0x42, 0, True), (0x43, 0, False), (0x42, 1, False)):
                world = self.world([(WAIT, 0), (PULSE, 9, 0, 11)])
                world.trigger("i2c", expected, count, bus=bus)
                i2c_start(world, bus)
                wire = bits_of(b"\xa0") + [address_ack] + bits_of(b"\x10") + [0] + bits_of(bytes([value]))
                i2c_bits(world, wire, bus, stretch=500)
                world.tick(200)
                self.assertEqual(world.matched, match)
                if match:
                    self.assert_pulse(world, 9)
                    final_sample = world.matcher.samples[-1][0]
                    pulse = next(c for c, kind, pin, value in world.edges if kind == "level" and pin == 11 and value)
                    self.assertEqual(pulse - final_sample, 18)
                    clock_edge = max(c for c, pin in world.matcher.wait_edges
                                     if pin == 5 + 2 * bus and c < final_sample)
                    self.assertEqual(pulse - clock_edge, 19)
                else:
                    self.assertFalse(any(pin == 11 for _, _, pin, _ in world.edges))

    def test_i2c_partial_boundaries_never_join(self):
        expected, count = self.pack("i2c", b"\x10\x42")
        wire = bits_of(b"\xa0") + [0] + bits_of(b"\x10") + [0] + bits_of(b"\x42")
        for bus in (0, 1):
            for restart in (False, True):
                for cut in range(1, len(wire)):
                    with self.subTest(bus=bus, restart=restart, cut=cut):
                        world = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
                        world.boundary_delay = 8
                        world.trigger("i2c", expected, count, bus=bus)
                        i2c_start(world, bus)
                        i2c_bits(world, wire[:cut], bus)
                        i2c_boundary(world, restart, bus)
                        i2c_bits(world, wire[cut:], bus)
                        self.assertFalse(world.matched)
                        i2c_boundary(world, False, bus)
                        i2c_start(world, bus)
                        i2c_bits(world, wire, bus)
                        self.assertTrue(world.matched)
                        self.assert_pulse(world, 8)

    def test_spi_modes_orders_prefixes_at_10mhz(self):
        for mode in range(4):
            for lsb in (False, True):
                for prefix in (b"\xa5", b"\x93\x10\x42\xe7"):
                    for active_low in (False, True):
                        expected, count = self.pack("spi", prefix, lsb)
                        world = self.world([(WAIT, 0), (PULSE, 9, 0, 11)])
                        world.trigger("spi", expected, count, mode=mode, active_low=active_low)
                        wrong = bits_of(prefix, lsb)
                        wrong[len(wrong) // 2] ^= 1
                        spi_bits(world, wrong, mode, active_low=active_low)
                        self.assertFalse(world.matched)
                        spi_bits(world, bits_of(prefix, lsb), mode, active_low=active_low)
                        self.assertTrue(world.matched, (mode, lsb, prefix, active_low))
                        self.assert_pulse(world, 9)
                        pulse = next(c for c, kind, pin, value in world.edges if kind == "level" and pin == 11 and value)
                        self.assertEqual(pulse - world.matcher.samples[-1][0], 18)
                        sample = world.matcher.samples[-1][0]
                        clock_edge = max(c for c, pin in world.matcher.wait_edges if pin == 19 and c < sample)
                        self.assertEqual(pulse - clock_edge, 19)

    def test_spi_partial_windows_and_arm_selected(self):
        for mode in range(4):
            for lsb in (False, True):
                for prefix in (b"\xa5", b"\x93\x10\x42\xe7"):
                    expected, count = self.pack("spi", prefix, lsb)
                    wire = bits_of(prefix, lsb)
                    for active_low, cut in product((False, True), range(1, len(wire))):
                        with self.subTest(mode=mode, lsb=lsb, length=len(prefix), cut=cut, active_low=active_low):
                            world = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
                            world.boundary_delay = 8
                            world.trigger("spi", expected, count, mode=mode, active_low=active_low)
                            spi_bits(world, wire[:cut], mode, active_low=active_low)
                            spi_bits(world, wire[cut:], mode, active_low=active_low)
                            self.assertFalse(world.matched)
                            spi_bits(world, wire, mode, active_low=active_low)
                            self.assertTrue(world.matched)
                            self.assert_pulse(world, 8)
                    for active_low in (False, True):
                        world = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
                        world.trigger("spi", expected, count, mode=mode, active_low=active_low, initially_selected=True)
                        spi_bits(world, wire, mode, select=False, active_low=active_low)
                        self.assertFalse(world.matched)
                        spi_bits(world, wire, mode, active_low=active_low)
                        self.assertTrue(world.matched)

    def test_underrun_and_boundary_delivery_failure(self):
        steps = [(op, 8, 0, 11) for _ in range(12) for op in (PULSE, DELAY)]
        world = self.world(steps)
        world.wave_dma_enabled = False
        world.tick(200)
        self.assertTrue(world.wave.tx_stall)
        self.assertEqual(world.wave.pc, 27, "eight-word prefill can stop inside a three-word frame")
        world.wave_dma_enabled = True
        world.tick(400)
        self.assertTrue(world.snapshot, "final FIFO write must retain earlier underrun")
        self.assertFalse(world.wave_complete, "normal final stall must not hide DMA underrun")
        world.shutdown()
        self.assertFalse(world.levels & (1 << 11))
        world = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
        world.trigger("i2c", *self.pack("i2c", b"\x10\x42"))
        world.boundary_budget = 2
        for _ in range(8):
            i2c_boundary(world, False)
        self.assertEqual(world.boundary_budget, 0)
        self.assertTrue(world.boundary.rx_stall)
        world.shutdown()
        world.tick(64)
        self.assertFalse(world.matched)

    def test_gpio_edges_latency_and_rearm(self):
        for pin, rising, initial in product((0, 29), (False, True), (False, True)):
            with self.subTest(pin=pin, rising=rising, initial=initial):
                world = self.world([(WAIT, 0), (PULSE, 9, 0, 11)])
                world.gpio_trigger(pin, rising, initial)
                self.assertFalse(world.matched)
                self.assertFalse(world.directions & (1 << pin))
                world.set_pin(pin, not rising)
                world.tick(4)
                self.assertFalse(world.matched)
                world.set_pin(pin, rising)
                world.tick(64)
                self.assertTrue(world.matched)
                self.assert_pulse(world, 9)
                sampled = world.matcher.wait_edges[-1][0]
                pulse = next(c for c, kind, p, value in world.edges
                             if kind == "level" and p == 11 and value)
                self.assertEqual(pulse - sampled, 15)
                # Additional edges cannot replay the completed pulse.
                world.set_pin(pin, not rising); world.tick(4)
                world.set_pin(pin, rising); world.tick(64)
                self.assert_pulse(world, 9)
                world.shutdown()
                world.waveform(self.compile([(WAIT, 0), (PULSE, 8, 0, 11)]))
                world.gpio_trigger(pin, rising, rising)
                self.assertFalse(world.matched)
                world.set_pin(pin, not rising); world.tick(4)
                world.set_pin(pin, rising); world.tick(64)
                self.assertTrue(world.matched)

    def test_gpio_latches_during_reset_and_abort_disarms(self):
        world = self.world([(PULSE, 12500, 0, 10), (WAIT, 0), (DELAY, 8), (PULSE, 8, 0, 11)])
        world.gpio_trigger(21, True, False)
        world.set_pin(21, True); world.tick(32)
        self.assertTrue(world.matched)
        self.assertTrue(world.directions & (1 << 10))
        world.tick(12600)
        self.assert_pulse(world, 8)
        stopped = self.world([(WAIT, 0), (PULSE, 8, 0, 11)])
        stopped.gpio_trigger(21, False, True)
        stopped.shutdown()
        stopped.set_pin(21, False); stopped.tick(64)
        self.assertFalse(stopped.matched)
        self.assertFalse(stopped.levels & (1 << 11))

    def test_latched_during_reset_and_delayed_cpu(self):
        world = self.world([(PULSE, 12500, 0, 10), (WAIT, 0), (DELAY, 2500), (PULSE, 25, 0, 11)])
        expected, count = self.pack("spi", b"\xa5")
        world.trigger("spi", expected, count)
        spi_bits(world, bits_of(b"\xa5"), 0)
        self.assertTrue(world.matched)
        self.assertTrue(world.directions & (1 << 10))
        # No CPU service exists in this execution: instructions and DMA alone.
        world.tick(16000)
        self.assert_pulse(world, 25)
        self.assertFalse(world.directions & (1 << 10))
        self.assertTrue(world.wave.tx_stall)
        for reason in ("abort", "DTR loss", "deadline"):
            with self.subTest(reason=reason):
                active = self.world([(PULSE, 12500, 0, 10)])
                active.tick(32)
                active.shutdown()
                active.tick(13000)
                self.assertFalse(active.directions & (1 << 10))
                self.assertFalse(active.levels & (1 << 11))


if __name__ == "__main__":
    unittest.main()
