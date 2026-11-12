import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ProductionFirmwareTests(unittest.TestCase):
    def compile_run(self, name, sources):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / name
            subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                            "-Werror", "-ffunction-sections", "-fdata-sections", "-I.",
                            "-Itests/native", *sources,
                            "-Wl,--gc-sections", "-o", str(target)], cwd=ROOT, check=True)
            subprocess.run([str(target)], cwd=ROOT, check=True)

    def test_bounded_production_protocol(self):
        self.compile_run("protocol", ["protocol.c", "tests/protocol_harness.c"])

    def test_production_run_lifecycle(self):
        """Cover concurrent WriteAt groups, command origins, barriers and cancellation."""
        self.compile_run("firmware", ["protocol.c", "sequence.c", "i2c_bus.c", "timing.c",
                                      "tests/firmware_harness.c"])


if __name__ == "__main__":
    unittest.main()
