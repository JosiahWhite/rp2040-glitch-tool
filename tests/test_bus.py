"""Exercise the production transfer core and native register lifecycle."""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ProductionBusTests(unittest.TestCase):
    def test_transfer_core(self):
        self.run_harness("bus_harness")

    def test_controller_lifecycle(self):
        self.run_harness("i2c_runtime_harness")

    def run_harness(self, name):
        with tempfile.TemporaryDirectory(prefix="rp2040-bus-") as directory:
            executable = Path(directory) / name
            subprocess.run(
                shlex.split(os.environ.get("CC", "cc"))
                + [
                    "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT),
                    str(ROOT / "i2c_bus.c"), str(ROOT / "tests" / f"{name}.c"),
                    "-o", str(executable),
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([str(executable)], check=True, cwd=ROOT, timeout=10)


if __name__ == "__main__":
    unittest.main()
