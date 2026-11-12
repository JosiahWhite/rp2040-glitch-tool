"""Exercise the production core1 mailbox with real native worker threads."""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ProductionExecutorTests(unittest.TestCase):
    def test_threaded_ownership_and_progress(self):
        with tempfile.TemporaryDirectory(prefix="rp2040-executor-") as directory:
            executable = Path(directory) / "executor-runtime"
            subprocess.run(
                shlex.split(os.environ.get("CC", "cc"))
                + [
                    "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread",
                    "-I", str(ROOT),
                    str(ROOT / "tests/executor_runtime_harness.c"),
                    "-o", str(executable),
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([str(executable)], check=True, cwd=ROOT, timeout=15)


if __name__ == "__main__":
    unittest.main()
