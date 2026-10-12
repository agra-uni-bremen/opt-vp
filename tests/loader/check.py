"""Check that the VP reports a program it cannot load instead of crashing.

    python3 tests/loader/check.py

The VP maps the ELF file into memory before the simulation starts. A file that does not
exist or is empty must end the run with an error that names the file. A segmentation fault
or a silent exit is a failure. A valid program must still run, so that the check cannot pass
because every run fails.

The binaries come from vp/build/bin, or from the directory in VP_BIN_DIR.
"""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "scripts"))
import vpbench  # noqa: E402


def run(vp, program):
    process = subprocess.run([str(vp), "--intercept-syscalls", str(program)],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             env={**os.environ, "SYSTEMC_DISABLE_COPYRIGHT_MESSAGE": "1"}, timeout=60)
    return process.returncode, process.stdout


def main():
    try:
        vp = vpbench.find_vp("tiny32-vp")
        valid = vpbench.resolve_program("sw/trace-test-minimal")
    except vpbench.VpError as error:
        print(f"[loader-check] {error}")
        return 1

    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        missing = Path(tmp) / "missing.elf"
        empty = Path(tmp) / "empty.elf"
        empty.write_bytes(b"")
        cases = [
            ("missing file", missing, f"cannot open '{missing}': No such file or directory"),
            ("empty file", empty, f"cannot map '{empty}'"),
        ]
        for name, program, expected in cases:
            code, output = run(vp, program)
            # a negative return code is a signal, such as -11 for a segmentation fault
            if code <= 0 or expected not in output:
                failures.append(name)
                print(f"FAIL  {name}: exit {code}, expected an error containing \"{expected}\"")
                for line in output.splitlines()[-5:]:
                    print(f"      {line}")
            else:
                print(f"ok    {name}")

    code, output = run(vp, valid)
    if code != 0:
        failures.append("valid program")
        print(f"FAIL  valid program: exit {code}")
        for line in output.splitlines()[-5:]:
            print(f"      {line}")
    else:
        print("ok    valid program")

    if failures:
        print(f"\n{len(failures)} loader cases failed: {', '.join(failures)}")
        return 1
    print("\nall loader cases pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
