"""Run the riscv-tests ISA suites on the VP and compare the results against suites.json.

    python3 tests/isa/check.py                 # build what is missing, run every suite
    python3 tests/isa/check.py --suite rv32um  # one suite; may be given more than once
    python3 tests/isa/check.py --test rv32ui-p-add
    python3 tests/isa/check.py --list

Each test is a small self checking program from riscv-tests (the submodule in
`riscv-tests/`). It ends by writing to the `tohost` symbol: 1 is a pass, any other value
names the failing test case as (number << 1) | 1. `test32-vp` prints the value as
`to-host: <value>` and exits 1 on a failure, and this script reads both.
"""

import argparse
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "scripts"))
import vpbench  # noqa: E402

SUITES = HERE / "suites.json"
RISCV_TESTS = HERE / "riscv-tests"
ISA_DIR = RISCV_TESTS / "isa"
ENV_DIR = RISCV_TESTS / "env" / "p"
BUILD_DIR = vpbench.REPO_ROOT / "out" / "isa"

#: riscv-tests link every program at the start of RAM on the reference platform.
MEMORY_START = 0x80000000
#: A test runs a few thousand instructions. One that is still running after this is stuck.
TIMEOUT_S = 30

_TO_HOST = re.compile(r"^to-host: (\d+)\s*$", re.M)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="check.py",
        description="Run the riscv-tests ISA suites on the VP.")
    parser.add_argument("--suite", action="append", default=None,
                        help="run only this suite; may be given more than once")
    parser.add_argument("--test", action="append", default=None,
                        help="run only this test, e.g. rv32ui-p-add; may be given more than once")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1,
                        help="tests to build and run at the same time")
    parser.add_argument("--list", action="store_true", help="list the tests and exit")
    return parser.parse_args(argv)


def compiler():
    """Return the cross compiler, or explain how to get one."""
    prefixes = [os.environ["RISCV_PREFIX"]] if os.environ.get("RISCV_PREFIX") else \
        ["riscv64-unknown-elf-", "riscv32-unknown-elf-"]
    for prefix in prefixes:
        path = shutil.which(prefix + "gcc")
        if path:
            return path
    raise vpbench.VpError(
        f"no cross compiler: looked for {', '.join(p + 'gcc' for p in prefixes)}. "
        f"Install gcc-riscv64-unknown-elf or set RISCV_PREFIX")


def suite_tests(suite):
    """Read the test names of one suite from its Makefrag."""
    makefrag = ISA_DIR / suite / "Makefrag"
    if not makefrag.is_file():
        raise vpbench.VpError(
            f"{makefrag} is not there. Fetch the submodule first:\n"
            f"    git submodule update --init --recursive tests/isa/riscv-tests")
    text = makefrag.read_text().replace("\\\n", " ")
    match = re.search(rf"^{suite}_sc_tests\s*=(.*)$", text, re.M)
    if not match:
        raise vpbench.VpError(f"{makefrag} has no {suite}_sc_tests list")
    return [f"{suite}-p-{name}" for name in match.group(1).split()]


def build(gcc, suite, test):
    """Compile one test unless its ELF is newer than its source. Return the ELF path."""
    source = ISA_DIR / suite["name"] / (test.split("-p-", 1)[1] + ".S")
    elf = BUILD_DIR / test
    if elf.is_file() and elf.stat().st_mtime >= source.stat().st_mtime:
        return elf
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    command = [gcc, f"-march={suite['march']}", f"-mabi={suite['mabi']}",
               "-static", "-mcmodel=medany", "-fvisibility=hidden", "-nostdlib", "-nostartfiles",
               f"-I{ENV_DIR}", f"-I{ISA_DIR / 'macros' / 'scalar'}", f"-T{ENV_DIR / 'link.ld'}",
               str(source), "-o", str(elf)]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if result.returncode != 0:
        raise vpbench.VpError(f"{test} did not build:\n    {' '.join(command)}\n"
                              + "\n".join("    " + line for line in result.stdout.splitlines()[-8:]))
    return elf


def run(suite, elf):
    """Run one test and return (passed, what to print when it did not)."""
    vp = vpbench.find_vp(suite["vp"])
    # Without --suppress-prompts the VP asks on the terminal what to do about a misaligned
    # access or a disabled FPU, and waits 10 s for an answer. With it, both trap, which is
    # what the specification asks for.
    command = [str(vp), "--suppress-prompts", "--memory-start", str(MEMORY_START),
               "--isa", suite["isa"], str(elf)]
    env = dict(os.environ, SYSTEMC_DISABLE_COPYRIGHT_MESSAGE="1")
    try:
        result = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, env=env, timeout=TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return False, f"did not finish within {TIMEOUT_S} s"
    text = result.stdout.decode("utf-8", "replace")
    to_host = _TO_HOST.findall(text)
    if result.returncode == 0 and to_host == ["1"]:
        return True, ""
    if to_host and to_host[-1] != "1":
        value = int(to_host[-1])
        # The trap handler of env/p reports a trap the test did not expect by setting the
        # bits of 1337 in the number of the running test case.
        if value & 1337 == 1337:
            return False, f"a trap the test did not expect (tohost {value})"
        return False, f"test case {value >> 1} failed (tohost {value})"
    last = [line for line in text.splitlines() if line.strip()][-3:]
    return False, (f"exited {result.returncode} without writing tohost"
                   + "".join(f"\n        {line}" for line in last))


def main(argv):
    sys.stdout.reconfigure(line_buffering=True)
    args = parse_args(argv)
    config = json.loads(SUITES.read_text())
    suites = config["suites"]
    if args.suite:
        unknown = set(args.suite) - {s["name"] for s in suites}
        if unknown:
            vpbench.note(f"[isa-check] no such suite: {', '.join(sorted(unknown))}")
            return 1
        suites = [s for s in suites if s["name"] in args.suite]
    skip = config.get("skip", {})
    known = config.get("known_failures", {})

    try:
        tests = [(s, t) for s in suites for t in suite_tests(s["name"]) if t not in skip]
    except vpbench.VpError as error:
        vpbench.note(f"[isa-check] {error}")
        return 1
    if args.test:
        unknown = set(args.test) - {t for _, t in tests}
        if unknown:
            vpbench.note(f"[isa-check] no such test: {', '.join(sorted(unknown))}")
            return 1
        tests = [(s, t) for s, t in tests if t in args.test]

    if args.list:
        for _, test in tests:
            reason = known.get(test)
            print(f"{test:<32}{'known failure: ' + reason if reason else ''}")
        return 0

    try:
        gcc = compiler()
        for vp in {s["vp"] for s, _ in tests}:
            vpbench.find_vp(vp)
    except vpbench.VpError as error:
        vpbench.note(f"[isa-check] {error}")
        return 1

    def build_and_run(item):
        suite, test = item
        try:
            return run(suite, build(gcc, suite, test))
        except vpbench.VpError as error:
            return False, str(error)

    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(build_and_run, tests))

    failures, fixed, expected = [], [], 0
    for (_, test), (passed, message) in zip(tests, results):
        if test in known:
            if passed:
                print(f"PASS  {test:<32}listed as a known failure. Remove it from suites.json")
                fixed.append(test)
            else:
                expected += 1
            continue
        if not passed:
            print(f"FAIL  {test:<32}{message}")
            failures.append(test)

    passed = sum(1 for ok, _ in results if ok)
    skipped = sum(1 for test in skip if test.split("-p-")[0] in {s["name"] for s in suites})
    print(f"\n{len(tests)} tests: {passed} pass, {expected} known failures, "
          f"{len(failures)} unexpected failures, {skipped} skipped")
    if failures or fixed:
        print("A failure here means the VP executes an instruction differently from the RISC-V")
        print("specification. Run one test alone with --test to see its output.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
