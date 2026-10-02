# ISA test suites

Checks that the VP execution matches the RISC-V specification. 
Run it after changes to the cores that affect simulation behavior:

    cd vp/build && ctest -R isa

or directly:

    python3 tests/isa/check.py

It takes about five seconds for 238 tests. It needs `test32-vp` and `test64-vp` (both in
`make essential`), the bare metal cross compiler (`riscv64-unknown-elf-gcc` or
`riscv32-unknown-elf-gcc`, or set `RISCV_PREFIX`), and the submodule:

    git submodule update --init --recursive tests/isa/riscv-tests

## What it runs

`riscv-tests/` is the upstream riscv-tests repository, pinned by the submodule. Each test is
a small assembly program that checks one instruction or one privileged feature and reports
through the `tohost` symbol: 1 is a pass, `(n << 1) | 1` says test case `n` failed. The
programs run in machine mode on bare memory (the `p` environment of riscv-tests).

`suites.json` lists the suites, which are directories of `riscv-tests/isa`:

* `rv32ui`, `rv32um`, `rv32ua`, `rv32uc`, `rv32uf`, `rv32ud`: user level instructions of the
  I, M, A, C, F and D extensions on `test32-vp`;
* `rv32mi`, `rv32si`: machine and supervisor level features, such as CSRs, traps and
  virtual memory;
* the same eight for rv64 on `test64-vp`.

`check.py` reads the test names from each suite's `Makefrag` and compiles every test itself
into `out/isa/`, with the `-march` that `suites.json` gives. It does not use the riscv-tests
Makefile, because that one assembles `rv32ua` with Zacas and Zabha, which the Debian cross
compiler does not know. Each test runs with `--suppress-prompts`: without it the VP asks on
the terminal what to do about a misaligned access or a disabled FPU and waits.

The B extension suites (`rv32uzba` and the others) are not listed: the VP does not
implement B.

## Results

Every test ends in one of three ways:

* it passes;
* it fails and `suites.json` lists it under `known_failures`, with the reason. That is not
  reported;
* anything else is a failure of the check: a test that fails and is not listed, and a
  listed test that passes. The second keeps the list current: remove the entry.

`skip` lists the tests that are not built at all, also with a reason.

A failure prints the test and how it failed:

    FAIL  rv32ui-p-sltu                   test case 7 failed (tohost 15)

To see why, run that test alone and read the source of test case 7 in
`riscv-tests/isa/rv64ui/sltu.S` (the rv32 tests include the rv64 sources):

    python3 tests/isa/check.py --test rv32ui-p-sltu
    vp/build/bin/test32-vp --suppress-prompts --memory-start 2147483648 out/isa/rv32ui-p-sltu

`a trap the test did not expect` means the program trapped where it did not ask for a trap,
usually an instruction the VP decodes as illegal.

## test32-vp and test64-vp

The test platform links the program at 0x80000000, as riscv-tests expect, and watches
`tohost`. It prints `to-host: <value>` when the program writes it and exits 1 when the value
is a failure. `test64-vp` is the same source compiled against the rv64 core, see
`vp/src/platform/test64/CMakeLists.txt`.
