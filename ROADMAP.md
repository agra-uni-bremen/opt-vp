# Roadmap

Where the VP stands, what is planned, and what to do next. One line per item; the detail lives
in the linked plan or study. Keep this file up-to-date.

**Status words:** `done`, implemented and in use. `partial`, part of it is implemented and the
entry says which. `in progress`, being worked on now. `planned`, agreed but not started.
`proposed`, an idea with a written case, not yet agreed.

*Last updated: 2026-10-02.*

## State of the repository
The repository builds. 
Simulation and tracing of the benchmark suites all work. 

## Features and goals
| Item | Status | Where | Next step |
| --- | --- | --- | --- |
| Refactor the repository | in progress | [refactor-vp](docs/plans/refactor-vp.md) | Man refactor of the VP is done. The next target is the platform layer and a decision on how to unify rv32 vs rv64|
| Testsuite and CI | partial | [test-suite](docs/plans/test-suite.md), [tests/trace](tests/trace/README.md), [tests/isa](tests/isa/README.md), [ci.yml](.github/workflows/ci.yml) | The riscv-tests ISA suites currently runs on `test32-vp` and `test64-vp` (238 tests, 17 known failures)|
| Fix bugs found by the ISA suites| proposed | [tests/isa/suites.json](tests/isa/suites.json), [test-suite](docs/plans/test-suite.md) | rv64 SRLI drops the upper 32 bits (`(uint32_t)` cast in rv64 `iss.cpp`, a one line fix that also fixes C.SRLI); rv64 lacks the trigger CSRs rv32 has; FLW/FSW/FLD/FSD ignore `mstatus.FS`; a `minstret` write does not suppress its own increment; EBREAK throws in the direct runner instead of trapping; Sv32/Sv39 runs segfault in `CombinedMemoryInterface` because the DMI bounds checks are asserts. Each fix removes a known failure |
| Unify the rv32 and rv64 ISS | next | [next-steps](docs/plans/next-steps.md) | decide how to unify the two implementations |
| Configurable pipeline | proposed | - | would split `exec_step` by pipeline stage rather than by extension|
| Unify the platform mains | proposed | [vp/src/platform](vp/src/platform) | `tiny32-mc`/`tiny64-mc` differ in one line, the `using namespace`, plus two whitespace hunks; `linux`/`linux32` in that line and one memory size, and their `prci.h` files are byte identical; `tiny32`/`tiny64` in that line, `rv32_align_address` against `rv64_align_address` and a commented out include. The includes are unqualified and resolve through the rv32 or rv64 target, which is why the copies came out identical, so each pair can be one source compiled twice with `-DRV_ARCH_NS=rv32/rv64`. |
| Decode cache | proposed | [instr.cpp](vp/src/core/common/instr.cpp), `ISS::exec_step` | every instruction is decoded from scratch on every execution. Cache the decode and not the fetch: keep `load_instr(pc)` every step, then look up a direct mapped table indexed by pc bits and tagged with the word just fetched. A tag mismatch is a miss, so a remap, an `SFENCE.VMA` and code written at run time all correct themselves with no invalidation hook, which a pc keyed cache would need in three places the code has none |
| Flat per pc statistics | proposed | `register_sets` in [trace.h](vp/src/trace/trace.h) | the last tree lookup on the insert path, once per node per executed instruction, and 112 bytes of allocation per pc. Measured shape: 74.6 percent of nodes run at one pc but the 3 percent with more than six take 31 percent of the updates, so it wants the first entry inline and the rest in a sorted array |

## Next steps
[docs/plans/next-steps.md](docs/plans/next-steps.md) is the short version of everything below,
written to be read before changing anything.

Finish the survey of [refactor-vp](docs/plans/refactor-vp.md). The remaining readers
are the two instruction set simulators and the trace library read as fresh code.

Open Decisions:
  - `tiny32` and `tiny64` use `RealCLINT`, which follows the wall clock, while every other
    single core platform uses the simulated time `CLINT<1>`. `blocking-sleep` on `tiny32-vp`
    retires 4,496,352 / 4,493,272 / 4,491,858 instructions on three runs and the same program on
    `riscv-vp` retires 4,166,813 every time. A trace of a program that reads mtime is therefore
    not reproducible on one of the most used platforms. Switching the default target to `riscv-vp` is not
    the answer, because it brings thirteen bus ports of peripherals against tiny32's three. `hwitl` is the one platform that needs wall clock time;
  - whether to unify the rv32 and rv64 ISS. After normalising the word size, `iss.cpp` still
    differs in 1,135 of about 2,250 lines, so the shared part is smaller than it looks. 
    `iss.h` differs in 105 of 444 and `mmu.h` only in the namespace name;

  - `InstructionBuffer` in [rv32/iss.h](vp/src/core/rv32/iss.h) is dead and was already deleted
    from rv64. Wiring it up would halve the fetches for compressed code, which is about half of
    an rv32imc benchmark.
  - `export_trees` holds two copies of a tree's json at once, because `update` deep copies its
    argument. Moving the top level values keeps the bytes and the key order
    ([export_json.cpp](vp/src/trace/export_json.cpp));

Run `python3 tests/trace/check.py` and `python3 tests/isa/check.py` after every commit. both
must stay green without touching a reference file or the list of known failures.
