# Extend the test suite: simulator correctness and trace correctness

This ExecPlan is a living document. The sections `Progress`, `Surprises & Discoveries`,
`Decision Log`, and `Outcomes & Retrospective` must be kept up to date as work proceeds. It
follows `.claude/PLANS.md`.


## Purpose / Big Picture

The VP (this repository: a SystemC simulator of RISC-V cores that records what a program
executes as execution sequence trees and writes them as JITR JSON files) changes often,
because several research topics build on it. Before this work, one check existed:
`tests/trace/check.py` runs a few small programs and compares a digest of each output file
against a committed reference. That says whether a change moved the trace. It cannot say
whether the trace was right in the first place, and it barely exercises the simulator: a
change that made SLTU compare signed numbers passed all eleven cases.

After this work, three more questions have an automatic answer, each in a few seconds:

1. Does the VP execute instructions the way the RISC-V specification says? The upstream
   riscv-tests suites run on both cores, 238 tests, with a list of known failures that
   names the reason for each.
2. Is a JITR trace internally consistent? A set of invariants every trace must satisfy,
   checked on every case and usable on any benchmark trace.
3. Is a JITR trace right? A second, independent implementation of the trace, built from the
   list of executed instructions, compared node by node with what the tracer wrote.

Plus: switches that are meant to change only the speed (`--performance-mode`, `--no-dmi`)
are pinned to produce the identical result.

To see it working, from the repository root after `make essential`:

    python3 tests/isa/check.py
    238 tests: 221 pass, 17 known failures, 0 unexpected failures, 5 skipped

    python3 tests/trace/check.py
    ...
    all 15 cases match the reference

The owner asked that the tests concentrate on simulator correctness and the main tracing
functions (the JITR trees). The analysis part of the VP (sequence selection, scoring,
coverage export) is expected to change a lot and is deliberately not tested here.


## Progress

- [x] (2026-10-02) Survey: existing checks, CI workflow, `test32-vp`, tracer insert path.
- [x] (2026-10-02) Milestone 1: rv32 ISA suites on `test32-vp`, `tests/isa/check.py`,
      riscv-tests as a pinned submodule, `test32-vp` exit status.
- [x] (2026-10-02) Milestone 2: trace invariants, `tests/trace/invariants.py`, run on every
      case.
- [x] (2026-10-02) Milestone 3: reference model, `tests/trace/model.py`, new program
      `sw/trace-test-rv32im`, nine cases marked `"model": true`.
- [x] (2026-10-02) Milestone 4: `same_as` cases for `--performance-mode` and `--no-dmi`.
- [x] (2026-10-02) Milestone 5: `test64-vp` and the rv64 suites.
- [x] (2026-10-02) Milestone 6: ctest `isa`, `essential` builds both test platforms, CI runs
      both checks. Also running on GitHub.
- [ ] Milestone 7 (proposed): run every platform binary once in the `build-all` CI job.
- [ ] Milestone 8 (proposed): compiled C programs in CI with a fixed toolchain.
- [ ] Decide with the owner which of the findings below to fix. Each fix changes either a
      known failure in `tests/isa/suites.json` or a known deviation in
      `tests/trace/model.py`, and the tracer fixes change the reference traces.


## Surprises & Discoveries

Findings in the simulator, from the ISA suites. Each one is a known failure in
`tests/isa/suites.json`; none is fixed by this plan.

- rv64 SRLI is wrong. `vp/src/core/rv64/iss.cpp` computes
  `((uint32_t)regs[instr.rs1()]) >> instr.shamt()`, so the upper 32 bits are lost. It also
  breaks C.SRLI and every `srl` with a constant, which is how `rv64mi-p-mcsr` reads
  `misa.MXL`. Changing the cast to `uint64_t` makes `rv64ui-p-srli`, `rv64uc-p-rvc` and
  `rv64mi-p-mcsr` pass (tried and reverted). This is the most serious finding: any rv64
  program that shifts a 64 bit value right by a constant computes a wrong result.
- rv64 does not implement the trigger CSRs (`tselect`, `tdata1` to `tdata3`); they are
  commented out in `vp/src/core/rv64/iss.cpp`, while rv32 has them. `rv64mi-p-breakpoint`
  traps on the first access.
- FLW, FSW, FLD and FSD do not check `mstatus.FS`, on both cores. With the FPU off an FP
  store writes memory instead of trapping (`rv32mi-p-csr` test case 13).
- A CSR write to `minstret` does not suppress the increment of the writing instruction
  (`instret_overflow` test case 2).
- EBREAK stops the direct runner with an exception instead of raising a breakpoint trap
  (`sbreak`). The code says this is deliberate (`// TODO: also raise trap`).
- The VP traps on misaligned loads and stores, which the specification allows; `ma_data`
  needs them to complete. Not a bug.
- Segmentation faults under virtual memory: `rv32si-p-dirty` and `rv64si-p-dirty` in
  `CombinedMemoryInterface::store_word`, `rv64si-p-icache-alias` in
  `CombinedMemoryInterface::load_instr`. The direct memory access bounds checks in
  `vp/src/core/common/dmi.h` are `assert`s, which a Release build removes, so a translated
  address outside the memory reads or writes outside the host buffer. Not investigated
  further.
- Without `--suppress-prompts` the VP asks on the terminal what to do about a misaligned
  access, a disabled extension or a disabled FPU, and waits 10 seconds for an answer. A
  test harness must pass it and give the VP no stdin, or a run takes 30 seconds and its
  result depends on the terminal.

Findings in the tracer, from the reference model. Each one is a known deviation in
`tests/trace/model.py`; none is fixed by this plan, because each fix changes the reference
traces and that is the owner's decision.

- The last full window is lost. `Tracer::flush` (`vp/src/trace/tracer.cpp`) starts at
  `index_ + 1`, but after the last `end_step` the slot at `index_` holds the oldest window
  not yet inserted: the one of full depth that ends at the program's final instruction. So
  every run is one window short and the root weights sum to the instruction count minus
  one. Evidence, `sw/trace-test-prefix` at depth 6: the model expects weight 51 on
  `ADDI > ADDI > ADDI > ADDI > ADDI`, the trace has 50. A fix inserts the slot at `index_`
  with offset 0 before the loop.
- x0 carries anti and output dependencies. True dependencies already ignore x0
  (`register_dependencies_true[0] = -1` after every instruction), anti and output ones do
  not, so `JALR x0` followed by `JAL x0` records an output dependency at distance 1. 
  This should be fixed, currently the new frontend can already handle the extra registers. 
- SLLI, SRLI and SRAI are R-type in `Opcode::getType` (`vp/src/core/common/instr.cpp`),
  while SLLIW, SRLIW and SRAIW are I-type. The tracer therefore reads the shift amount as
  an rs2 register number: `slli a5, a0, 3` reads x3 and depends on its last writer.
- Instructions without a destination register (branches, ECALL) still use their rd field
  for anti and output dependencies. In a branch that field holds immediate bits.
- ECALL reports x0 as an input, from its rs1 and rs2 fields.
- FLD and FSD do not call `log_memory_read` or `log_memory_store` (rv32 `iss.cpp`), so the
  tracer prints `[ERROR] memory operation without access` for every FSD and the trace has no
  address for double precision loads and stores. Seen in the rv32ud runs; the model does not
  describe FP instructions, so this is not a deviation entry.

Other observations.

- The upstream riscv-tests Makefile assembles `rv32ua` with `-march=rv32g_zacas_zabha`,
  which the Debian GCC 13 cross compiler rejects. `check.py` therefore compiles each test
  itself, with the `-march` from `suites.json`, and skips the Zacas tests.
- The whole rv32 and rv64 suite runs in 2.4 s (rv32) and 4.4 s (both) on 16 cores. A test
  takes about 60 ms, most of it SystemC start up.
- Mutation checks, each applied, rebuilt, run and reverted. Counting `true_weight` for
  overlapping windows (`<` to `<=` in `InstructionNode::update_weight`): the model reports
  `ADDI > AUIPC > ADDI: true_weight 2, model 1`. Swapping taken and not taken in
  `BranchNode::register_branch`: 107 model differences. Shifting every anti dependency
  one further back: 101 model differences. SLTU computed signed: the ISA suite reports
  `rv32ui-p-sltu`, and the trace check passes all its cases.


## Decision Log

- Decision: use the upstream riscv-tests for instruction correctness, as a git submodule
  at `tests/isa/riscv-tests`, pinned to bcffa2b. Rationale: they are self checking, need no
  reference simulator, and build with the bare metal compiler CI already installs.
- Decision: run the ISA tests with tracing on, as the VP is normally used. Rationale: a
  tracer bug that changes execution would then show up as an ISA failure. The cost is
  small (about 60 ms per test).
- Decision: record failing tests as known failures with a reason instead of fixing them in
  this plan drectly.
- Decision: `test32-vp` exits 1 when `tohost` reports a failure and prints `to-host: 1` on
  a pass as well. Rationale: the platform printed only failures and always exited 0, so a
  harness could not tell a pass from a program that ended another way. A program that never
  writes `tohost` still exits 0, as before.
- Decision: `test64-vp` is `test32_main.cpp` compiled against the rv64 core with
  `TEST_RV64` defined, not a copy. Rationale: the rv32 and rv64 mains differ only in the
  namespace and the address alignment helper. 
- Decision: the reference model reads `--trace-mode` output rather than a new machine
  readable trace. Rationale: `--trace-mode` already prints pc, mnemonic and operand
  registers per executed instruction, so the model needs no change to the VP, and the
  model stays independent of the tracer code it checks. The price: no values or addresses,
  so the model does not check `parameters`, the memory fields, or memory dependencies. It
  accepts a memory dependency only where one is possible. 
- Decision: the model follows the architectural meaning of each instruction and reproduces
  each current tracer difference behind a named entry in `VP_DEVIATIONS`.
- Decision: x0 counts as an input of the instruction that reads it, but carries no
  dependency. 


## Outcomes & Retrospective

(2026-10-02) Milestones 1 to 6 are done. The two checks together take about 7 seconds and
run in CI. They found one wrong rv64 instruction (SRLI), one missing rv64 CSR group, three
specification deviations shared by both cores, a crash under virtual memory, and five
tracer deviations. 
None of these is fixed yet; they are listed under Surprises & Discoveries and in
`ROADMAP.md`.

Milestones 7 and 8 are proposals;
the model does not cover memory addresses, parameters, FP or compressed instructions.


## Context and Orientation

Terms used below. An *execution sequence tree* is the VP's record of a run: one tree per
mnemonic that started a window, where a *window* (also called a sequence) is a run of up
to `depth` consecutively executed instructions and each path from a root to a node is one
window. A node's `weight` is how often its window ran; `true_weight` counts only the
occurrences that do not overlap the last counted one. *JITR* is the JSON format the VP
writes these trees in, one file per root, enabled with `-e`; `CLAUDE.md` describes every
field. The *tracer* is the code that builds the trees during simulation:
`vp/src/trace/tracer.h` (ring buffer of the last `depth` steps) and
`vp/src/trace/trace.cpp` (`InstructionNode::insert_rb`, which inserts one window and
computes its dependencies, and `update_weight`, which updates one node).

Files this plan adds or changes:

    tests/isa/check.py              builds and runs the ISA suites, compares to suites.json
    tests/isa/suites.json           suites, skipped tests, known failures with reasons
    tests/isa/riscv-tests           submodule, upstream riscv-tests at bcffa2b
    tests/isa/README.md
    tests/trace/jitr.py             loads a JITR directory, walks the trees
    tests/trace/invariants.py       the trace invariants, also a command line tool
    tests/trace/model.py            the reference model, also a command line tool
    tests/trace/check.py            runs invariants and model on every case; same_as
    tests/trace/cases.json          model flags, rv32im case, three same_as cases
    tests/trace/reference/rv32im.json
    sw/trace-test-rv32im/           every RV32IM instruction the model describes
    vp/src/platform/test32/test32_main.cpp   exit status, shared with test64
    vp/src/platform/test64/CMakeLists.txt    test64-vp
    vp/CMakeLists.txt               ctest `isa`, essential target
    .github/workflows/ci.yml        builds rv32im, runs the ISA suites

`scripts/vpbench.py` is shared by all of these: it finds a VP binary (`VP_BIN_DIR`, default
`vp/build/bin`), resolves a program (`sw/<name>` means `sw/<name>/main`), and runs the VP
with `--intercept-syscalls --output-file <dir>/`.


## Plan of Work

Milestone 1, ISA suites on rv32. Add riscv-tests as a submodule. Write `tests/isa/check.py`:
read each suite's test names from `riscv-tests/isa/<suite>/Makefrag` (the line
`<suite>_sc_tests = ...`), compile each `riscv-tests/isa/<suite>/<test>.S` with
`-march=<from suites.json> -static -mcmodel=medany -nostdlib -nostartfiles`, the include
paths `riscv-tests/env/p` and `riscv-tests/isa/macros/scalar`, and the linker script
`riscv-tests/env/p/link.ld`, into `out/isa/<suite>-p-<test>`. Run each with
`test32-vp --suppress-prompts --memory-start 2147483648 --isa imacfdsu <elf>`, stdin closed,
30 s timeout, in parallel. A pass is exit status 0 and exactly one line `to-host: 1`. Change
`test32_main.cpp` so the tohost callback records the value, prints it, and `sc_main`
returns 1 when the value is greater than 1. Acceptance: every rv32 test passes or is a known
failure.

Milestone 2, invariants. `tests/trace/jitr.py` loads `*.json` files that carry
`format_version` and walks nodes with their path and parent. `tests/trace/invariants.py`
checks, per node: `1 <= true_weight <= weight`; register counts sum to the weight;
predecessors of a pc sum to its count, and below the root are pcs of the parent; children
weigh no more than the node, and at most one child per mnemonic; nodes at the trace depth
are leaves with `PCs` equal to the register counts, nodes above it have a `children` list;
dependencies lie in `1..depth of the node`; branch outcomes per pc sum to its count. Over
the run: exactly one window ends early at every depth below the last (the one the program
end cuts off), and the root weights sum to the instruction count. Acceptance: all cases and
three embench benchmarks (crc32, nettle-sha256, statemate) pass.

Milestone 3, reference model. `tests/trace/model.py` runs the program with
`-e --trace-mode --trace-depth <d>`, parses lines of the form
`core  0: prv 3: pc    10074: ADDI a0  (x10), zero (x0), 0x32` into (pc, mnemonic,
registers, immediate), rejects anything outside RV32IM plus ECALL, any trap and any
compressed instruction, and builds for every start position every prefix window: weight,
greedy `true_weight`, counts and predecessors per pc, register numbers, register
dependencies (per window: last writer, readers and writers of each register), inputs,
outputs and, below the root, branch outcomes (taken when the next pc is not pc + 4).
Compare node by node, in both directions. Add `sw/trace-test-rv32im`. Acceptance: zero
differences on all model cases, and the three mutations in Surprises & Discoveries are each
reported.

Milestone 4, switches. `check.py` gains `"same_as"`: compare against the named case's
reference and skip the flags comparison. Acceptance: `performance-mode`, `no-dmi` and
`basic-c-performance-mode` pass.

Milestone 5, rv64. Add `vp/src/platform/test64/CMakeLists.txt` compiling
`../test32/test32_main.cpp` with `TEST_RV64`, which selects `using namespace rv64` and
`rv64_align_address`. Add the eight rv64 suites with `-march=rv64g` (`rv64gc` for `rv64uc`)
and `-mabi=lp64d`. Acceptance: every test passes or is a known failure.

Milestone 6, wiring. Register `isa` with ctest next to `trace`, with `VP_BIN_DIR` set. Add
`test32-vp` and `test64-vp` to `essential`. In CI, build `sw/trace-test-rv32im` with the
other programs, and run `tests/isa/check.py` after the trace check with
`if: ${{ !cancelled() }}`, so one log shows both results.

Milestone 7 (proposed), platform smoke runs. The `build-all` job builds every platform but
runs none. Running one terminating program on each (with the memory map each needs) would
catch a platform that crashes at start up. Not started: each platform needs its own program
or memory options, and the owner has not asked for it.

Milestone 8 (proposed), C programs in CI. `basic-c` and `no-trace` are left out of `--ci`
because the CI compiler has no C library. Ubuntu ships `picolibc-riscv64-unknown-elf`, but a
reference trace pins the exact binary, and a different compiler or C library produces a
different one. Options: commit the ELF of each C case, or build in CI with the same pinned
toolchain as locally. Not started.


## Concrete Steps

From the repository root:

    git submodule update --init --recursive tests/isa/riscv-tests
    make essential
    for p in trace-test-minimal trace-test-default trace-test-memory trace-test-output-dep \
             trace-test-prefix trace-test-minimal-rv64 trace-test-rv32im basic-asm; do
        make -C sw/$p; done
    python3 tests/isa/check.py
    python3 tests/trace/check.py

Or both through ctest (on the development machine use `/usr/bin/ctest`; the `ctest` first
on PATH is a broken Python shim):

    cd vp/build && /usr/bin/ctest -R '^(trace|isa)$'
    1/2 Test #5: trace ............................   Passed    2.21 sec
    2/2 Test #6: isa ..............................   Passed    4.37 sec


## Validation and Acceptance

Expected output:

    python3 tests/isa/check.py
    238 tests: 221 pass, 17 known failures, 0 unexpected failures, 5 skipped

    python3 tests/trace/check.py
    ok    minimal                     2 files, 5 instructions
    ...
    ok    rv32im                      46 files, 195 instructions
    ...
    ok    performance-mode            18 files, 821 instructions
    ok    no-dmi                      46 files, 195 instructions
    ok    basic-c-performance-mode    19 files, 626 instructions
    all 15 cases match the reference

    python3 tests/trace/model.py sw/trace-test-default --depth 6 --strict
    ...
    BGE: weight 171, model 172
    ...

The last command shows the known deviations; without `--strict` it reports 0 differences.
To see that a check is sensitive, change one line of the tracer or the core (for example
SLTU to a signed compare in `vp/src/core/rv32/iss.cpp`), rebuild, and run the checks: the
ISA suite reports `rv32ui-p-sltu`. Revert afterwards.


## Idempotence and Recovery

Both checks can run any number of times. `tests/isa/check.py` rebuilds a test only when its
source is newer than the ELF; delete `out/isa/` to force a rebuild. `tests/trace/check.py`
deletes the output of a passing case and keeps that of a failing one under
`out/trace-check/`. Nothing here writes outside `out/` except `--update`, which rewrites
files in `tests/trace/reference/`.


## Interfaces and Dependencies

New dependency: the riscv-tests submodule (BSD license) at `tests/isa/riscv-tests`, with
its own submodule `env`. No new Python packages: the scripts use the standard library.
The cross compiler is the one CI already installs, `gcc-riscv64-unknown-elf`.

`tests/trace/model.py` exposes `check(vp, program, out_dir, depth, flags=(), strict=False)`,
returning a list of difference strings, and `VP_DEVIATIONS`, a dict of name to
explanation. `tests/trace/invariants.py` exposes `check(trees, depth, recorded=None)`,
returning a list of problem strings. `tests/trace/jitr.py` exposes `load(directory)` and
`walk(root)`.
