## Refactor the VP so a new developer can change tracing without reading the whole simulator

This ExecPlan is a living document. The sections `Progress`,
`Surprises & Discoveries`, `Decision Log`, and `Outcomes & Retrospective` must
be kept up to date as work proceeds.

This ExecPlan must be maintained in accordance with `.claude/PLANS.md` from the
repository root.

### Purpose / Big Picture

The VP has historically grown through different projects. 
This makes it rich in geatures and actively used, but its codebase is cluttered and the structure no 
longer suited for the complexity. Especially the ISS core contains large parts of the implementation in 
one file. 
This refactor should improve the code and layout of the project to make it easier for new people or agents 
to work on it. 

You can see the refactor working at every step: the trace output of a fixed set of
programs is captured as committed reference files before any code moves, and a
test compares against them after each milestone. A milestone that changes a
single byte of that output has failed unless the plan says it should.

### Progress

- [x] (2026-09-28) Audited the repository and measured the baseline
      (file sizes, duplication, build times, simulation rate, test state).
      Findings recorded under `Surprises & Discoveries`.
- [x] (2026-09-28) Wrote this ExecPlan.
- [x] (2026-09-28) Refactored most of the tracing and core (Milestones 1 - 6). 
- [~] (2026-10-02) Milestone 7 in progress.
- [ ] Milestone 8: build system and repository hygiene.
- [ ] Milestone 9: unify the rv32 and rv64 cores, with rv32 as the reference.

### Surprises & Discoveries

### Decision Log

- Decision: performance outranks readability. A change that makes the code slightly easier
  to read or moves a setting from build time to run time is usually not worth a slowdown.
  The working budget is that no milestone may cost more than 2 percent on the
  md5sum measurement, and any cost at all has to be argued for rather than
  accepted. .

- Decision: split the tracing configuration by where it is read, rather than
  making all of it runtime configuration. Settings the tracer reads once per run,
  which is every export flag, the coverage file, the top N and the similarity
  threshold, become runtime fields on `TraceConfig`. Settings read once per
  executed instruction, which is every `trace_parameter*`, `trace_predecessor_pcs`
  and `trace_branch_outcomes` switch and the depth, stay compile time and become
  template parameters on the tracer rather than preprocessor macros. Rationale:
  the original idea would have put a runtime branch on the hot path for
  each of those, which conflicts with the decision above. A template parameter
  reads the same as a `bool` field in the source and compiles to the same code the
  `#ifdef` does now. 

- Decision: `--no-trace` is a separate instantiation, not a runtime check. The ISS
  run loop is templated on the tracer type and a `NullTracer` whose `record_step`
  is an empty inline function is instantiated alongside the real one. 
  A runtime `if (enabled)` on the hot path is the version
  that costs speed. Two instantiations cost binary size, which nothing in this
  repository is short of.

- Decision: the refactor covers the whole VP, not only the tracing subsystem.
  Milestone 7 is added for the platform layer, which is eleven `main.cpp` files of
  120 to 310 lines each that construct the same objects in the same order with
  different address maps. Rationale: The tracing
  work is the largest single improvement but it does not make a change to a peripheral or
  a new board any easier. 

- Decision: the JITR export via `-e` is the only output this plan maintains or
  tests. The dot, csv and sequence exports serve older tools that are largely
  retired. They keep working but they are not pinned by the reference check and a
  change that moves them is not a failure. 

- Decision: the checked in test suites are treated as inherited, not as a
  baseline to restore. `vp/tests` is a git submodule from the upstream project and
  the `sw` suite predates this fork's tracing work. Milestone 0 therefore builds a
  new check under `tests/trace/` rather than repairing `gdb`, `integration` or
  `libgdb`, and the new GitHub Actions workflow runs only the new check.

- Decision: measurements use md5sum from embench at trace depth 6, and the
  reference check uses the programs under `sw/` at depth 6 with one case at depth
  1. Rationale: the owner confirms md5sum is the right reference workload and that
  depth drives execution speed more than anything else, with 4 to 8 the useful
  range for testing. md5sum takes 6.56 s at depth 4,
  7.26 s at depth 6, 8.28 s at depth 8 and 16.63 s at depth 20, which is why the
  tests do not use the compiled default of 20.

- Decision: snapshots record the register file at the end of the run alongside the
  timings and the trace digest. Rationale: It
  is the cheapest functional check the VP offers, it costs nothing to collect
  because the VP already prints it, and it is what distinguishes "the refactor
  changed what the VP computes" from "the refactor changed what it costs". The
  reference check records it too, for the same reason.

- Decision: the reference check commits digests, not trace files. Each reference
  is one JSON file holding the counters, the register file, and per output file a
  size and two digests: one over the JSON re-serialised with sorted keys, one over
  the bytes. Rationale: the nine cases produce about 700 KB of JSON, which would
  be rewritten on every real change. The whole reference directory is 56 KB
  instead. Two digests rather than one is what separates a changed trace from a
  changed spelling, which during a refactor is the distinction that matters.

- Decision: `TraceConfig` carries the settings and `TraceReport` holds a reference
  to it rather than a copy of every field. Rationale: the point of the milestone is
  that a setting exists in one place. A flat `TraceReport` would have listed every
  field a second time, which is the duplication being removed.

- Decision: `Tracer::begin_step`, `step`, `previous_step` and `end_step` are defined
  in the header. Rationale: they run once per simulated instruction, hundreds of
  millions of times in a benchmark run. A call across a translation unit boundary
  there is not worth the tidiness. The cold parts, `configure`, `flush` and
  `report`, are in the .cpp.

- Decision: the hart id goes into exported filenames only for harts above zero.
  Rationale: it fixes the multicore platforms overwriting their own output without
  moving the filenames every single core run has produced so far, which would have
  invalidated the reference files and every trace anyone already has.

- Decision: the field removing tracing switches stay preprocessor macros. Rationale:
  they exist to make tracing faster during development, not to
  configure a run. A macro is the right tool for that, and it costs nothing at run
  time. 

- Decision: one concrete `InstructionNode` with no virtual functions, and the memory
  and branch payloads allocated in the same block as the node. Rationale: the six class
  hierarchy cost an indirect call per node per executed instruction with six possible
  targets, plus virtual base offsets on every access to an inherited member. Putting
  everything inline in one class instead would have grown every node from 224 to about
  500 bytes, which the memory constraint at depth 30 and above rules out. A
  payload in the same allocation costs nothing for a node that has none, needs no
  pointer, and keeps the payload next to the node rather than at a random address.

- Decision: `InstructionNode::dependencies_true_` stays `std::array<bool, DEPTH>`.
  Rationale: measured. A `uint64_t` mask shrank the node by 16 bytes, freed no memory
  at all, and cost about half a percent of run time, because the two writes per node
  visit become a read-modify-write of one word instead of two independent byte stores.

### Outcomes & Retrospective

Not started. Add an entry at the end of the refactor comparing what was
achieved against the acceptance stated, and a final entry
comparing the result against the `Purpose / Big Picture` section.

### Context and Orientation

This section assumes you know nothing about this repository.

The VP is a simulator for RISC-V programs, written in C++17 on top of SystemC.
SystemC is a C++ library for modelling hardware; the parts that matter here are
that the program's entry point is `sc_main` instead of `main`, that
`sc_core::sc_start()` runs the simulation, and that the simulator core runs as a
SystemC thread. VP stands for Virtual Prototype.

The simulator is built once per target board. Each board is a directory under
`vp/src/platform/` with a `main.cpp` that constructs the core, the memory, the
bus and the peripherals, wires them together, and calls `sc_core::sc_start()`.
There are eleven of them. `vp/src/platform/tiny32/tiny32_main.cpp` is the one
used for almost all tracing work; it is a bare core with memory, a timer and a
syscall handler and nothing else. `make essential` builds four of the eleven:
`riscv-vp` (from `vp/src/platform/basic/`), `linux32-vp`, `tiny32-vp` and
`microrv32-vp`.

The instruction set simulator, abbreviated ISS, is the object that executes
instructions. There are two, one per register width:
`vp/src/core/rv32/iss.{h,cpp}` for 32 bit and `vp/src/core/rv64/iss.{h,cpp}`
for 64 bit. The class is called `ISS` in both, inside namespaces `rv32` and
`rv64`. Its main loop is `ISS::run_step()`, which calls `ISS::exec_step()`,
which fetches one instruction word, decodes it into an `Instruction` and an
`Opcode::Mapping`, and executes it in a `switch` over the opcode.
`Opcode::Mapping` is an enumeration of every instruction the VP knows, declared
in `vp/src/core/common/instr.h`.

On top of plain simulation, this fork records what the program executed. The
record is a set of trees, one per instruction that ever started a window. A
window is a run of instructions that were executed back to back. Each node of a
tree is one dynamically executed instruction, and the path from a root to a node
is one window the program actually ran, with a count of how many times it ran.
The repository calls this an execution sequence tree; 
the structure is similar to a trie or a prefix tree. The exported form is called
JITR and is one JSON file per root instruction. The file
`vp/src/core/common/trace.h`, 1316 lines, declares the node classes; the file
`vp/src/core/common/trace.cpp`, 1934 lines, implements them;
`vp/src/core/common/trace_analysis.{h,cpp}` holds the scoring and similarity
helpers. `CLAUDE.md` in the repository root has a field by field table of the
JITR format, and `docs/glossary.md` defines the terms.

The mechanism inside the ISS works like this. `ISS` owns a fixed size ring
buffer, `std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_steps`,
and an index `ring_buffer_index`. At the top of `exec_step()`, before the new
instruction runs, the entry that is about to be overwritten is the oldest one in
the buffer; the ISS finds or creates the tree for that instruction's opcode in
`std::list<InstructionNodeR> instruction_trees` and calls
`found_tree->insert_rb(last_executed_steps, ring_buffer_index)`, which inserts
the whole window. At the bottom of `exec_step()`, after the instruction has run,
the ISS fills the current slot of the ring buffer with everything it learned:
the opcode, the program counter, the register numbers, whether memory was read
or written and at what address, the decoded immediate, the branch outcome, and
so on. `ExecutionInfo`, declared in `trace.h`, is that record. It already
contains no 32 or 64 bit specific type. This is the seam the refactor uses.

At the end of the simulation, the platform's `main.cpp` calls `core.show()`.
`ISS::show()` prints the registers, then calls `flush_ringbuffer()` to insert the
windows still in the buffer, then runs the analysis: for every tree it calls
`extend_top_paths` to find the highest scoring windows, sorts them, filters out
windows too similar to ones already kept, prints the best four, and writes
whichever exports were requested. Then, if `--interactive` was passed, it enters
a command loop where you can reload a shared library of scoring functions with
`r`, re-run the analysis with `a`, prune the trees with `p`, and export with `d`
or `e`. All of that is inside `ISS::show()` and its `output_*` helpers, in both
architectures.

The tracing options are command line options, declared in
`vp/src/platform/common/options.cpp` as members of a class `Options` that every
platform's option class derives from. The values then have to be copied onto
the `ISS` object by hand, one assignment per option, in each platform's
`main.cpp`. `vp/src/platform/tiny32/tiny32_main.cpp` has nine such
assignments plus a call to `set_trace_depth`. Five platforms have none.

Build and run commands, from the repository root:

    make essential
    ./vp/build/bin/tiny32-vp --intercept-syscalls <program> --output-file <dir>/ -e

`-e` is the flag that writes the JITR JSON files. `<dir>` must exist and the
trailing slash matters, because the output filename is built by string
concatenation. Tests run from the build directory:

    cd vp/build && ctest

Note for this machine: the `ctest` first on `PATH` is a broken Python shim at
`/home/jz/.local/bin/ctest`. Use `/usr/bin/ctest`.

### Milestones

#### Milestone 0: a safety net that fails when the trace changes (done)

Nothing in this repository told you whether a change altered the trace. This
milestone built that, and fixed what the new check immediately found. You can now
make any edit and know within two seconds whether the JITR output moved.

What exists at its end. `tests/trace/` holds nine cases in `cases.json` and a
committed reference per case in `reference/`. `tests/trace/check.py` runs each case,
reduces its output directory to one digest per file, and compares that against the
reference along with the counters the VP reported and the register file at the end of
the run. It is registered with ctest as `trace` and runs in 1.35 s. The reference
directory is 56 KB, because what is committed is digests rather than trace files:
per output file a size, a digest over the bytes, and a digest over the JSON
re-serialised with sorted keys. The two digests separate a changed trace from a
changed spelling, which is the distinction a refactor needs.

The new tests live at `tests/trace/`, not under `vp/tests/`, because `vp/tests` is a
git submodule pointing at the upstream project and nothing committed there would
belong to this repository.

`scripts/snapshot.py` measures cost rather than correctness. It runs the VP over a
benchmark set from `scripts/benchmarks.json` and writes
`docs/snapshots/<date>-<short commit>.json` holding per workload the wall time, peak
memory, instruction and cycle counts, tree count, the register file at the end of the
run, and a digest of the trace, plus the build settings that make two snapshots
comparable. `--compare OLD NEW` prints the change per workload and refuses to present
it as a result when the two runs used different settings. Both tools share
`scripts/vpbench.py`, which finds the binaries, runs them, reads their report, and
digests their output.

Acceptance, as met. `cd vp/build && ctest -R trace` passes in 1.35 s. Changing the
shift constant in the `InstructionNode` constructor from 6 to 7 makes all nine cases
fail and names the files that differ; reverting makes them pass. Perturbing a register
value in a reference file makes the check report `hart 0 x15: 0x99 -> 0x2a`.
`riscv-vp` exits 0 on all six trace programs and is clean under valgrind, where it
previously reported five invalid frees and died. 19 of 22 `sw` programs build and run,
up from 0. The first snapshot is committed at
`docs/snapshots/2026-09-28-c56790.json`.

#### Milestone 1: move analysis and export out of the ISS (done)

This milestone moved code and changed nothing else. At its end the tracing, analysis and
export code exists once instead of twice.

What exists at its end. `vp/src/trace/` is a CMake library named `trace`. It holds the
tracer (`trace.{h,cpp}`, moved from `vp/src/core/common/`), the analysis helpers
(`analysis.{h,cpp}`, moved from `trace_analysis.{h,cpp}`), one exporter per format
(`export_dot.cpp`, `export_csv.cpp`, `export_json.cpp`), the report driver
(`report.{h,cpp}`) and the interactive command loop (`interactive.cpp`). It depends on
`core-common` for the instruction decoding and on nothing else in the VP, and
`core-common` no longer depends on it, so a change to the tracer rebuilds the tracer and
the two ISS translation units rather than the whole tree.

The boundary is one struct. `TraceReport` in `vp/src/trace/report.h` holds what the report
needs from the core that produced the trees: the trees themselves, three counters, the
memory access map, and the output settings. An ISS fills it in and calls
`run_trace_report`. `flush_ring_buffer` is a free function taking the trees, the step
array and the index.

What `ISS::show()` is now: 34 lines that flush the ring buffer, print the hart id, the
register file, the program counter and the instruction count, fill in a `TraceReport` and
hand it over. The 32 and 64 bit versions are byte for byte identical, which is the point:
the next change to the report cannot land in one and miss the other.

What was deliberately not done. The dot and csv exporters still redirect `std::cout` at a
file while they run, because the node classes write through `std::cout` themselves
(`InstructionNodeR::tree_to_dot`, `to_csv`). Threading a stream through the node hierarchy
belongs with rewriting that hierarchy, which is Milestone 5. The two JSON exporters, which
build a value and write it in one place, take their own stream and do not touch
`std::cout`.

Acceptance, as met. All nine reference cases pass with no reference file changed. Beyond
that, the output of all four export formats was captured from the pre-move binaries over
three programs, 105 files, and compared against the post-move output: all 102 exported
files are byte identical, including the dot graphs, the tree csv, the sequence and variant
JSON, the coverage csv and the memory map. Standard output differs only in the two
`restored cout` lines that the JSON exporters no longer print.

The numbers. `rv32/iss.cpp` went from 3074 to 2324 lines and `rv64/iss.cpp` from 3069 to
1.    Compiling `rv32/iss.cpp` went from 10.65 s and 590 MB peak to 7.30 s and 517 MB. A
rebuild of `essential` after touching the tracer header went from 41.78 s to 33.56 s. The
md5sum snapshot moved by a median of -0.8 percent, within noise, with every trace digest
and register file matching.

What is still duplicated. `diff` over the two `iss.cpp` files with digits stripped reports
1147 changed lines, down from 1440. What remains is the fetch, decode and execute path,
where the differences are real: register width, sign extension, the `W` suffixed
instructions and the CSR layout. Unifying that is the deferred plan, not this one.

#### Milestone 2: one configuration object, and every option on every platform (done)

At its end adding a tracing option means editing two files instead of seven, and every
platform honours every option.

What exists at its end. `TraceConfig` in `vp/src/trace/config.h` holds what to record and
what to write. `Tracer` in `vp/src/trace/tracer.h` owns the config, the ring buffer and
the trees, and gives a core three calls per executed instruction: `begin_step` inserts the
window that ends at the slot about to be overwritten, `step` hands out the slot to fill in,
and `end_step` advances. All three are defined in the header, because they run once per
simulated instruction. `Options::trace_config(hart_id)` in
`vp/src/platform/common/options.cpp` turns the parsed command line into a `TraceConfig`.

What a platform does now, in full:

    core.tracer.configure(opt.trace_config());

replacing between zero and ten hand copied assignments plus a `set_trace_depth` call. The
two multicore platforms pass each core's id, and the two Linux platforms pass the loop
index.

What the `ISS` lost: `instruction_trees`, `last_executed_steps`, `ring_buffer_index`,
`output_filename_string`, `output_filename`, `input_filename`, `path_hashes`,
`coverage_csv_file`, `coverage_top_n`, `coverage_similarity_threshold`, `output_as_dot`,
`output_as_csv`, `output_as_json`, `output_full_export`, `output_coverage_csv_enabled`
and `interactive_mode`. It gained one member, `Tracer tracer`. `suppress_prompts` stayed,
because the ISS reads it itself for the misaligned access and ISA extension prompts. The
constructor went from five parameters to two.

The multicore platforms overwrote their own output, because the exported filename carried
no hart id. `TraceConfig::hart_id` is now part of every exported filename, as `-hart<n>`
for harts above zero. Hart 0 keeps the old names, so single core output did not move.
`tiny32-mc` on `sw/basic-multicore` now writes 12 files per hart instead of 12 in total.

Acceptance, as met. All nine reference cases pass with no reference file changed, and the
four export formats over three programs are still byte identical to the capture taken
before Milestone 1. `grep -c tracer.configure` returns 1 for every platform and 2 for the
two multicore ones; no platform mentions a tracing field by name. `linux32-vp --help`
lists the tracing options, which it did not before. The `sw` suite is unchanged at 20 of
1.  Median cost change +0.5 percent against Milestone 1.

A note on that number. Two snapshots of the same build taken back to back often differ by a
median of +0.7 percent, with per workload swings from -1.3 to +1.7 percent, so anything
under about 2 percent on the same machine is not measurable as a change.

#### Milestone 3: node storage, measured (done)

This milestone was planned as turning the compile time tracing switches into runtime
configuration. It is not that. The new position is that the switches which only drop
fields from the trace exist to make tracing faster during development, and that they should
stay macros. 

Acceptance, as met. All nine reference cases pass with no reference file changed.

#### Milestone 4: split exec_step (dropped)

Dropped. Splitting the instruction definitions by RISC-V
extension is not wanted: having every instruction in one file is convenient for the
work that actually happens there, and the split offered no performance gain to pay for
the churn. What may happen instead is a split by pipeline stage, alongside the planned
configurable pipeline, and an optimisation of decode, which is one of the real
bottlenecks. 

#### Milestone 5: one node class instead of six (done)

At its end `vp/src/trace/trace.h` is 767 lines instead of 1302 and `trace.cpp` 1594
instead of 1925, each exporter exists once, and the embench set at depth 6 runs 9.8
percent faster on 4.4 percent less memory with the JITR output unchanged.

What was there. `InstructionNode` was an abstract virtual base. `InstructionNodeR` and
`InstructionNodeLeaf` derived from it virtually, `MemoryNode` and `BranchNode` were
separate bases, and `InstructionNodeMemory`, `InstructionNodeMemoryLeaf`,
`InstructionNodeBranch` and `InstructionNodeBranchLeaf` combined them. Six concrete
classes, four near identical `to_dot` implementations totalling about 400 lines, two
`get_pc` implementations, six `get_node_type` overrides each returning a constant, and
an indirect call per node per executed instruction with six possible targets.

What is there now. One concrete `InstructionNode` with no virtual functions. One byte,
`node_type`, says what it is, and it holds the value the JITR export already reported
as `type`: LEAF for a node that ends a window, MEMORY for a load or a store, BRANCH for
a conditional branch or a jump. The memory and branch payloads are constructed directly
after the node in the same allocation, so a node that needs neither pays nothing and a
node that needs one follows no pointer to reach it. `insert` and `update_weight` are
direct calls the compiler inlines into `insert_rb`.

The five changes, each measured on its own over the six workload embench set at trace
depth 6, against a baseline measured twice:

| change | median | every workload |
| ------ | ------ | -------------- |
| one node class, no virtual dispatch, payload in the same block | -4.0 to -4.5 % | faster |
| trees indexed by opcode instead of searched for | -1.8 % | faster |
| children in one contiguous table with the opcode inline | -4.0 % | faster |
| dependency offsets by bit reversal instead of a position walk | -1.1 % at depth 6 | 5 of 6 faster |
| all four together | -9.8 % | -7.9 to -15.2 % |

Peak memory over the set went from 169.9 MB to 162.4 MB. A plain node is 208 bytes
instead of 224, a leaf 208 instead of 248 plus a 48 byte map and a 48 byte tree node per
pc, a load or store node 400 instead of 416, a branch node 312 instead of 328.

The bit reversal is the change that matters at the depths where the VP is expensive.
md5sum, before and after, at four compiled depths:

| trace depth | position walk | bit reversal | change |
| ----------- | ------------- | ------------ | ------ |
| 4  | 6.02 s  | 5.99 s  | -0.5 %  |
| 8  | 7.56 s  | 7.40 s  | -2.1 %  |
| 12 | 9.65 s  | 8.85 s  | -8.3 %  |
| 20 | 14.86 s | 12.25 s | -17.6 % |
| 32 (compiled) | 27.07 s | 19.17 s | -29.2 % |

For each instruction of a window, the old code walked every earlier position of the
window to turn "position j wrote or read this register" into "the dependency reaches
j instructions back". That is the term that grows with the square of the depth.
Reversing the bit order of the position mask produces every offset at once, in about
fifteen instructions rather than four per position. Above a compiled depth of 64 the
masks no longer fit in a word and the walk is still used.

What changed in the output. Nothing in the JITR export: all 45 files across three
programs are byte for byte what they were, as are the counters and the register file at
the end of every run. 

How it was verified. `tests/trace/check.py` after every step, with no reference file
touched. All four export formats captured for three programs before the milestone and
compared after each step. For the bit reversal, the JITR export of four programs at
eight trace depths from 2 to 20, 454 files, compared against a build forced onto the
position walk: identical. Pruning, the dot export of pruned nodes and a scoring library
loaded through the interactive mode exercised by hand.

#### Milestone 6: survey the whole VP and remove deprecated features

Dropped (for now). Retiring features is off the table: working features stay. 

The VP outside `vp/src/vendor` and `vp/src/lib` is 28,750 lines:

| area | lines | note |
| ---- | ----- | ---- |
| `platform` | 10,074 | eleven boards, several pairs near identical |
| `core/common` | 5,692 | decoding, gdb, mmu, elf loading |
| `core/rv32` | 4,383 | |
| `core/rv64` | 4,314 | 3,300 of its lines are identical to rv32 |
| `trace` | 3,587 | rewritten in Milestone 5 |
| `util` | 660 | |

##### What the survey found, and what was done about it

**Direct memory access was off by default, and it is worth 30 percent.** `--use-dmi`
lets instruction fetch and load/store read the plain memory ranges directly instead of
through a TLM transaction on the bus. Nothing in the repository turned it on: not the
run command in `CLAUDE.md`, not `scripts/snapshot.py`, not the reference cases. It is
now the default, with `--no-dmi`, `--no-instr-dmi` and `--no-data-dmi` to turn it off.
Measured over the six workload embench set: 29.7 percent faster, every workload between
20 and 33 percent. Instruction fetch alone is worth 18 points of it, load and store the
rest.

It is safe because every platform builds its direct access range from plain memory,
never from a peripheral: `mem.data` in eight of them, `dram.data` and `flash.data` in
hifive. Anything outside those ranges still goes through the bus. Verified: all nine
reference cases produce byte identical output with it on and with it off, on
`tiny32-vp`, `riscv-vp` and `tiny64-vp`, including the reported cycle count, and all 102
files of all four export formats are unchanged.

**The predecessor map was a tree lookup and an allocation per node per instruction.**
`RegisterSetCounter::predecessors` was a `std::map<uint64_t, uint64_t>`. Measured over
md5sum: 99.1 percent of entries have exactly one predecessor, covering 98.6 percent of
updates, and the most any entry had was four. The first predecessor is now stored in the
entry and the rest in a vector that stays empty.

**Counting cycles as a number instead of a time: measured, reverted.**
`_compute_and_get_current_cycles`, called once per instruction, divided a `sc_time` by
`cycle_time`. Replacing that with a counter incremented from a second per opcode table
of cycle counts was 3 percent slower over the whole set, with all six workloads slower,
because the second 1.3 KB table displaces the hot fields of the ISS. The division stays.
The function is now called `get_current_cycles`, which is what it does.

**`subdirs()` is removed in CMake 4.** Eleven calls across five files, replaced with
`add_subdirectory()`.

##### What the survey found and left alone, with the evidence

**rv32 and rv64 are 3,300 identical lines.** `iss.cpp` 74 percent identical line for
line, `iss.h` 84, `csr.h` 77, `syscall.cpp` 86, `syscall.h` 94, `syscall_if.h` 85,
`mmu.h` 100. Unifying them is the largest maintainability item in the VP and the
roadmap already carries it as needing instruction level equivalence checking first.
Not started here, because it is a milestone of its own and it needs a 
decision on how far to go: a shared template, a shared implementation file with the
width as a parameter, or leaving the split and sharing only what is exactly equal.

**The platform mains are near clones.** `tiny32_main.cpp` and `tiny64_main.cpp` differ
in 6 of 133 lines, `linux_main.cpp` and `linux32_main.cpp` in 2 of 243,
`tiny32-mc/mc_main.cpp` and `tiny64-mc/mc_main.cpp` in 3 of 140. The differences are the
architecture namespace, the address alignment helper and one commented out line. A single
template over the architecture would replace three pairs.

**Peak memory is not mostly the trees.** md5sum at depth 20: 140 MB floor before the
trees grow, 22 MB of trees, and 12 MB more while the JITR export builds its json tree in
memory before writing it. Streaming the export would return that 12 MB. Shrinking a node
by 32 bytes, which Milestone 5 did by dropping `occurrence`, moved peak memory by
400 KB, so the per pc statistics and not the nodes are what tree memory is made of.

**The per pc statistics are the next real target, for both time and memory.** Measured
over md5sum at depth 6: 74.6 percent of nodes run at exactly one pc and those take 32.4
percent of the updates, but the 3 percent of nodes with more than six pcs take 31
percent, and the largest has 177. So neither a plain linear scan nor the current
`std::map` fits: the shape that does is the first entry inline in the node and the rest
in a sorted array searched by halving. That removes the last tree lookup on the insert
path and the 112 bytes of allocation per pc.

**The TLM quantum default synchronises on every instruction, and costs half the run
time.** `--tlm-global-quantum` defaults to 10 ns, which is exactly one cycle, so the core
hands control back to the SystemC scheduler after every single instruction and pays a
coroutine context switch for it. md5sum at depth 6: 4.39 s at the default, 3.78 s at
100 ns, 2.46 s at 1000 ns, 2.27 s at 10000 ns, 2.21 s at 100000 ns.

The default stays at 10 ns, because raising it delays when the core observes an interrupt
or a peripheral and so changes what an interrupt driven workload executes. The owner asked
for it behind a switch instead: `--performance-mode` collects every setting that trades
timing accuracy for speed, which today is direct memory access on and the quantum at
100000 ns. Over the embench set it is 45.6 percent faster, median, with every workload
between 32 and 59 percent. All nine reference cases produce byte identical
output with it, and a switch given after it still wins in either order, so
`--performance-mode --tlm-global-quantum 10` keeps the accurate quantum and
`--performance-mode --no-dmi` keeps the bus. The next such setting goes in
`Options::apply_performance_mode` and in the switch's help text together.

**Decode costs about 3 percent.** `Instruction::decode_normal`
switches on the opcode field, then on funct3, then on funct7 or funct6, and then verifies
the whole word against a mask and encoding. Measured by decoding every
instruction twice and taking the difference: one decode is 0.11 to 0.19 s of md5sum's
4.41 s at depth 6, so between 2.5 and 4 percent, and the upper figure includes
constructing a second `Instruction`. A perfect decoder would therefore buy about three
percent here. It is worth more in a VP that does not trace, where the per instruction
cost it is a share of is much smaller.

**Where md5sum's time actually goes, with direct memory access on.** Depth 2 is 3.26 s,
depth 4 is 3.88 s, depth 6 is 4.48 s, depth 20 is 9.78 s. So about 0.3 s per depth step
of tracing, and a 3.26 s floor that is fetch, decode, execute, the quantum keeper and
the per instruction bookkeeping. 5 million instructions in 3.26 s is 650 ns each, which
for an interpreter reading memory directly is dominated by the scheduler synchronisation
above, not by the instruction semantics.

**Two dead headers.** `vp/src/core/rv32/timing/timing_simple.h` and `timing_external.h`
hold a copy of the per opcode cycle table that nothing includes. Left in place: working
features stay, and these are a stub for a planned feature rather than a dead export.

**CMake defaults to a Debug build while the Makefile defaults to Release.** Someone who
configures by hand gets `-g3` and no optimisation, which for an interpreter is several
times slower, and no warning says so. Left alone because the mirror image trap, a Release
build for someone who wanted to debug, is just as bad. Worth a status message.

**`file(GLOB LIST_DIRECTORIES true *)` picks up the platform directories.** Adding a
board does not re-run CMake, so the build silently ignores it until something else
triggers a configure.

**75 TODO, FIXME and XXX markers** outside the vendored code.

#### Milestone 7: the whole VP reviewed again, this time by a fan out

Survey the whole VP for refactoring, performance and design
improvements, and think about the design rather than only the code.

Nothing below has been through the adversarial pass. The two findings that touched work done in
Milestone 6 were checked by hand instead, and both were real.

##### Checked by hand and fixed

**Instruction fetch by direct memory access skipped address translation.** `InstrMemoryProxy`
in `vp/src/core/rv32/mem.h` and `rv64/mem.h` loaded through the DMI with the virtual address,
while the bus path did `_raw_load_data(v2p(addr, FETCH))`. The rv64 copy still carried the
commented out assert saying the proxy does not support virtual memory. Harmless while the fast
path was opt in, and a silent break of `linux32-vp` and `linux-vp` the moment Milestone 6 made
it the default. The proxy now delegates to the translating interface whenever satp is not BARE.
Every platform passes that interface. One load per fetch, measured inside the noise.

**Release builds ran their asserts.** `CMAKE_CXX_FLAGS_RELEASE` was `-O3`, which replaces
CMake's own `-O3 -DNDEBUG`. All 237 asserts ran in the default build, including two in
`get_current_cycles` (one of them a modulo of two `sc_time` values and one a 64 bit modulo), one
per register file access and one per instruction on the pc alignment. Worth 1.6 percent over the
embench set. `RelWithDebInfo` is now `-O2 -g` so there is still a fast build with the checks on.

**The run timeout in `scripts/vpbench.py` was dead.** Declared as a parameter, never read, and
both reads in the function block without a limit, so a VP that does not terminate hung the
reference check forever. That is not hypothetical here: three `sw` programs do not terminate on
`tiny32-vp`. A watchdog now kills the run and the result says so.

**`make codestyle` formatted 870 files, none of them a header, including SystemC's.** In
`find . ... -o -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -print` the `-print` binds to the
last branch only. It now formats the 170 first party sources under `vp/src`.

##### Checked by hand, not fixed

**`tiny32` and `tiny64` map 128 MB of RAM over the CLINT and the syscall region.**
`tiny32_main.cpp:28` sets `mem_size` to 128 MB from address 0, so memory ends at `0x07FFFFFF`,
while the CLINT sits at `0x02000000` and the syscall region at `0x02010000`, both inside it. The
comment on that line says the size was chosen "to place it before the CLINT", which 32 MB would
do and 128 MB does not. `SimpleBus::decode` (`vp/src/platform/common/bus.h:43`) returns the first
port whose range contains the address and memory is `ports[0]`, so a CLINT access has always
been answered by RAM. Data direct memory access does the same thing one layer earlier, because
the DMI range is the whole 128 MB.

This is very likely the answer to the question Milestone 0 left open: `sw/blocking-sleep`,
`sw/busy-wait-sleep` and `sw/basic-asm` do not terminate on `tiny32-vp` but run on `riscv-vp`.
A program that waits on `mtime` reads RAM, sees zero, and waits forever. Programs that use
`--intercept-syscalls`, which is every reference case, never touch either region, which is why
the suite is green.

Two fixes, both changing what a program observes:
  - Decode the CLINT and the syscall region before memory, and split the DMI range around them.
    Keeps `mem_size`, so the initial stack pointer and every reference digest stay put.
  - Shrink `mem_size` to `0x02000000`. Simpler and matches the comment, but it moves the stack
    pointer, so every reference file and every trace anyone holds moves with it.
The first is the one to take, but it changes simulated behavior.

##### Not surveyed

Not finished: the two instruction set simulators, decoding, the platform
layer, the trace library reviewed as fresh code, and the cross cutting design questions. The
cross cutting one is important, because it is the one that asks what
the module boundaries should be and counts what a developer must touch to add an instruction, a
board, a peripheral or a trace field.

#### Milestone 8: build system and repository hygiene

At its end a newcomer can configure, build and run without reading the Makefile, and
nothing in the build refers to something that does not exist.

Replace the deprecated `subdirs()` calls with `add_subdirectory()`. Replace every
`file(GLOB_RECURSE HEADERS ...)` with explicit source lists, because a glob is
evaluated when you configure and not when you build, so a new file is silently missing
until someone re-runs CMake. Declare the `COLOR_THEME` option that
`vp/src/core/rv32/CMakeLists.txt` reads, or delete the branch that reads it; as it
stands it would give the library and the platform different class layouts if anyone
ever set it. Turn `vp/src/scoring_functions/` into an ordinary target in the main
build instead of a nested `project()` built in source by a separate `make
scoring-functions`, and delete that Makefile target (it was originally designed to be recompiled and reloaded at runtime into the VP). 
Check whether to delete `vp/src/core/rv32/timing/`,
which nothing compiles, and delete the stale `#include "trace.h"` in
`vp/src/core/common/instr.cpp`.

Collapse the eight root level `run*.sh` scripts into one `scripts/run-benchmarks.sh`
that takes the benchmark directory as an argument instead of hardcoding
`../embench-iot/bd/src`, and reads the workload list from
`scripts/benchmarks.json`, which already exists and which `scripts/snapshot.py`
already uses.

Strip the byte order mark from `.clang-format`. Run `make codestyle` once, on its own
commit, so the formatting change does not hide a real change inside a diff. Triage the
107 `TODO`, `FIXME`, `HACK` and `XXX` markers: delete the ones that are done, and turn
the rest into either a `ROADMAP.md` entry or a comment saying what would have to be
true to remove it.

Update `README.md` to match the code, update `docs/glossary.md` with the terms this
refactor introduces, and set the `Refactor the repository` row in `ROADMAP.md` to
`done`.

Acceptance. A fresh clone, `git submodule update --init --recursive`, `make essential`,
`cd vp/build && ctest -R trace` all succeed with no manual steps beyond what the README
states. `grep -rn 'subdirs(' vp` returns nothing. `grep -rn 'GLOB' vp/src --include=CMakeLists.txt`
returns nothing. The README's option table matches `tiny32-vp --help`.

### Validation and Acceptance

The overall acceptance is behavioral and can be checked by a person who has
just cloned the repository.

First, the VP computes and records the same thing. From a clean clone at the end
of this work, `make essential` then `cd vp/build && ctest -R trace` passes, and the
reference files under `tests/trace/reference/` are identical to the ones committed at
the end of Milestone 0, including the register file each one records. Any file that
had to change has a `Decision Log` entry saying which milestone changed it and why,
and a `docs/changelog.md` entry.

Second, and first in priority, the simulator is no slower. Take a snapshot before
and after with `python3 scripts/snapshot.py --repeat 3` and compare them with
`--compare`. The median change must be within 2 percent and no workload may be more
than 5 percent slower. Measured noise between two back to back snapshots of md5sum
at `--repeat 3` is 0.3 percent, so 2 percent is a real signal rather than jitter.
Peak memory must not rise. A milestone that costs more than this is not accepted on
the grounds that the code reads better.

Third, the iteration cost is lower. `touch` on the tracer's main header
followed by a parallel `essential` build takes less than 20 s, against the
41.78 s baseline. No single translation unit takes more than 5 s or peaks above
300 MB, against the 10.65 s and 590 MB baseline for `vp/src/core/rv32/iss.cpp`.

Fourth, the code is where a newcomer would look for it. `grep -c` finds one
definition of each exporter rather than four. No tracing or export symbol
appears in `vp/src/platform/*/main*.cpp`.

Fifth, the test suite says something true. `/usr/bin/ctest` runs `trace`,
`unit`, `libgdb`, `gdb`, `integration` and `sw`; the suites that need a RISC-V
cross compiler report a clear skip when it is absent instead of a CMake error;
and the GitHub Actions workflow is green.

### Interfaces and Dependencies

No new runtime dependency. The VP keeps SystemC, Boost `iostreams`,
`program_options` and `log`, the vendored Berkeley SoftFloat, and the nlohmann
JSON library. Milestone 1 vendors one new header, `doctest.h`, for tests only,
at `vp/src/vendor/doctest/doctest.h`, which matches how SoftFloat is already
vendored. It is MIT licensed and is a single header with no build step.

The directory layout at the end of this work:

    vp/src/core/common/     ISA decode, memory, CSR, CLINT, ELF, GDB. No trace types.
    vp/src/core/rv32/       32 bit ISS. iss.{h,cpp} plus exec/ per extension.
    vp/src/core/rv64/       64 bit ISS, same shape.
    vp/src/trace/           new library `trace`
      step.h                ExecutionInfo, the one record the ISS produces
      config.h              TraceConfig
      tracer.{h,cpp}        Tracer: ring buffer, tree list, the ISS facing API
      node.{h,cpp}          InstructionNode
      analysis/             Path, scoring, similarity, filtering
      export/               json.cpp, dot.cpp, csv.cpp, report.cpp
      interactive.cpp       the post simulation command loop
    vp/src/platform/        eleven boards, reshaped in Milestone 7 around a builder
    tests/trace/            the reference check, its cases and its reference digests
    tests/unit/             doctest unit tests
    scripts/                vpbench.py, snapshot.py, benchmarks.json, run-benchmarks.sh
    docs/snapshots/         committed measurement snapshots

The signatures that must exist at the end of Milestone 2. `ExecutionInfo` is
unchanged from what `vp/src/core/common/trace.h` declares today, moved to
`vp/src/trace/step.h`:

    // vp/src/trace/config.h
    struct TraceConfig {
      bool enabled = true;
      uint32_t trace_depth = INSTRUCTION_TREE_DEPTH;
      bool export_dot = false;
      bool export_csv = false;
      bool export_sequences = false;
      bool export_full = false;
      bool export_coverage_csv = false;
      std::string coverage_csv_file;
      unsigned coverage_top_n = 10;
      float coverage_similarity_threshold = 0.2f;
      bool interactive = false;
      bool suppress_prompts = false;
      std::string output_directory;
      std::string input_program;
      std::string scoring_library = "./vp/build/lib/libfunctions.so";
    };

    // vp/src/trace/tracer.h
    class Tracer {
      public:
        explicit Tracer(TraceConfig config = {});
        void configure(TraceConfig config);
        void begin_step();
        void record_step(const ExecutionInfo &step);
        void flush();
        const std::list<InstructionNodeR> &trees() const;
        const TraceConfig &config() const;
    };

    // vp/src/trace/export/report.h
    void report(const Tracer &tracer, std::ostream &log);

    // vp/src/trace/export/json.h
    void export_jitr(const Tracer &tracer);

    // vp/src/trace/export/dot.h
    void export_dot(const Tracer &tracer);

    // vp/src/trace/export/csv.h
    void export_csv(const Tracer &tracer);

    // vp/src/trace/interactive.h
    void run_interactive(Tracer &tracer);

What Milestone 5 left in place of the six class hierarchy. One class, with the
exporters still members of it rather than free functions, because each of them
recurses over the tree and reads members the tree owns:

    // vp/src/trace/trace.h
    class InstructionNode {
        static InstructionNode *create(Opcode::Mapping op, uint64_t parent_hash, bool leaf);

        NODE_TYPE node_type;          // LEAF | MEMORY | BRANCH, one byte
        bool is_leaf() const;
        bool has_memory() const;
        bool has_branch() const;
        MemoryNode *memory();         // the payload after the node, if has_memory()
        BranchNode *branch();         // the payload after the node, if has_branch()

        void insert_rb(const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> &, uint32_t);
        InstructionNode *insert(const StepInfo &);
        void update_weight(const StepInfo &);

        nlohmann::ordered_json to_json();
        void to_csv(const CsvParams &);
        void tree_to_dot(uint64_t total_instructions, float branch_threshold);
    };

`StepInsertInfo` and `StepUpdateInfo` are one `StepInfo`, so `insert` passes the
object it was given instead of rebuilding 144 bytes of it per node per executed
instruction.

The interface a scoring library compiles against is `vp/src/trace/score.h`:
`ScoreParams`, `ScoreFunction` and `SF_BATCH_SIZE`, and nothing else.

Include hygiene rules the new library must follow, because they are the reason
the build is slow today. `vp/src/trace/step.h` and `config.h` must not include
the JSON library, so the ISS headers do not pull it in. Only the files under
`vp/src/trace/export/` include it, and only in their `.cpp` files where
possible. `vp/src/core/rv32/iss.h` includes `vp/src/trace/tracer.h` and nothing
else from the trace library.

The one behavioral contract that must not move without a version bump:
`TRACE_FORMAT_VERSION`, currently `"1.2"`, and the field layout that `CLAUDE.md`
documents. The reference test in Milestone 0 is what enforces it. The RETrace
frontend at `../EX-T-Viz/` reads that format, and so does the Instruction Set
Extender at `../ISE/`; neither is in scope for this plan, and both break if the
format changes silently.
