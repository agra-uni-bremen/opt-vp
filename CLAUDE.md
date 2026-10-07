# VP Core Agent Guide

This file contains repository-specific instructions for coding agents working on
the VP. 
This repository contains the SystemC RISC-V Virtual Prototype and its tracing extension (sometimes called RISC-V Opt VP). 

It simulates and traces RISC-V applications with the goal of identifying hotspots and bottlenecks.
The generated traces using the JITR format are then consumed by the RETrace frontend tool. 

It is a fork of https://github.com/agra-uni-bremen/riscv-vp. 

Note: https://github.com/ics-jku/riscv-vp-plusplus is a fork of the original risc-v vp which contains many performance improvements. It also contains numerous changes and additions, not all of which we want to adopt in our VP. It was therefore not merged. 

**Related repositories.** The RETrace framework that consumes the traces is usually found locally at `../EX-T-Viz/`.
A separate tool, the **Instruction Set Extender** (`../ISE/`), is taking over automation of
the external toolchain (Verilator, LLVM) and integration as well as acting as an experimental frontend for the VP; 

## Repository Layout

The VP repository is quite large, but most tasks usually only touch a handful of files. 

/vp : the code for the simulator lives in this directory. 
/vp/src/core : the main part of the simulator including fetch decode execute. Common contains shared definitions while rv32 and rv64 implement the 32 and 64 bit RISC-V ISA respectively. 
/vp/src/trace: everything about execution sequence trees. trace.h/trace.cpp hold the node type
that builds them, tracer.h the per core recorder the ISS drives, and export_*.cpp the outputs
including the JITR json. score.h is the interface a --scoring-library plugin compiles against. 
/vp/src/core/common/instr.h: instruction masks and encodings. 
/vp/src/core/common/mmu.h: memory management. 
/vp/src/core/rv32/iss.h and iss.cpp: The heart of the VP. Contains the main code for the instruction set simulator and almost any change will touch this file. 
/vp/src/core/rv32/mem.h: Memory transactions.  

/vp/src/platform : The different configurations for the VP. Each main.cpp models a different target/board, e.g., the hifive board. Tiny32-VP is the most used target. 


/sw: small example programs for testing basic features. For proper tests/evaluation use e.g. ../embench-iot/ or /tacle-bench/out/ 

/img, /env, /demo: ignore these directories. 

/out: often used as the standard output for the JITR trace output. Do not commit these to git. 

## Working Principles

- Follow the repository's documented development policies. 
  Use neighboring code as evidence of established practice, not as
  authority when it conflicts with current policy.
- Prefer the smallest change that fully solves the problem.
- Write code comments, documentation, tests, changelog entries, and public text
  for the final design. Never preserve prompts, review chronology, former names,
  or abandoned approaches unless they remain necessary user-facing context.
- Apply
  [Orwell's six rules for writing](https://www.orwellfoundation.com/the-orwell-foundation/orwell/essays-and-other-works/politics-and-the-english-language/)
  to every category of prose, including reasoning, descriptions, commit
  messages, documentation, docstrings, comments, test text, diagnostics, and
  handoffs:

  1. Do not use a familiar metaphor, simile, or other figure of speech.
  2. Use a short word when it has the same meaning as a long word.
  3. Remove every word that does not add meaning.
  4. Use active voice when possible.
  5. Use everyday English instead of a foreign phrase, scientific word, or
     jargon term when this does not reduce precision.
  6. Break a rule before it makes the text unclear, incorrect, or needlessly
     difficult to read.

- Apply the relevant principles of
  [ASD-STE100 Simplified Technical English](https://www.asd-ste100.org/): use
  short, direct sentences; give each sentence one main idea; use one term for
  one meaning; and use explicit nouns instead of vague pronouns. These are
  mandatory style rules, not a claim of formal ASD-STE100 compliance.
- Base terminology and phrasing on repository usage and established precedents
  in the electronic design automation domain and compiler, high-performance computing,
  and general computer science communities. Use the established term that most
  precisely matches the concept. If communities use different terms, explain the
  mapping once. Never invent synonyms for variety.
- Use the preferred terms in `docs/glossary.md`. Update the glossary in the same
  change when public or potentially ambiguous terminology is introduced or
  changed.
- Add or update automated tests for every behavioral code change. During
  development, run the narrowest relevant test first, then the required lint
  checks before handoff.
- Add tests that protect intended behavior or reproduce a concrete regression.
  Never test provisional implementation choices that are not part of the
  supported contract. 
- Place tests in the corresponding test tree, organized by the subsystem that
  owns the behavior. 
- Do not commit the changes unless asked to. I will review changes and commit them manally. 
- Update CLAUDE.md files with important concepts when necessary, but keep it concise. 

### RETrace conventions
  **non Unicode** do not use characters outside UTF-8 in code or documentation (e.g. em dash, three dots or any invisible characters)

## Building

Use the `essential` target to only build the commonly used targets. Other targets are not critical and can be skipped during development. Bugs that affect these targets will almost always also manifest when compiling the essential targets. 

```bash
make essential
```
## Running

The VP supports a large number of command line options. 
`-e` enables the export of the tracing data. 

```bash
./vp/build/bin/tiny32-vp --intercept-syscalls "$input_file" --output-file "$out_dir/" -e
```

`--performance-mode` trades timing accuracy for speed and halves the run time of a benchmark. It
raises the TLM quantum, so the core runs further ahead of the rest of the simulation before
synchronising: use it when the result does not depend on when the core observes an interrupt or a
peripheral, which is the normal case for a benchmark traced for JITR. Every reference case
produces the same trace with it as without. A switch given after it wins, so
`--performance-mode --tlm-global-quantum 10` keeps the accurate quantum.

Direct memory access is on by default and is worth about 30 percent. `--no-dmi` turns it off,
which is only needed when something has to observe the core's memory traffic on the bus.

`--no-trace` runs the VP as a plain simulator: nothing is recorded and every export is empty.
Recording costs about five times the simulation, so this is 80 percent faster. The switch that
decides it costs 0.2 percent when tracing is on, so a VP built with the tracing in it runs a
benchmark as fast as one built without it.

```bash
./vp/build/bin/tiny32-vp --intercept-syscalls "$input_file" --output-file "$out_dir/" -e --performance-mode
```

## Tests

Run them after changes. Together they take under ten seconds. CI runs them too.

```bash
python3 tests/isa/check.py     # riscv-tests on test32-vp/test64-vp: does the VP execute correctly?
python3 tests/trace/check.py   # trace cases: did the trace change, and are they correct?
```

`tests/isa/suites.json` lists the known failures of the simulator with their reasons.
`tests/trace/check.py` compares each case against its committed reference, checks the trace
invariants (`invariants.py`), and compares the model cases node by node against an independent
reference model (`model.py`). `VP_DEVIATIONS` in `model.py` names where the tracer differs from
the meaning of the instructions. 
Fixing an issue -> delete the entry -> `--update` traces. 
A trace check failure starting with `invariant:` or `model:` means the trace is
wrong, not just changed. The READMEs in `tests/isa` and `tests/trace` explain the output.

`/sw/` contains a small number of example applications.
`../embench-iot/bd/src` contains prebuilt EmBench binaries locally one some systems. 
`python3 tests/trace/invariants.py <out dir> --depth <d>` checks a benchmark trace.

## Roadmap

`ROADMAP.md` lists every planned or proposed feature with a status and a next step. Add an entry
when a feature is agreed, and change its status in the commit that changes the work.

**Testsuite and CI** `tests/isa` and `tests/trace` run in GitHub CI, see Tests above. Not covered yet: the platforms other than tiny32, tiny64, basic and test32/64 are built but never run, and the C cases need a C library the CI image lacks. One fork used TestRIG for equivalence checking, which needs large changes to the VP core. 
**Refactoring** The VP is a complex framework and contains some very large files (e.g. iss.cpp). While smaller modifications are easy to implement with its current design, larger additions require much more work. Refactoring the VP into smaller modules would make future work easier and faster. Even now many modules are never touched and most changes happen in iss.cpp. The Refactor is currently ongoing. 

## ExecPlans

When writing complex features or significant refactors, use an ExecPlan (as
described in [`.claude/PLANS.md`](.claude/PLANS.md)) from design to
implementation. Keep one ExecPlan per independently implemented task and store
it under `docs/plans/<task-slug>.md`; the plan is a living record of that
task's decisions and progress.

## Important Concepts 

### JITR and execution sequence trees

One JSON file per *root instruction* (`md5sumADD.json` holds every k-bounded window/sequence starting
with `ADD`). Each node is one dynamically executed instruction; a root->node path is a
contiguous executed instruction window. The tree structure is similar to tries/prefix trees. 

| Field | Meaning |
| ----- | ------- |
| `instruction` | Mnemonic, e.g. `"ADD"` |
| `type` | Numeric instruction class from the VP |
| `weight` | Occurrences of this window during execution |
| `true_weight` | Non-overlapping occurrence count (the coverage numerator) |
| `register_sets` | `{"<pc>": {count, rd, rs1, rs2, predecessors}}` - **the PC is the key, a decimal string**. `predecessors` is `{"<pc>": count}` for the instruction that ran immediately before, summing to `count` |
| `dependencies_true/anti/output` | *Backward offsets* along the path (`1` = parent) |
| `inputs` / `outputs` | Union of source/destination register numbers |
| `parameters` | `[[pc, [[value, count], ...]], ...]` - the decoded immediate of every instruction carrying one, signed 64-bit, pc-ordered; branches and jumps record their **target PC**. Here PCs are JSON *numbers*. |
| `BranchOutcomes` | `{"<pc>": {offset, taken, not_taken}}`. `Direction`/`offsets` summarise taken executions only and exclude `JALR` |
| `format_version` | `"1.2"`, on the root node only |
| `children` | Nested nodes |

### Snapshots

Snapshots document the result quality at that tool state. This allows measuring the impact of 
changes in different commits. 
Snapshots are **committed**, in `docs/snapshots/`, named `<date>-<first 6 chars of the last commit
id>.json`. The id is the *previous* commit on purpose: the one a snapshot documents does not
exist until it has been taken. 
You can take a snapshot before a change and one after; `--compare` prints the deltas and
**warns loudly when the two runs used different settings**. 