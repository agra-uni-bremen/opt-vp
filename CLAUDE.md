# VP Core Agent Guide

This file contains repository-specific instructions for coding agents working on
the VP. 
This repository contains the SystemC RISC-V Virtual Prototype and its tracing extension (sometimes called RISC-V Opt VP). 

It simulates and traces RISC-V applications with the goal of identifying hotspots and bottlenecks.
The generated traces using the JITR format are then consumed by the RETrace frontend tool. 

It is a fork of https://github.com/agra-uni-bremen/riscv-vp. 

Note: https://github.com/ics-jku/riscv-vp-plusplus is a fork of the original risc-v vp which contains many performance improvements. It also contains numerous changes and additions, not all of which we want to adopt in our VP. It was therefore not merged. 

**Related repositories.** The RETrace framework that consumes the traces is at `../EX-T-Viz/`.
A separate tool, the **Instruction Set Extender** (`../ISE/`), is taking over automation of
the external toolchain (Verilator, LLVM) and integration as well as acting as an experimental frontend for the VP; 

## Repository Layout

The VP repository is quite large, but most tasks usually only touch a handful of files. 

/vp : the code for the simulator lives in this directory. 
/vp/src/core : the main part of the simulator including fetch decode execute. Common contains shared definitions while rv32 and rv64 implement the 32 and 64 bit RISC-V ISA respectively. 
/vp/src/core/common/trace.h, trace.cpp: this is the core for constructing the execution sequence trees and generating the JITR output. 
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

## Tests

There is currently no proper testing system in place. 
`/sw/` contains a small number of example applications. 
`/home/jz/Documents/RISCV/embench-iot/bd/src` contains prebuilt EmBench binaries. 

## Roadmap

`ROADMAP.md` lists every planned or proposed feature with a status and a next step. Add an entry
when a feature is agreed, and change its status in the commit that changes the work.

**Testsuite and CI** There exist simple software examples in /sw and multiple bechmark suites like embench, ml-commons, tacle or RIOT are frequently run on the VP without issues. However in its current state it lacks a proper test setup. One Fork started to use TestRIG for equivalence checking, but this requires large modifications to the VP core. A test setup that can be run using github CI would be good. 
**Refactoring** The VP is a complex framework and contains some very large files (e.g. iss.cpp). While smaller modifications are easy to implement with its current design, larger additions require much more work. Refactoring the VP into smaller modules would make future work easier and faster. Even now many modules are never touched and most changes happen in iss.cpp. 

## ExecPlans

When writing complex features or significant refactors, use an ExecPlan (as
described in [`.claude/PLANS.md`](.claude/PLANS.md)) from design to
implementation. Keep one ExecPlan per independently implemented task and store
it under `docs/plans/<task-slug>.md`; the plan is a living record of that
task's decisions and progress.

## Important Concepts 

### JITR and execution sequence trees

One JSON file per *root instruction* (`md5sumADD.json` holds every k-bounded window/sequence starting
with `ADD`). Each node is one dynamically executed instruction; a root→node path is a
contiguous executed instruction window. The tree structure is similar to tries/prefix trees. 

| Field | Meaning |
| ----- | ------- |
| `instruction` | Mnemonic, e.g. `"ADD"` |
| `type` | Numeric instruction class from the VP |
| `weight` | Occurrences of this window during execution |
| `true_weight` | Non-overlapping occurrence count (the coverage numerator) |
| `register_sets` | `{"<pc>": {count, rd, rs1, rs2, predecessors}}` — **the PC is the key, a decimal string**. `predecessors` is `{"<pc>": count}` for the instruction that ran immediately before, summing to `count` |
| `dependencies_true/anti/output` | *Backward offsets* along the path (`1` = parent) |
| `inputs` / `outputs` | Union of source/destination register numbers |
| `parameters` | `[[pc, [[value, count], ...]], ...]` — the decoded immediate of every instruction carrying one, signed 64-bit, pc-ordered; branches and jumps record their **target PC**. Here PCs are JSON *numbers*. |
| `BranchOutcomes` | `{"<pc>": {offset, taken, not_taken}}`. `Direction`/`offsets` summarise taken executions only and exclude `JALR` |
| `format_version` | `"1.2"`, on the root node only |
| `children` | Nested nodes |

### Snapshots

Snapshots document the result quality at that tool state. This allows measuring the impact of 
changes in different commits. 
Snapshots are **committed**, in `snapshots/`, named `<date>-<first 6 chars of the last commit
id>.json`. The id is the *previous* commit on purpose: the one a snapshot documents does not
exist until it has been taken. 
You can take a snapshot before a change and one after; `--compare` prints the deltas and
**warns loudly when the two runs used different settings**. 
This concept is currently not implemented, but a reference for a different project is availbale here `/../EX-T-Viz/scripts/snapshot.py`