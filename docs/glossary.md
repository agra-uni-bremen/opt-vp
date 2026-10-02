(glossary)=

# Glossary

This glossary records preferred terms. 

Update this page at the same time when introducing or changing a public
or potentially ambiguous term. Each new entry must name the preferred term, list
accepted aliases, and be understandable without detailed knowledge of the
implementation.

## Terms and abbreviations

```{glossary}
:sorted:

coverage
  **Preferred term:** coverage. **Accepted aliases:** none. The share of the
  executed instructions of a workload that a candidate replaces. It is a share
  of *instructions*, not of run time: every trace and simulator can produce an
  instruction count, where a share of run time needs a timing model. 

cycle coverage
  **Preferred term:** cycle coverage. **Accepted aliases:** none. The same share
  as {term}`coverage`, counted in cycles instead of instructions: what the
  {term}`variant` costs in software, times its {term}`true weight`, over the
  whole workload priced by the same cost model. 

reference model
  **Preferred term:** reference model. **Accepted aliases:** trace model. A
  second implementation of the JITR trace, `tests/trace/model.py`. It derives
  every node from the list of executed instructions that `--trace-mode` prints,
  and the trace check compares it node by node with what the tracer wrote.

reference trace
  **Preferred term:** reference trace. **Accepted aliases:** reference. The
  committed record in `tests/trace/reference/` of what one case produced: its
  flags, counters, final register file and a digest per output file. It says
  whether a trace changed, not whether it is right.

riscv_standard
  **Preferred term:** `riscv_standard`. **Accepted aliases:** standard contract.
  The {term}`design contract` of one 32-bit instruction in an unmodified
  pipeline: two source registers, one result, one immediate, no memory and no
  control flow.

riscv_relaxed
  **Preferred term:** `riscv_relaxed`. **Accepted aliases:** relaxed contract.
  Still one instruction word, but the core gives ground: a third source
  register, a register-pair result, operands pinned to fixed registers, memory
  through the core's own load/store unit, and up to two {term}`state register`s.
  Each of those is a change to the core, not just to the decoder.

effective coverage
  **Preferred term:** effective coverage. **Accepted aliases:** none. The share
  of the workload a proposal can actually replace, which is its {term}`coverage`
  minus the executions whose dataflow the proposed hardware does not implement.

execution sequence tree
  **Preferred term:** execution sequence tree.
  The compressed record of what a program executed, one tree per starting
  instruction. Each node is one dynamically executed instruction and each path
  from the root is a contiguous {term}`window` the program ran, with the number
  of times it ran.

expected speedup
  **Preferred term:** expected speedup. **Accepted aliases:** none. What the
  whole workload gains, by Amdahl's law, from replacing a candidate's
  {term}`coverage` at its {term}`sequence speedup`.

extension
ISA extension
  **Preferred term:** extension. **Accepted expansion:** ISA extension. A set of
  custom instructions designed and integrated together, sharing one custom
  opcode. Their encodings are allocated as a set, their area is not the sum of
  the parts, and their speedups do not multiply.

IR
intermediate representation
  **Preferred term:** intermediate representation. **Accepted abbreviation:**
  IR. A program representation used between a source language and final output
  so compiler analyses and transformations can operate on explicit structure.

ISA test suite
  **Preferred term:** ISA test suite. **Accepted aliases:** riscv-tests, the name
  of the upstream project. The self checking assembly programs in
  `tests/isa/riscv-tests`, one directory per extension and privilege level, for
  example `rv32ui` (user level base integer). Each program reports pass or the
  number of the failing test case through the `tohost` symbol.

known deviation
  **Preferred term:** known deviation. **Accepted aliases:** none. A named
  difference between what the VP's tracer records and what the
  {term}`reference model` derives from the architectural meaning of the
  instructions. The model reproduces it, so the trace check passes, and
  `--strict` shows where it applies. Listed in `VP_DEVIATIONS` in
  `tests/trace/model.py`.

known failure
  **Preferred term:** known failure. **Accepted aliases:** expected failure. A
  test of the {term}`ISA test suite` that is built and run and is expected to
  fail, listed with the reason in `tests/isa/suites.json`. A known failure that
  starts to pass fails the check, so the list stays current.

LLVM
  **Preferred term:** LLVM. **Accepted aliases:** none. The compiler
  [infrastructure project](https://llvm.org/). LLVM is
  the current project name; do not expand it as an abbreviation.

sequence speedup
  **Preferred term:** sequence speedup. **Accepted aliases:** none. How much
  faster the custom instruction is than the instructions it replaces, as a ratio
  of cycles. It describes the sequence and not the program: pair it with a
  {term}`coverage` to get an {term}`expected speedup`.

trace invariant
  **Preferred term:** trace invariant. **Accepted aliases:** none. A property
  every JITR trace must have, whatever program produced it. Checked by
  `tests/trace/invariants.py`.

snapshot
  **Preferred term:** snapshot. **Accepted aliases:** none. A committed record
  of what the VP produced for a fixed set of traces and settings, so a change
  to the tool can be compared against what it produced before. Named for the
  commit *before* the work it documents.

VP
Virtual Prototype
  **Preferred term:** VP. **Accepted expansion:** Virtual Prototype. 
  SystemC based Simulator.

sequence
  **Preferred term:** sequence. **Accepted aliases:** window, n-gram (the term related work uses for the
  same construct). A contiguous run of
  executed instructions, which is what a path in an
  {term}`execution sequence tree` is and what a custom instruction replaces. A
  value the window reads and does not produce is an operand; a value it produces
  and something else reads is a result.
```
