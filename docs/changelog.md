# Changelog

What changed, newest first, one entry per work item. Each entry says what it *is*, not how
it was built - the ExecPlans carry the reasoning and the code carries the detail, and git
history has the rest. You should not normally need to read this file. 
Document important changes that are directly relevant for future implementation work. 
There is no need to document old behavior that is no longer relevant unless its a bugfix 
that might affect other forks. 

## 2026-10-02 - VP Refactor

**Added: the ISA test suites.** `tests/isa/check.py` builds the riscv-tests suites (a pinned
submodule at `tests/isa/riscv-tests`) and runs them on `test32-vp` and the new `test64-vp`: I,
M, A, C, F, D and the machine and supervisor tests, 238 in about five seconds. 17 fail, each
listed in `tests/isa/suites.json` with its reason; the most serious is that rv64 SRLI drops the
upper 32 bits. `test32-vp` now prints `to-host: 1` on a pass and exits 1 on a failure, where it
used to exit 0 either way. `test64-vp` is the same source built against the rv64 core. Both are
in `make essential`, and ctest runs the suite as `isa`.

**Fixed: a trapping instruction made the tracer count one window twice.** `exec_step` opens a
step, and every `raise_trap` throws out of the middle of it, so the ring buffer slot was never
written and the index never moved on. The next instruction then inserted the window that had
just been inserted. A ten iteration loop that traps once per iteration reported one tree root at
weight 19 for ten executions, with `true_weight` at the correct 10. The step is opened before the
fetch now, so a fetch fault is covered too, and the trap path marks the slot unwritten. The
trapping instruction stays out of the trace, as it always has.

**Fixed: the MMU indexed its TLB with a privilege level it has no row for.** `mode` comes from
`prv` or, under `mstatus.mprv`, from the two bit `mstatus.mpp`, so it can hold the reserved value
1. The TLB has one row per supported mode, two, and machine mode was the only value sent down the
untranslated path, so mode 2 read and wrote 12 KB past the array. On `tiny32` the MMU is a local
of `main`, so that is the stack. One comparison now covers machine mode and the reserved value
together. `SATP_MODE_SV64` is gone from the mode decoder for the same reason: it produced a shift
of 65 on a 64 bit value, and six levels of nine index bits cannot describe a 64 bit address space.

**Fixed: `RealCLINT` ran 1.7 percent fast.** `#define DIVIDEND (uint64_t(15625)/uint64_t(512))`
is integer division, so it was 30 and the timer modelled 33.3 kHz rather than the 32.768 kHz its
comment promises. Numerator and denominator are kept apart now. The conversion is also capped:
software arms `mtimecmp` with all ones before writing the real value, as the privileged spec's own
example does, and the multiplication wrapped, which fired the timer almost at once.

**Fixed in the ELF loader:** a segment ending on the last byte of the target memory was rejected
as not fitting; `get_heap_addr` returned `s + s % 8`, which is 8 byte aligned only when `s` is
already 0 or 4 modulo 8; and the "Offset overlaps into section" branch could not be reached,
because the loop above it skips every segment that starts below the memory.

**`SimpleBus` reports a port an earlier port hides, and a port with no address range.**
`decode` answers with the first port whose range contains the address, so a port overlapped by
an earlier one is unreachable and nothing fails. On `tiny32` the memory port covered the CLINT
and the syscall region for years, so a program waiting on mtime read memory, saw zero, and
waited forever. The bus now says so at startup, which is also how a workload that raises
`--memory-size` past the CLINT learns what it gave up. `ports` is zero initialised, so an
unassigned port is reported rather than dereferenced.

**Removed a string round trip from every intercepted syscall.** The rv32 handler wrote the
result back with `boost::lexical_cast<int32_t>`, which formats an `int` into a string and parses
it again to produce the same `int`. The rv64 handler always assigned it directly. That also
takes `boost/lexical_cast.hpp` out of the header.

**Fixed: instruction fetch by direct memory access skipped address translation.** `load_instr`
on `InstrMemoryProxy` read the memory behind the bus with the virtual address, while the bus path
translates first. That was harmless while the fast path was opt in and nothing that pages used
it. Making it the default would have broken `linux32-vp` and `linux-vp` silently as soon as a
program turned paging on. The proxy now hands the fetch to the translating interface whenever satp is not in BARE mode. 

Also fixed there: the rv64 proxy read eight bytes and returned four, which reads past the end of
the memory range at its last word.

**Added: `--performance-mode`, which halves the run time.** One switch for every setting that
trades timing accuracy for speed. Currently it sets direct memory access and raises the TLM
quantum to 100000 ns. The core then runs that far
ahead of the rest of the simulation before synchronising, so use it for a workload whose result
does not depend on when it observes an interrupt or a peripheral. 

**Direct memory access is the default (about 30% faster).** The core
reads instructions and data straight from the plain memory ranges instead of sending a TLM
transaction over the bus for each one. `--use-dmi` has always done this (but was rarely used).

**One instruction node class instead of abstract ones.** `vp/src/trace/trace.h` had an abstract
`InstructionNode`, two classes deriving from it virtually, two payload base classes and four
concrete combinations. There is now one concrete `InstructionNode` with no virtual functions.
`node_type` is now the type. 

**The tree for an opcode is found by index.** `Tracer::tree_for` searched a list of trees on
every executed instruction, and a real program has around a hundred trees. It indexes them by
opcode instead.

**A node's children are one contiguous table.** Each entry is the child's opcode next to its
pointer, so finding the next node of a window reads one array instead of loading every child
node to look at its opcode.

**The dependency analysis no longer grows with the square of the trace depth.** For each
instruction of a window, `insert_rb` walked every earlier position of the window.
Reversing the bit order of the position mask produces all offsets at once. md5sum at
trace depth 20 is 17.6 percent faster, at compiled depth 32 it is 29 percent faster. 
Beyond 64 the masks no longer fit in a word and the walk is still
used.

**Fixed: the multicore platforms overwrote their own trace.** Exported filenames carried no
hart id, so the second core replaced the first core's files. Harts above zero now get a
`-hart<n>` suffix; hart 0 filenames are unchanged.

**Trace reference check.** `tests/trace/` checks that the VP still produces the trace it
produced before, over nine cases, in under two seconds. `cd vp/build && ctest -R trace`, or
`python3 tests/trace/check.py`. 
A change that is meant to change the trace is accepted with `--update`, and belongs in this file.
See `tests/trace/README.md`.

**Measurement snapshots.** `python3 scripts/snapshot.py` runs the VP over a benchmark set and
writes `docs/snapshots/<date>-<commit>.json` with the wall time, peak memory, counters,
end-of-run registers and a trace digest per workload. 
Two snapshots of the same build differ
by roughly half a percent, usually in one direction, so a single comparison cannot resolve a
change under about one percent. 
