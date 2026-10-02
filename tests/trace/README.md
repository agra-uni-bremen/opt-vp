# Trace reference check

Checks that the VP still produces the trace it produced before. 
Run it after changes that affect tracing:

    cd vp/build && ctest -R trace

or directly:

    python3 tests/trace/check.py

It takes about two seconds. If it passes, the JITR output of every case is identical
to what `reference/` records, down to the register file at the end of the run, and the
trace is consistent with the two checks below.

## Is the trace right?

The reference says whether a trace changed, not whether it is right. Two more checks
say that, and both run on every case:

* `invariants.py` checks the properties every JITR trace must have, whatever program
  produced it: a node weighs at least as much as its children together, the counts per pc
  add up to the weight, the predecessors of a pc add up to its count, every dependency
  points inside its window, and the root weights add up to the number of executed
  instructions. It also works on a trace nobody recorded a reference for:

      python3 tests/trace/invariants.py out/md5sum/ --depth 6

* `model.py` is a second implementation of the trace. It runs the program with
  `--trace-mode`, which prints every executed instruction, and derives from that list
  what every node of every tree must hold: weight, true_weight, the pcs and their
  predecessors, the registers, the register dependencies and the branch outcomes. Then it
  compares node by node. A case with `"model": true` in `cases.json` gets this check. The
  model describes RV32IM programs that do not trap, which is what the `trace-test-*`
  programs are. It can also run on its own:

      python3 tests/trace/model.py sw/trace-test-rv32im --depth 6

### Known deviations

Where the tracer currently records something other than what the instructions mean, the
model names the difference in `VP_DEVIATIONS` and reproduces it, so the check passes and
still pins everything else. `--strict` turns them off and shows where each one applies:

    python3 tests/trace/model.py sw/trace-test-default --depth 6 --strict

When a deviation is fixed in the tracer, delete its entry in `model.py`, re-record the
references with `--update`, and say so in the changelog.

## Switches that must not change the result

A case with `"same_as": "<case>"` has no reference of its own: it must produce exactly the
files, counters and registers of the named case. `performance-mode` and `no-dmi` use it to
pin that `--performance-mode` and `--no-dmi` change the speed and not the result.

## Failures 

The message names the case, the output file, and what changed. It keeps the output it
produced under `out/trace-check/<case>/`.

A line starting with `invariant:` or `model:` means the trace is wrong. Find the cause. 
`--update` does not record a trace that fails either check.

Three kinds of reference failure, in decreasing order of criticality:

* `hart 0 x15: 0x2a -> 0x99` means the VP computed something different. The simulator
  changed, not just the recorder.
* `mainADD.json: the trace changed` means the recorded trace changed, but the simulation behavior is identical.
* `mainADD.json: same trace, different spelling` means only the field order or the
  formatting changed. The trace is the same. The simulator behavior is the same. 

If the change was meant to alter the trace, accept it:

    python3 tests/trace/check.py --update

Do not run `--update` to fix a mismatch without knowing which of the three
kinds of failure it was.

## What the reference files hold

One JSON file per case: the flags that produced it, the counters the VP reported, the
register file at the end of the run, and one entry per output file with its size and
two digests. 

The *semantic* digest is over the JSON parsed and re-serialised with sorted keys, so
it ignores field order and whitespace. The *raw* digest is over the bytes as written.

## Adding a case

Add it to `cases.json` with a comment saying what it pins, build its program, then
record it. Give it `"model": true` when its program is RV32IM without traps:

    make -C sw/<program>
    python3 tests/trace/check.py --update --case <name>

Two keys change how a case is treated. `"ci": false` leaves it out of `--ci`, for a
program that needs a C library the CI image does not have. `"writes_files": false` says
the case writes no output, so the counters and the register file are the whole check;
the `no-trace` case uses it to pin that `--no-trace` does not change what a program does.

## Related

`scripts/snapshot.py` measures cost rather than correctness: it runs the VP over a
benchmark set and records timings, memory, counters, registers and a trace digest into
`docs/snapshots/`, so two commits can be subtracted. Both tools share
`scripts/vpbench.py`.
