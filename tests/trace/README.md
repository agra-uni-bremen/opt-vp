# Trace reference check

Checks that the VP still produces the trace it produced before. Run it after every
change:

    cd vp/build && ctest -R trace

or directly:

    python3 tests/trace/check.py

It takes about two seconds. If it passes, the JITR output of every case is identical
to what `reference/` records, down to the register file at the end of the run.

## Failures 

The message names the case, the output file, and what changed. It keeps the output it
produced under `out/trace-check/<case>/`.

Three kinds of failure, in decreasing order of criticality:

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
record it:

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
