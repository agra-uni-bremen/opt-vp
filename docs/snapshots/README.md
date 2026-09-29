# Measurement snapshots

One file per measurement of what the VP costs: wall time, peak memory, the counters it
reported, the register file at the end of each run, and a digest of the trace it produced.
They are committed so two points in the repository's history can be subtracted.

Take one with `python3 scripts/snapshot.py` and compare two with
`python3 scripts/snapshot.py --compare OLD NEW`, which warns when the two runs used
different settings.

## Naming

`<date>-<first 6 characters of the last commit>.json`. The commit is the *previous* one on
purpose: the commit a snapshot documents does not exist until the snapshot has been taken.

A second measurement against the same commit takes a suffix saying what it is, for example
`2026-09-29-e78862-before-m5.json`. The pair in this directory brackets Milestone 5 of
[refactor-vp](../plans/refactor-vp.md): the same commit, measured before and after the
change, because this machine drifts upward by about half a percent during a session and a
single comparison cannot resolve anything smaller.

Every file records the commit, the branch, whether the tree was dirty, the build options
that change what the VP does, and the machine, so a comparison can say when two numbers are
not comparable instead of reporting the difference as a result.

## What the recorded flags do not cover

A snapshot records the flags it passed, not the VP's defaults. Two defaults changed on
2026-09-29 and a comparison across that date will attribute them to the commit:

* direct memory access is now on, worth about 30 percent, and
* `occurrence` no longer takes 32 bytes per node.

Comparisons within either side of that date are sound.

`2026-09-29-fc8560-performance-mode.json` is the same commit measured with
`--performance-mode`, which trades timing accuracy for speed. Compare it against
`2026-09-29-28d43f.json` to see what the mode costs in accuracy terms and buys in time; do not
treat it as a point in the default series.
