"""Take a measurement snapshot of the VP.

    python3 scripts/snapshot.py

Runs the VP over a fixed benchmark set and writes
`docs/snapshots/<date>-<first 6 chars of the last commit id>.json`. The commit in
the name is the one the snapshot was taken *against*, which is the commit before
the work it documents, because that commit does not exist yet when you take it.

What a snapshot holds and what it does not. It holds how long each workload took,
how much memory it needed, how many instructions and cycles it ran, how many
trees it produced, and a digest of the trace. It does not hold the trace itself,
which is megabytes per workload and changes with every real improvement. The
digest is enough to say whether two runs produced the same trace, and the timings
are enough to plot whether a commit made the VP faster or slower.

Take one before a change that could move any of those numbers and one after:

    python3 scripts/snapshot.py
    [implement changes]
    python3 scripts/snapshot.py
    python3 scripts/snapshot.py --compare docs/snapshots/OLD.json docs/snapshots/NEW.json
    [???]
    [profit/results]

`--compare` prints the change per workload and says when the two runs did
not use the same settings, because a difference in trace depth or in the build
type changes every number and is not a result.

Two snapshots of the same build, taken minutes apart, differ by roughly half a
percent on some machines, and the difference tends to be in one direction
because the machine warms up. So a single comparison cannot resolve a change
under about one percent. When the exact answer matters, bracket it: snapshot before,
make the change, snapshot after, then revert and snapshot again. The two
before-snapshots bound the drift, and anything smaller than that gap is not a
result.

A performance regression that is almost indistinguishable from noise is acceptable if it brings real improvements of e.g. readability or maintainability. 

Commit the snapshot.

Options:

    --benchmarks DIR   where the embench-style workloads live. Each workload is a
                       directory holding an ELF named after it, or named `main`.
    --set NAME         which list from scripts/benchmarks.json to run.
    --depth N          trace depth. Depth drives the run time more than anything
                       else, so two snapshots at different depths are not
                       comparable and --compare says so.
    --repeat N         run each workload N times and keep the fastest. The fastest
                       run is the one least disturbed by the rest of the machine.
    --dry-run          print what would run and exit.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vpbench  # noqa: E402

REPO_ROOT = vpbench.REPO_ROOT
SNAPSHOT_DIR = REPO_ROOT / "docs" / "snapshots"
BENCHMARK_LIST = Path(__file__).resolve().parent / "benchmarks.json"

SCHEMA = "opt-vp/snapshot"
SCHEMA_VERSION = "1.0"

#: Settings that must match before two snapshots may be subtracted. Anything here
#: that differs turns a comparison into a warning instead of a result.
COMPARABLE = ("set", "depth", "vp", "flags", "benchmarks")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="snapshot.py",
        description="Measure the VP over a benchmark set and record the numbers.")
    parser.add_argument("--compare", nargs=2, metavar=("OLD", "NEW"),
                        help="compare two snapshots instead of taking one")
    parser.add_argument("--set", default="default",
                        help="benchmark list from scripts/benchmarks.json (default: default)")
    parser.add_argument("--benchmarks", default=None,
                        help="directory holding the workloads (default: the one in benchmarks.json)")
    parser.add_argument("--vp", default="tiny32-vp", help="VP binary to run (default: tiny32-vp)")
    parser.add_argument("--depth", type=int, default=6,
                        help="trace depth; drives run time more than anything else (default: 6)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="runs per workload, fastest is kept (default: 1)")
    # One string rather than a list: argparse cannot take a dash prefixed value in nargs="*",
    # so --flags "-e --performance-mode" is the only form that works for more than one flag.
    parser.add_argument("--flags", default="-e",
                        help='extra VP flags as one string (default: "-e", the JITR export). '
                             'Quote them: --flags "-e --performance-mode"')
    parser.add_argument("--out", default=None, help="write here instead of docs/snapshots/")
    parser.add_argument("--dry-run", action="store_true", help="print the plan and exit")
    return parser.parse_args(argv)


def load_set(name, override_dir):
    """Return (workload names, the directory they live in) for a named set."""
    data = json.loads(BENCHMARK_LIST.read_text())
    if name not in data["sets"]:
        known = ", ".join(sorted(data["sets"]))
        raise vpbench.VpError(f"no benchmark set named '{name}'. Known sets: {known}")
    entry = data["sets"][name]
    directory = override_dir or entry["directory"]
    return entry["workloads"], directory


def locate(directory, workload):
    """Find the ELF for one workload.

    embench puts it at <dir>/<name>/<name>, the programs under sw/ put it at
    <dir>/<name>/main. Accept both, and say which ones are missing rather than
    failing on the first.
    """
    base = Path(directory)
    if not base.is_absolute():
        base = REPO_ROOT / base
    for candidate in (base / workload / workload, base / workload / "main", base / workload):
        if candidate.is_file():
            return candidate
    return None


def measure(vp, program, out_dir, flags, repeat):
    """Run one workload `repeat` times and keep the fastest run.

    The fastest run is the one least disturbed by whatever else the machine was
    doing. Every run writes to the same directory, so the digest describes the
    last one, and the counters are identical across runs because the VP is
    deterministic.
    """
    runs = []
    for _ in range(max(1, repeat)):
        runs.append(vpbench.run(vp, program, out_dir, flags))
    best = min(runs, key=lambda r: r["wall_s"])
    entry = {
        "wall_s": best["wall_s"],
        "user_s": best["user_s"],
        "peak_rss_kb": best["peak_rss_kb"],
        "exit_code": best["exit_code"],
        "runs": len(runs),
    }
    if len(runs) > 1:
        entry["wall_s_all"] = sorted(r["wall_s"] for r in runs)
    entry.update(best["counters"])
    if "instructions_per_s" in best:
        entry["instructions_per_s"] = best["instructions_per_s"]

    # The register file at the end of the run. Two builds that execute the same program
    # must end with the same registers, so this is what says a change to the simulator
    # altered what it computes rather than only what it costs.
    entry["registers"] = best["registers"]

    files = vpbench.manifest(out_dir)
    entry["output_files"] = len(files)
    entry["output_bytes"] = sum(f["bytes"] for f in files.values())
    entry["output_digest"] = vpbench.manifest_digest(files)
    return entry


def take(args):
    vp = vpbench.find_vp(args.vp)
    workloads, directory = load_set(args.set, args.benchmarks)
    flags = [*args.flags.split(), "--trace-depth", str(args.depth)]

    found, missing = [], []
    for name in workloads:
        path = locate(directory, name)
        (found if path else missing).append((name, path))

    if args.dry_run:
        print(f"vp      {vp}")
        print(f"flags   {' '.join(flags)}")
        print(f"set     {args.set} from {directory}")
        for name, path in found:
            print(f"  run   {name}  {path}")
        for name, _ in missing:
            print(f"  skip  {name}  not found")
        return 0

    if not found:
        raise vpbench.VpError(
            f"none of the {len(workloads)} workloads of set '{args.set}' were found under "
            f"{directory}. Pass --benchmarks with the right directory.")
    if missing:
        vpbench.note(f"[snapshot] {len(missing)} workload(s) not found and skipped: "
                     f"{', '.join(n for n, _ in missing)}")

    scratch = REPO_ROOT / "out" / "snapshot-scratch"
    results, failed = {}, []
    started = time.monotonic()
    for index, (name, path) in enumerate(found, 1):
        vpbench.note(f"[snapshot] {index}/{len(found)} {name}")
        out_dir = scratch / name
        if out_dir.is_dir():
            for stale in out_dir.iterdir():
                if stale.is_file():
                    stale.unlink()
        entry = measure(vp, path, out_dir, flags, args.repeat)
        if entry["exit_code"] != 0:
            failed.append(name)
        results[name] = entry

    snapshot = {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "run": {
            "set": args.set,
            "benchmarks": str(directory),
            "vp": args.vp,
            "depth": args.depth,
            "flags": args.flags.split(),
            "repeat": args.repeat,
            "workloads": [n for n, _ in found],
            "skipped": [n for n, _ in missing],
        },
        "environment": vpbench.environment(),
        "timing": {"wall_clock_s": round(time.monotonic() - started, 3)},
        "totals": totals(results),
        "workloads": results,
    }

    out_path = Path(args.out) if args.out else default_path(snapshot)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(snapshot, indent=2, sort_keys=False) + "\n")

    report(snapshot)
    print(f"\n[snapshot] wrote {short_path(out_path)}")
    revision = snapshot["environment"]["revision"]
    print(f"[snapshot] Commit it. The name says {revision['short'] or 'no commit'}, which is the "
          f"commit this snapshot was taken against.")
    if revision["dirty"]:
        print("[snapshot] The tree was dirty, so the name does not fully describe what ran.")
    if failed:
        vpbench.note(f"[snapshot] these workloads exited non-zero: {', '.join(failed)}")
        return 1
    return 0


def totals(results):
    """Sums and rates over every workload, which is what a trend plot uses."""
    wall = sum(r["wall_s"] for r in results.values())
    instructions = sum(r.get("instructions", 0) for r in results.values())
    return {
        "workloads": len(results),
        "wall_s": round(wall, 3),
        "instructions": instructions,
        "instructions_per_s": int(instructions / wall) if wall > 0 else 0,
        "peak_rss_kb": max((r["peak_rss_kb"] for r in results.values()), default=0),
        "output_bytes": sum(r["output_bytes"] for r in results.values()),
    }


def short_path(path):
    """Repository relative when it is inside the repository, absolute otherwise.

    `--out` may point anywhere, so this must not assume the path is under the tree.
    """
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def default_path(snapshot):
    date = time.strftime("%Y-%m-%d")
    short = snapshot["environment"]["revision"]["short"] or "nocommit"
    return SNAPSHOT_DIR / f"{date}-{short}.json"


def report(snapshot):
    print()
    print(f"{'workload':<22}{'wall s':>9}{'instr/s':>12}{'peak MB':>10}{'out MB':>9}")
    for name, entry in snapshot["workloads"].items():
        print(f"{name:<22}{entry['wall_s']:>9.2f}"
              f"{entry.get('instructions_per_s', 0):>12,}"
              f"{entry['peak_rss_kb'] / 1024:>10.1f}"
              f"{entry['output_bytes'] / 1e6:>9.1f}")
    total = snapshot["totals"]
    print(f"{'total':<22}{total['wall_s']:>9.2f}"
          f"{total['instructions_per_s']:>12,}"
          f"{total['peak_rss_kb'] / 1024:>10.1f}"
          f"{total['output_bytes'] / 1e6:>9.1f}")


# --------------------------------------------------------------------------- #
# comparing
# --------------------------------------------------------------------------- #

def compare(old_path, new_path):
    old = json.loads(Path(old_path).read_text())
    new = json.loads(Path(new_path).read_text())

    warnings = []
    for key in COMPARABLE:
        if old["run"].get(key) != new["run"].get(key):
            warnings.append(f"{key}: {old['run'].get(key)!r} -> {new['run'].get(key)!r}")
    old_build = old["environment"].get("build", {})
    new_build = new["environment"].get("build", {})
    for key in sorted(set(old_build) | set(new_build)):
        if old_build.get(key) != new_build.get(key):
            warnings.append(f"build {key}: {old_build.get(key)!r} -> {new_build.get(key)!r}")
    if old["environment"].get("platform") != new["environment"].get("platform"):
        warnings.append("the two runs were taken on different machines")

    if warnings:
        print("!! These two snapshots did not measure the same thing. Every number below is")
        print("!! the sum of your change and the difference in settings, so it is not a result.")
        for line in warnings:
            print(f"!!   {line}")
        print()

    print(f"old  {Path(old_path).name}  {old['environment']['revision']['short']}")
    print(f"new  {Path(new_path).name}  {new['environment']['revision']['short']}")
    print()
    print(f"{'workload':<22}{'old s':>9}{'new s':>9}{'change':>10}  trace")

    changed_traces, changed_registers, deltas = [], [], []
    names = [n for n in new["workloads"] if n in old["workloads"]]
    for name in names:
        a, b = old["workloads"][name], new["workloads"][name]
        change = percent(a["wall_s"], b["wall_s"])
        deltas.append(change)
        same = a.get("output_digest") == b.get("output_digest")
        if not same:
            changed_traces.append(name)
        if a.get("registers") != b.get("registers"):
            changed_registers.append(name)
        print(f"{name:<22}{a['wall_s']:>9.2f}{b['wall_s']:>9.2f}{change:>9.1f}%  "
              f"{'same' if same else 'CHANGED'}")

    ta, tb = old["totals"], new["totals"]
    print(f"{'total':<22}{ta['wall_s']:>9.2f}{tb['wall_s']:>9.2f}"
          f"{percent(ta['wall_s'], tb['wall_s']):>9.1f}%")
    print(f"{'peak MB':<22}{ta['peak_rss_kb'] / 1024:>9.1f}{tb['peak_rss_kb'] / 1024:>9.1f}"
          f"{percent(ta['peak_rss_kb'], tb['peak_rss_kb']):>9.1f}%")

    only_old = sorted(set(old["workloads"]) - set(new["workloads"]))
    only_new = sorted(set(new["workloads"]) - set(old["workloads"]))
    if only_old:
        print(f"\nonly in the old snapshot: {', '.join(only_old)}")
    if only_new:
        print(f"only in the new snapshot: {', '.join(only_new)}")

    if deltas:
        print(f"\nmedian change {statistics.median(deltas):+.1f}%")
    if changed_registers:
        print(f"\nThe registers at the end of the run changed for: {', '.join(changed_registers)}")
        print("The VP now computes something different. Find out why before reading anything else.")
    if changed_traces:
        print(f"\nThe trace changed for: {', '.join(changed_traces)}")
        print("If that was not the point of the change, find out why before trusting the timings.")
    if not changed_traces and not changed_registers:
        print("\nEvery trace digest and register file matched, so only the cost changed.")
    return 0


def percent(before, after):
    if not before:
        return 0.0
    return (after - before) / before * 100.0


def main(argv):
    args = parse_args(argv)
    try:
        if args.compare:
            return compare(*args.compare)
        return take(args)
    except vpbench.VpError as error:
        vpbench.note(f"[snapshot] {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
