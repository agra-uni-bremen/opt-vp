"""Check that the VP still produces the trace it produced before.

    cd vp/build && ctest -R trace          # for CI
    python3 tests/trace/check.py        # the same, directly
    python3 tests/trace/check.py --update   # accept the current output as the new reference

Run the VP over the programs in
`cases.json`, reduce each output directory to one digest per file, and compare
that against the committed reference in `reference/`. 

When a digest mismatches, the produced output is kept and the message says where,
so you can look at the actual JSON. To see the difference against the previous
commit, check that commit out, run with `--update`, and read the diff of the
reference file (or keep the two output directories and diff them).
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "scripts"))
import vpbench  # noqa: E402

REFERENCE_DIR = HERE / "reference"
CASES = HERE / "cases.json"
OUT_DIR = vpbench.REPO_ROOT / "out" / "trace-check"


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="check.py",
        description="Compare the VP's trace output against the committed reference.")
    parser.add_argument("--update", action="store_true",
                        help="write the current output as the new reference instead of comparing")
    parser.add_argument("--case", action="append", default=None,
                        help="run only this case; may be given more than once")
    parser.add_argument("--keep", action="store_true",
                        help="keep the produced output even when a case passes")
    parser.add_argument("--ci", action="store_true",
                        help='run only cases without "ci": false, which are the ones whose '
                             'program builds with a bare metal cross compiler and no C library')
    parser.add_argument("--list", action="store_true", help="list the cases and exit")
    return parser.parse_args(argv)


def run_case(case):
    """Run one case and return its manifest, or raise with what went wrong."""
    vp = vpbench.find_vp(case.get("vp", "tiny32-vp"))
    program = vpbench.resolve_program(case["program"])
    out_dir = OUT_DIR / case["name"]
    if out_dir.is_dir():
        shutil.rmtree(out_dir)

    result = vpbench.run(vp, program, out_dir, case.get("flags", []))
    if result.get("timed_out"):
        raise vpbench.VpError(
            f"{case['name']}: the VP did not terminate and was killed\n"
            f"  {' '.join(result['command'])}")
    if result["exit_code"] != 0:
        raise vpbench.VpError(
            f"{case['name']}: the VP exited {result['exit_code']}\n"
            f"  {' '.join(result['command'])}\n"
            + "\n".join("  " + line for line in result["stdout"].splitlines()[-12:]))

    entries = vpbench.manifest(out_dir)
    #a case with "writes_files": false asks the VP not to export anything, which is what
    #--no-trace does. There the counters and the register file are the whole check.
    if not entries and case.get("writes_files", True):
        raise vpbench.VpError(
            f"{case['name']}: the VP wrote nothing to {out_dir}. Did the flags change?")
    if entries and not case.get("writes_files", True):
        raise vpbench.VpError(
            f"{case['name']}: the case says it writes no file, but the VP wrote "
            f"{len(entries)} of them to {out_dir}")
    return entries, result, out_dir


def reference_path(name):
    return REFERENCE_DIR / f"{name}.json"


def write_reference(case, entries, result):
    """Record a manifest plus the counters, so a reference says what produced it."""
    payload = {
        "case": case["name"],
        "vp": case.get("vp", "tiny32-vp"),
        "program": case["program"],
        "flags": case.get("flags", []),
        "counters": result["counters"],
        "registers": result["registers"],
        "digest": vpbench.manifest_digest(entries),
        "files": entries,
    }
    path = reference_path(case["name"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1, sort_keys=True) + "\n")
    return path


def register_problems(want, got):
    """Name every register that ends the run holding a different value.

    A difference here means the VP executed the program differently, which is a much
    more serious finding than a changed trace, so it is reported per register with both
    values in hex rather than as one line saying the dump moved.
    """
    problems = []
    if len(want) != len(got):
        return [f"the run reported {len(got)} core(s), the reference has {len(want)}"]
    for reference_core, current_core in zip(want, got):
        hart = reference_core.get("hart")
        for name in sorted(set(reference_core) | set(current_core)):
            if name == "hart":
                continue
            a, b = reference_core.get(name), current_core.get(name)
            if a != b:
                problems.append(f"hart {hart} {name}: "
                                f"{'absent' if a is None else f'0x{a:x}'} -> "
                                f"{'absent' if b is None else f'0x{b:x}'}")
    return problems


def compare(case, entries, result):
    """Return a list of human readable differences, empty when the case passes."""
    path = reference_path(case["name"])
    if not path.is_file():
        return [f"no reference at {path.relative_to(vpbench.REPO_ROOT)}. "
                f"Create it with: python3 tests/trace/check.py --update"]

    reference = json.loads(path.read_text())
    problems = []

    for key in ("vp", "program", "flags"):
        want = reference.get(key)
        got = case.get(key, "tiny32-vp" if key == "vp" else case.get(key))
        if key == "flags":
            got = case.get("flags", [])
        elif key == "program":
            got = case["program"]
        if want != got:
            problems.append(f"the case changed since the reference was taken: {key} "
                            f"{want!r} -> {got!r}. Re-record it with --update.")

    for name, want in reference["counters"].items():
        got = result["counters"].get(name)
        if got != want:
            problems.append(f"counter {name}: {want} -> {got}")

    problems.extend(register_problems(reference.get("registers", []), result["registers"]))

    want_files = set(reference["files"])
    got_files = set(entries)
    for missing in sorted(want_files - got_files):
        problems.append(f"missing output file: {missing}")
    for added in sorted(got_files - want_files):
        problems.append(f"unexpected output file: {added}")

    for name in sorted(want_files & got_files):
        a, b = reference["files"][name], entries[name]
        if a.get("semantic_sha256") != b.get("semantic_sha256"):
            problems.append(f"{name}: the trace changed "
                            f"({a['bytes']} -> {b['bytes']} bytes)")
        elif a.get("raw_sha256") != b.get("raw_sha256"):
            problems.append(f"{name}: same trace, different spelling "
                            f"(field order or formatting moved)")
    return problems


def main(argv):
    args = parse_args(argv)
    try:
        cases = vpbench.load_cases(CASES)["cases"]
    except vpbench.VpError as error:
        vpbench.note(f"[trace-check] {error}")
        return 1

    if args.ci:
        skipped = [c["name"] for c in cases if not c.get("ci", True)]
        cases = [c for c in cases if c.get("ci", True)]
        if skipped:
            vpbench.note(f"[trace-check] --ci leaves out: {', '.join(skipped)}")

    if args.case:
        wanted = set(args.case)
        unknown = wanted - {c["name"] for c in cases}
        if unknown:
            vpbench.note(f"[trace-check] no such case: {', '.join(sorted(unknown))}")
            return 1
        cases = [c for c in cases if c["name"] in wanted]

    if args.list:
        for case in cases:
            print(f"{case['name']:<24}{case.get('vp', 'tiny32-vp'):<14}{case['program']}")
        return 0

    failures = []
    for case in cases:
        name = case["name"]
        try:
            entries, result, out_dir = run_case(case)
        except vpbench.VpError as error:
            print(f"FAIL  {name}")
            vpbench.note(f"      {error}")
            failures.append(name)
            continue

        if args.update:
            path = write_reference(case, entries, result)
            print(f"WROTE {name:<24}{len(entries)} files -> "
                  f"{path.relative_to(vpbench.REPO_ROOT)}")
        else:
            problems = compare(case, entries, result)
            if problems:
                print(f"FAIL  {name}")
                for problem in problems[:20]:
                    print(f"      {problem}")
                if len(problems) > 20:
                    print(f"      ... and {len(problems) - 20} more")
                print(f"      output kept at {out_dir.relative_to(vpbench.REPO_ROOT)}")
                failures.append(name)
            else:
                print(f"ok    {name:<24}{len(entries)} files, "
                      f"{result['counters'].get('instructions', '?')} instructions")

        if not args.keep and name not in failures and not args.update:
            shutil.rmtree(out_dir, ignore_errors=True)

    if failures:
        print(f"\n{len(failures)} of {len(cases)} cases failed: {', '.join(failures)}")
        print("If the change was meant to move the trace, record it and say so in the")
        print("changelog: python3 tests/trace/check.py --update")
        return 1

    print(f"\nall {len(cases)} cases match the reference")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
