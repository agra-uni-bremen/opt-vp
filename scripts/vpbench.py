"""Run the VP and measure what it produced.

Shared by `scripts/snapshot.py`, which records performance over time, and by
`vp/tests/trace/check.py`, which checks that the trace did not change. Both need
the same three things, so they live here once:

* find a VP binary and a workload,
* run it and collect wall time, peak memory and the counters the VP prints,
* reduce the output directory to digests, so a change is detectable without
  committing megabytes of JSON.

Digests come in two kinds per file. The
*semantic* digest is taken over the JSON parsed and re-serialised with sorted
keys, so it ignores field order and whitespace. The *raw* digest is taken over
the bytes as written. A semantic mismatch means the trace changed. A raw
mismatch with the semantic digest intact means only the spelling changed, which
during a refactor is usually what you meant.

Nothing here imports anything outside the standard library.
"""

import hashlib
import json
import os
import platform
import re
import resource
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

#: Where `make essential` puts the binaries. Set VP_BIN_DIR to use another build tree,
#: which is how ctest points these tools at the build directory it was configured in.
BIN_DIR = Path(os.environ.get("VP_BIN_DIR") or REPO_ROOT / "vp" / "build" / "bin")

#: Files the VP writes that are not part of the trace. Kept out of the digests so
#: a change to one of them does not look like a change to the trace.
NON_TRACE_SUFFIXES = (".log",)


class VpError(RuntimeError):
    """A run could not be set up or did not finish. The message says what to do."""


# --------------------------------------------------------------------------- #
# locating things
# --------------------------------------------------------------------------- #

def find_vp(name="tiny32-vp"):
    """Return the path to a VP binary, or explain how to build it."""
    path = BIN_DIR / name
    if not path.is_file():
        raise VpError(
            f"{path} is not there. Build it first:\n"
            f"    make essential")
    return path


def resolve_program(spec):
    """Turn a workload spec into a path to an ELF file.

    A spec is either a path, absolute or relative to the repository root, or the
    name of a directory under `sw/`, in which case the ELF inside it is `main`.
    """
    candidate = Path(spec)
    if not candidate.is_absolute():
        candidate = REPO_ROOT / candidate
    if candidate.is_file():
        return candidate
    if candidate.is_dir() and (candidate / "main").is_file():
        return candidate / "main"
    raise VpError(
        f"no workload at {candidate}. For the programs under sw/, build them first:\n"
        f"    make -C {candidate if candidate.is_dir() else candidate.parent}")


# --------------------------------------------------------------------------- #
# running
# --------------------------------------------------------------------------- #

#: Lines of the VP's own report that carry a number worth keeping.
_COUNTERS = (
    ("instructions", re.compile(r"^total instructions:\s*(\d+)", re.M)),
    ("cycles", re.compile(r"^total cycles:\s*(\d+)", re.M)),
    ("trees", re.compile(r"^execution statistics:\s*\((\d+) Trees\)", re.M)),
    ("unused_instructions", re.compile(r"^\[Unused Instructions\]\s*\n\s*(\d+)", re.M)),
    ("retired", re.compile(r"^num-instr\s*=\s*(\d+)", re.M)),
)


#: The VP colours some of its report. Strip the escapes before matching anything.
_ANSI = re.compile(r"\x1b\[[0-9;]*m")

#: One line of the register dump, e.g. "s0/fp(x8) =        0". The value is hex without a prefix,
#: printed with "%8x", so it is the unsigned 32 or 64 bit pattern rather than a signed number.
_REGISTER = re.compile(r"^\S+\s*\(x(\d+)\)\s*=\s*([0-9a-fA-F]+)\s*$", re.M)
_PC = re.compile(r"^pc\s*=\s*([0-9a-fA-F]+)\s*$", re.M)

#: Start of one core's report. A multicore platform prints one per hart.
_CORE_HEADER = re.compile(r"^=\[ core : (\d+) \]=+", re.M)


def parse_registers(stdout):
    """Return the register file at the end of the run, one entry per core.

    This is the cheapest functional check the VP offers. Two builds that execute the
    same program must end with the same registers, so recording them turns a
    refactor of the simulator into something a test can verify, not just the trace.

    Each entry is {"hart": n, "pc": int, "x0": int, ... "x31": int}. Registers the
    dump does not name are absent rather than zero, so a format change shows up as
    a missing key instead of a wrong value.
    """
    text = _ANSI.sub("", stdout)
    sections = []
    matches = list(_CORE_HEADER.finditer(text))
    if not matches:
        return sections
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        body = text[match.start():end]
        entry = {"hart": int(match.group(1))}
        for register in _REGISTER.finditer(body):
            entry[f"x{int(register.group(1))}"] = int(register.group(2), 16)
        pc = _PC.search(body)
        if pc:
            entry["pc"] = int(pc.group(1), 16)
        if len(entry) > 1:
            sections.append(entry)
    return sections


def _parse_counters(stdout):
    """Pull the counters out of what the VP printed.

    The VP has no machine readable report, so this reads the human one. Every
    counter is optional: a platform that does not run the analysis prints fewer
    lines, and that is not an error here.
    """
    counters = {}
    for name, pattern in _COUNTERS:
        match = pattern.search(stdout)
        if match:
            counters[name] = int(match.group(1))
    return counters


def run(vp, program, out_dir, flags=(), timeout=1800, env_extra=None):
    """Run one workload once and return what it cost and what it printed.

    `out_dir` is created if it is absent and is passed to the VP with a trailing
    separator, which the VP needs because it builds filenames by concatenation.

    Peak memory comes from `os.wait4`, which reports the resource usage of that
    one child. `resource.getrusage(RUSAGE_CHILDREN)` would report the maximum
    over every child so far, which is wrong as soon as you run twice.
    """
    vp = Path(vp)
    program = Path(program)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    command = [str(vp), "--intercept-syscalls", str(program),
               "--output-file", f"{out_dir}{os.sep}", *flags]

    env = dict(os.environ)
    # The banner is three lines on every run and carries no information.
    env["SYSTEMC_DISABLE_COPYRIGHT_MESSAGE"] = "1"
    if env_extra:
        env.update(env_extra)

    started = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, env=env,
                               cwd=str(REPO_ROOT))
    try:
        stdout = process.stdout.read()
        _, status, usage = os.wait4(process.pid, 0)
    except BaseException:
        process.kill()
        process.wait()
        raise
    finally:
        process.stdout.close()
    process.returncode = os.waitstatus_to_exitcode(status) if hasattr(
        os, "waitstatus_to_exitcode") else (status >> 8)
    wall_s = time.monotonic() - started

    text = stdout.decode("utf-8", "replace")
    result = {
        "command": command,
        "exit_code": process.returncode,
        "wall_s": round(wall_s, 3),
        "peak_rss_kb": usage.ru_maxrss,
        "user_s": round(usage.ru_utime, 3),
        "system_s": round(usage.ru_stime, 3),
        "counters": _parse_counters(text),
        "registers": parse_registers(text),
        "stdout": text,
    }
    instructions = result["counters"].get("instructions")
    if instructions and wall_s > 0:
        result["instructions_per_s"] = int(instructions / wall_s)
    return result


# --------------------------------------------------------------------------- #
# digesting the output
# --------------------------------------------------------------------------- #

def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def canonical_json(raw):
    """Return the bytes of `raw` re-serialised with sorted keys and an indent.

    Two uses: as the input to the semantic digest, and as the readable form to
    write out when a digest mismatches and someone has to look at the diff.
    Returns None when `raw` is not JSON, which is how the CSV exports are told
    apart from the trees.
    """
    try:
        parsed = json.loads(raw)
    except (ValueError, UnicodeDecodeError):
        return None
    return json.dumps(parsed, sort_keys=True, indent=1).encode("utf-8")


def digest_file(path):
    """Return the digests and size of one output file."""
    raw = Path(path).read_bytes()
    entry = {"bytes": len(raw), "raw_sha256": _sha256(raw)}
    canonical = canonical_json(raw)
    if canonical is not None:
        entry["semantic_sha256"] = _sha256(canonical)
    return entry


def manifest(out_dir):
    """Reduce an output directory to one digest entry per file, path ordered.

    The path is relative to `out_dir` and uses forward slashes, so a manifest
    taken on one machine compares against one taken on another.
    """
    out_dir = Path(out_dir)
    entries = {}
    for path in sorted(out_dir.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix in NON_TRACE_SUFFIXES:
            continue
        entries[path.relative_to(out_dir).as_posix()] = digest_file(path)
    return entries


def manifest_digest(entries):
    """One digest standing for a whole manifest, for a single number comparison."""
    payload = json.dumps(entries, sort_keys=True, separators=(",", ":"))
    return _sha256(payload.encode("utf-8"))


# --------------------------------------------------------------------------- #
# describing the environment
# --------------------------------------------------------------------------- #

def _git(*args):
    try:
        done = subprocess.run(["git", "-C", str(REPO_ROOT), *args],
                              capture_output=True, text=True, check=False)
    except OSError:
        return ""
    return done.stdout.strip() if done.returncode == 0 else ""


def revision():
    """The commit a measurement was taken against, and whether the tree was dirty."""
    return {
        "commit": _git("rev-parse", "HEAD"),
        "short": _git("rev-parse", "--short=6", "HEAD"),
        "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git("status", "--porcelain")),
    }


def build_settings():
    """The build options that change what the VP does, read from the CMake cache.

    Tracing features are fixed when the VP is configured, so two measurements
    taken with different settings are not comparable. Recording them is what
    lets a later comparison say so instead of reporting the difference as a
    result.
    """
    cache = REPO_ROOT / "vp" / "build" / "CMakeCache.txt"
    wanted = ("CMAKE_BUILD_TYPE", "INSTRUCTION_TREE_DEPTH",
              "NO_TRACE_PARAMETER_IMMEDIATES", "NO_TRACE_PREDECESSOR_PCS",
              "NO_TRACE_BRANCH_OUTCOMES", "TRACE_ROOT_PARAMETERS",
              "USE_SYSTEM_SYSTEMC")
    settings = {}
    if not cache.is_file():
        return settings
    for line in cache.read_text(errors="replace").splitlines():
        if ":" not in line or "=" not in line or line.startswith("//"):
            continue
        key = line.split(":", 1)[0]
        if key in wanted:
            settings[key] = line.split("=", 1)[1]
    return settings


def environment():
    """Everything about this machine and build that a later comparison must check."""
    return {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "platform": platform.platform(),
        "processor": platform.processor() or platform.machine(),
        "cpu_count": os.cpu_count(),
        "python": platform.python_version(),
        "revision": revision(),
        "build": build_settings(),
    }


# --------------------------------------------------------------------------- #
# small shared helpers for the two front ends
# --------------------------------------------------------------------------- #

def load_cases(path):
    """Read a case list. See vp/tests/trace/cases.json for the shape."""
    path = Path(path)
    if not path.is_file():
        raise VpError(f"no case list at {path}")
    data = json.loads(path.read_text())
    if "cases" not in data:
        raise VpError(f"{path} has no 'cases' key")
    return data


def note(message):
    print(message, file=sys.stderr, flush=True)
