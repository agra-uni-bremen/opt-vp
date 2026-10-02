"""A second, independent implementation of the JITR trace, to check the VP's against.

    python3 tests/trace/model.py sw/trace-test-default --depth 6
    python3 tests/trace/model.py sw/trace-test-default --depth 6 --strict

The VP's tracer builds its trees incrementally from a ring buffer, inside the simulation. This
model starts from the other end: it runs the program once with `--trace-mode`, which prints
every executed instruction with its pc and operands, and computes from that list what every
node of every tree must hold. Then it compares node by node with the JITR files of the same
run. A difference is a bug in one of the two, and the model is short enough to read in one
sitting.

What the model computes, for every window of up to `depth` consecutive executed instructions:

* the tree shape, `weight` and `true_weight` (greedy count of non-overlapping occurrences);
* `register_sets`: the count per pc, the pc that ran immediately before, and the register
  numbers;
* `dependencies_true/anti/output`, `inputs` and `outputs` from the register operands;
* `BranchOutcomes` of the branch and jump nodes below the root.

What it does not compute: `parameters`, the memory fields, and dependencies through memory,
because `--trace-mode` prints no values or addresses. A node whose window holds a load or a
store may therefore carry a memory dependency the model cannot see. The model accepts such a
dependency only where one is possible: a load depending on an earlier store, or a store on an
earlier load or store.

The model supports the RV32IM base instructions and ECALL, which is what the `trace-test-*`
programs use. A program that executes anything else, traps, or uses compressed instructions
is rejected with a message, rather than compared with a model that does not describe it.

Known deviations. Where the VP's tracer currently differs from the architectural meaning of
an instruction, `VP_DEVIATIONS` names the difference and the model reproduces it, so the check
passes and still pins everything else. `--strict` turns them off and shows where they apply.
Fixing one in the VP means deleting its entry here.
"""

import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "scripts"))
sys.path.insert(0, str(HERE))
import jitr  # noqa: E402
import vpbench  # noqa: E402

VP_DEVIATIONS = {
    "x0-dependencies":
        "a read or a write of x0 creates anti and output dependencies, although x0 holds no "
        "value. True dependencies already ignore x0",
    "shift-amount-as-rs2":
        "SLLI, SRLI and SRAI are classified as R-type, so the shift amount field is read as "
        "an rs2 register number and creates dependencies on that register",
    "rd-field-of-instructions-without-rd":
        "branches and ECALL have no destination register, but the bits in the rd field "
        "(part of a branch's immediate) are used as one for anti and output dependencies",
    "last-full-window-missing":
        "Tracer::flush starts one slot after the oldest window still in the ring buffer, so the "
        "last window of full depth, the one that ends at the program's final instruction, never "
        "reaches its tree",
    "ecall-source-fields":
        "ECALL has no source registers, but its rs1 and rs2 fields (both zero) are reported as "
        "inputs",
}

# Operand roles by instruction format, as the RISC-V specification defines them.
R_TYPE = {"ADD", "SUB", "SLL", "SLT", "SLTU", "XOR", "SRL", "SRA", "OR", "AND",
          "MUL", "MULH", "MULHSU", "MULHU", "DIV", "DIVU", "REM", "REMU"}
SHIFT_IMMEDIATE = {"SLLI", "SRLI", "SRAI"}
I_TYPE = {"ADDI", "SLTI", "SLTIU", "XORI", "ORI", "ANDI", "JALR",
          "LB", "LH", "LW", "LBU", "LHU"} | SHIFT_IMMEDIATE
LOADS = {"LB", "LH", "LW", "LBU", "LHU"}
STORES = {"SB", "SH", "SW"}
BRANCHES = {"BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU"}
U_TYPE = {"LUI", "AUIPC"}
NO_OPERANDS = {"ECALL"}
SUPPORTED = R_TYPE | I_TYPE | STORES | BRANCHES | U_TYPE | {"JAL"} | NO_OPERANDS

_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_STEP = re.compile(r"^core\s+\d+: prv \d: pc\s+([0-9a-f]+): (\S+) ?(.*)$")
_REGISTER = re.compile(r"\(x(\d+)\)")


class Step:
    """One executed instruction as `--trace-mode` printed it."""

    def __init__(self, pc, mnemonic, operands):
        self.pc = pc
        self.mnemonic = mnemonic
        registers = [int(r) for r in _REGISTER.findall(operands)]
        immediates = re.findall(r"0x([0-9a-f]+)\s*$", operands)
        self.imm = None
        if immediates:
            value = int(immediates[0], 16)
            self.imm = value - (1 << 32) if value >= 1 << 31 else value
        # rd, rs1, rs2 in the VP's encoding: -1 where the format has no such operand.
        self.rd = self.rs1 = self.rs2 = -1
        self.shift_field = None
        if mnemonic in R_TYPE:
            self.rd, self.rs1, self.rs2 = registers
        elif mnemonic in SHIFT_IMMEDIATE:
            # The VP prints the shift amount field as a third register.
            self.rd, self.rs1, self.shift_field = registers
        elif mnemonic in I_TYPE:
            self.rd, self.rs1 = registers
        elif mnemonic in STORES or mnemonic in BRANCHES:
            self.rs1, self.rs2 = registers
        elif mnemonic in U_TYPE or mnemonic == "JAL":
            (self.rd,) = registers


def read_steps(stdout):
    """Turn `--trace-mode` output into a list of Steps, or raise if the model cannot use it."""
    steps = []
    for line in _ANSI.sub("", stdout).splitlines():
        match = _STEP.match(line)
        if match:
            steps.append(Step(int(match.group(1), 16), match.group(2), match.group(3)))
    if not steps:
        raise vpbench.VpError("the VP printed no instructions. Does it support --trace-mode?")
    unsupported = sorted({s.mnemonic for s in steps} - SUPPORTED)
    if unsupported:
        raise vpbench.VpError(f"the model does not describe {', '.join(unsupported)}")
    for before, after in zip(steps, steps[1:]):
        if before.mnemonic in BRANCHES | {"JAL", "JALR"}:
            continue
        if after.pc != before.pc + 4:
            raise vpbench.VpError(
                f"pc {after.pc:#x} follows {before.mnemonic} at {before.pc:#x}: a trap or a "
                f"compressed instruction, which the model does not describe")
    return steps


class Node:
    """What the model expects of one tree node."""

    def __init__(self):
        self.weight = 0
        self.true_weight = 0
        self.last_counted_start = None
        self.counts = defaultdict(int)
        self.predecessors = defaultdict(lambda: defaultdict(int))
        self.registers = {}
        self.dependencies = {"dependencies_true": set(), "dependencies_anti": set(),
                             "dependencies_output": set()}
        self.inputs = set()
        self.outputs = set()
        self.outcomes = {}
        self.children = set()
        self.memory_window = False


def reads_and_write(step, deviations):
    """Registers an instruction reads and the one it writes, as the tracer should see them."""
    m = step.mnemonic
    reads = [r for r in (step.rs1, step.rs2) if r >= 0]
    write = step.rd if step.rd >= 0 else None
    if m in SHIFT_IMMEDIATE and "shift-amount-as-rs2" in deviations:
        reads.append(step.shift_field)
    return reads, write


def phantom_rd(step, deviations):
    """The register a branch or ECALL is treated as writing, for anti and output dependencies."""
    if "rd-field-of-instructions-without-rd" not in deviations:
        return None
    if step.mnemonic == "ECALL":
        return 0
    if step.mnemonic in BRANCHES:
        # rd occupies bits 11..7, which in a branch hold imm[4:1] and imm[11].
        imm = step.imm & 0x1fff
        return ((imm >> 1) & 0xf) << 1 | ((imm >> 11) & 1)
    return None


def build(steps, depth, deviations):
    """Return {path: Node} for every window of the run."""
    nodes = defaultdict(Node)
    n = len(steps)
    for start in range(n):
        if "last-full-window-missing" in deviations and start == n - depth:
            continue
        length = min(depth, n - start)
        last_writer = {}
        readers = defaultdict(set)
        writers = defaultdict(set)
        for i in range(length):
            step = steps[start + i]
            path = tuple(s.mnemonic for s in steps[start:start + i + 1])
            node = nodes[path]
            if i > 0:
                nodes[path[:-1]].children.add(step.mnemonic)

            node.weight += 1
            if node.last_counted_start is None or start > node.last_counted_start + i:
                node.true_weight += 1
                node.last_counted_start = start
            node.counts[step.pc] += 1
            index = start + i
            node.predecessors[step.pc][steps[index - 1].pc if index > 0 else 0] += 1
            node.registers[step.pc] = (step.rd, step.rs1, step.rs2)
            if step.mnemonic in SHIFT_IMMEDIATE and "shift-amount-as-rs2" in deviations:
                node.registers[step.pc] = (step.rd, step.rs1, step.shift_field)

            reads, write = reads_and_write(step, deviations)
            if step.mnemonic == "ECALL" and "ecall-source-fields" in deviations:
                node.inputs.add(0)
            for register in reads:
                if register in last_writer:
                    node.dependencies["dependencies_true"].add(i - last_writer[register])
                else:
                    node.inputs.add(register)
            target = write if write is not None else phantom_rd(step, deviations)
            if target == 0 and "x0-dependencies" not in deviations:
                target = None
            if target is not None:
                node.dependencies["dependencies_anti"] |= {i - j for j in readers[target]}
                node.dependencies["dependencies_output"] |= {i - j for j in writers[target]}
            if write is not None and write != 0:
                node.outputs.add(write)
            # x0 is a source operand, so it counts as an input above, but it holds no value,
            # so it carries no dependency. The tracer already leaves it out of true ones.
            x0_carries = "x0-dependencies" in deviations
            for register in reads:
                if register != 0 or x0_carries:
                    readers[register].add(i)
            if write is not None and (write != 0 or x0_carries):
                writers[write].add(i)
            if write is not None and write != 0:
                last_writer[write] = i

            if any(s.mnemonic in LOADS | STORES for s in steps[start:start + i + 1]):
                node.memory_window = True

            if i > 0 and index + 1 < n:
                if step.mnemonic in BRANCHES:
                    taken = steps[index + 1].pc != step.pc + 4
                    outcome = node.outcomes.setdefault(step.pc, [step.imm, 0, 0])
                    outcome[1 if taken else 2] += 1
                elif step.mnemonic in {"JAL", "JALR"}:
                    outcome = node.outcomes.setdefault(step.pc, [step.imm, 0, 0])
                    outcome[1] += 1
    return nodes


def memory_dependency_possible(path, kind, offset):
    """Whether the dependency `offset` back from the end of `path` can come from memory."""
    this, other = path[-1], path[-1 - offset]
    if kind == "dependencies_true":
        return this in LOADS and other in STORES
    if kind == "dependencies_anti":
        return this in STORES and other in LOADS
    return this in STORES and other in STORES


def compare(expected, trees):
    """Return a list of differences between the model and the JITR trees."""
    problems = []
    seen = set()

    def problem(path, message):
        problems.append(f"{' > '.join(path)}: {message}")

    for mnemonic, root in sorted(trees.items()):
        for path, node, _ in jitr.walk(root):
            seen.add(path)
            want = expected.get(path)
            if want is None:
                problem(path, "in the trace, but the program never executed this window")
                continue
            for key, value in (("weight", want.weight), ("true_weight", want.true_weight)):
                if node[key] != value:
                    problem(path, f"{key} {node[key]}, model {value}")
            if jitr.register_counts(node) != dict(want.counts):
                problem(path, f"pc counts {jitr.register_counts(node)}, model {dict(want.counts)}")
            want_preds = {pc: dict(p) for pc, p in want.predecessors.items()}
            if jitr.predecessors(node) != want_preds:
                problem(path, f"predecessors {jitr.predecessors(node)}, model {want_preds}")
            for pc, entry in node.get("register_sets", {}).items():
                rd, rs1, rs2 = want.registers[int(pc)]
                got = (entry["rd"], entry["rs1"], entry["rs2"])
                checked = (rd, rs1, rs2) if path[-1] not in STORES | BRANCHES | NO_OPERANDS \
                    else (None, rs1, rs2) if path[-1] not in NO_OPERANDS else (None, None, None)
                for name, value, actual in zip(("rd", "rs1", "rs2"), checked, got):
                    if value is not None and value != actual:
                        problem(path, f"pc {pc}: {name} {actual}, model {value}")
            for key, value in want.dependencies.items():
                got = set(node.get(key, []))
                missing = value - got
                extra = {d for d in got - value
                         if not (want.memory_window and memory_dependency_possible(path, key, d))}
                if missing or extra:
                    problem(path, f"{key} {sorted(got)}, model {sorted(value)}")
            for key, value in (("inputs", want.inputs), ("outputs", want.outputs)):
                if set(node.get(key, [])) != value:
                    problem(path, f"{key} {sorted(node.get(key, []))}, model {sorted(value)}")
            children = {c["instruction"] for c in node.get("children", [])}
            if children != want.children:
                problem(path, f"children {sorted(children)}, model {sorted(want.children)}")
            outcomes = {int(pc): [o["offset"], o["taken"], o["not_taken"]]
                        for pc, o in node.get("BranchOutcomes", {}).items()}
            if len(path) > 1 and outcomes != want.outcomes:
                problem(path, f"BranchOutcomes {outcomes}, model {want.outcomes}")
    for path in sorted(set(expected) - seen):
        problem(path, "the program executed this window, but the trace has no node for it")
    return problems


def run(vp, program, out_dir, depth, flags=()):
    """Run the VP once with tracing and --trace-mode. Return (steps, trees)."""
    result = vpbench.run(vp, program, out_dir, ["-e", "--trace-mode", "--trace-depth",
                                                str(depth), *flags])
    if result["exit_code"] != 0 or result.get("timed_out"):
        raise vpbench.VpError(f"the VP did not finish cleanly:\n  {' '.join(result['command'])}")
    return read_steps(result["stdout"]), jitr.load(out_dir)


def check(vp, program, out_dir, depth, flags=(), strict=False):
    """Run `program` and return the differences between its trace and the model."""
    steps, trees = run(vp, program, out_dir, depth, flags)
    deviations = set() if strict else set(VP_DEVIATIONS)
    return compare(build(steps, depth, deviations), trees)


def main(argv):
    parser = argparse.ArgumentParser(
        prog="model.py",
        description="Compare the VP's JITR trace of a program against an independent model.")
    parser.add_argument("program", help="an ELF file, or a directory under sw/ holding `main`")
    parser.add_argument("--depth", type=int, default=6, help="trace depth (default 6)")
    parser.add_argument("--vp", default="tiny32-vp", help="VP binary (default tiny32-vp)")
    parser.add_argument("--strict", action="store_true",
                        help="model the architectural meaning, without the known deviations")
    args = parser.parse_args(argv)
    out_dir = vpbench.REPO_ROOT / "out" / "trace-model" / Path(args.program).name
    try:
        problems = check(vpbench.find_vp(args.vp), vpbench.resolve_program(args.program),
                         out_dir, args.depth, strict=args.strict)
    except vpbench.VpError as error:
        print(error)
        return 1
    for line in problems[:40]:
        print(line)
    if len(problems) > 40:
        print(f"... and {len(problems) - 40} more")
    print(f"{len(problems)} differences between the trace and the model")
    return 1 if problems else 0


if __name__ == "__main__":
    os.environ.setdefault("SYSTEMC_DISABLE_COPYRIGHT_MESSAGE", "1")
    raise SystemExit(main(sys.argv[1:]))
