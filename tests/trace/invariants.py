"""Check the properties every JITR trace must have, whatever program produced it.

    python3 tests/trace/invariants.py out/md5sum/ --depth 6

Sanity check for traces:

* a window that reaches a node passes through its parent, so a node's weight is the sum of
  its children's weights plus the windows that end at it (which can only happen when flushing the buffer at the end)
* a window ends before the trace depth only when the program ends, which happens once per
  window length, so over all trees exactly one window ends early at every depth below the
  last (when the program ran at least `depth` instructions);
* `weight` counts every window and `true_weight` only the ones that do not overlap the last
  counted one, so 1 <= true_weight <= weight;
* `register_sets` splits the weight by pc, and `predecessors` splits each pc's count by the pc
  that ran immediately before it. Below the root that is the parent instruction, so its pcs
  are pcs of the parent;
* a dependency is a distance back along the path, so it lies between 1 and the node's depth;
* a branch outcome is counted once per occurrence of its pc;
* every tree has a root per mnemonic, and the root weights add up to the number of
  instructions the tracer recorded.
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import jitr

#: Format version this checker understands. A new version needs a look at the checks.
FORMAT_VERSION = "1.2"


def check(trees, depth, recorded=None):
    """Return a list of human readable problems, empty when the trace is consistent.

    `recorded` is the number of instructions the tracer saw. Leave it None when unknown.
    """
    problems = []
    early_ends = defaultdict(int)  # depth -> windows that end there before the trace depth

    def problem(path, message):
        problems.append(f"{' > '.join(path)}: {message}")

    for mnemonic, root in sorted(trees.items()):
        if root.get("format_version") != FORMAT_VERSION:
            problem((mnemonic,), f"format_version {root.get('format_version')!r}, "
                                 f"expected {FORMAT_VERSION!r}")
        for path, node, parent in jitr.walk(root):
            level = len(path) - 1
            weight, true_weight = node["weight"], node["true_weight"]
            counts = jitr.register_counts(node)
            children = node.get("children", [])

            if node["instruction"] != path[-1]:
                problem(path, "instruction does not match the path")
            if not 1 <= true_weight <= weight:
                problem(path, f"true_weight {true_weight} is not between 1 and weight {weight}")
            if sum(counts.values()) != weight:
                problem(path, f"register_sets counts sum to {sum(counts.values())}, "
                              f"weight is {weight}")
            for pc, preds in jitr.predecessors(node).items():
                if preds and sum(preds.values()) != counts[pc]:
                    problem(path, f"pc {pc}: predecessors sum to {sum(preds.values())}, "
                                  f"count is {counts[pc]}")

            if level >= depth:
                problem(path, f"node at depth {level + 1}, the trace depth is {depth}")
            if level == depth - 1:
                if "children" in node:
                    problem(path, "a node at the trace depth has children")
                if "PCs" not in node:
                    problem(path, "a leaf has no PCs")
                elif {pc: c for pc, c in node["PCs"]} != counts:
                    problem(path, "PCs does not match register_sets")
            else:
                if "children" not in node:
                    problem(path, "a node above the trace depth has no children list")
                names = [c["instruction"] for c in children]
                if len(names) != len(set(names)):
                    problem(path, "two children with the same instruction")
                below = sum(c["weight"] for c in children)
                if below > weight:
                    problem(path, f"children weigh {below}, more than the node's {weight}")
                early_ends[level] += weight - below

            if level > 0:
                parent_counts = jitr.register_counts(parent)
                for pc, preds in jitr.predecessors(node).items():
                    stray = set(preds) - set(parent_counts)
                    if stray:
                        problem(path, f"pc {pc}: predecessor {sorted(stray)} is not a pc of "
                                      f"the parent")

            for key in ("dependencies_true", "dependencies_anti", "dependencies_output"):
                bad = [d for d in node.get(key, []) if not 1 <= d <= level]
                if bad:
                    problem(path, f"{key} {bad} reach outside the window of depth {level + 1}")

            outcomes = node.get("BranchOutcomes", {})
            for pc, outcome in outcomes.items():
                seen = outcome["taken"] + outcome["not_taken"]
                if counts.get(int(pc)) != seen:
                    problem(path, f"pc {pc}: {seen} branch outcomes for "
                                  f"{counts.get(int(pc))} occurrences")

    total = sum(root["weight"] for root in trees.values())
    if recorded is not None and total != recorded:
        problems.append(f"the root weights sum to {total}, the tracer recorded {recorded} "
                        f"instructions")
    if total >= depth:
        for level in range(depth - 1):
            if early_ends[level] != 1:
                problems.append(f"{early_ends[level]} windows end early at depth {level + 1}, "
                                f"exactly one should (the one the program end cuts off)")
    return problems



def main(argv):
    parser = argparse.ArgumentParser(
        prog="invariants.py",
        description="Check that a JITR output directory is internally consistent.")
    parser.add_argument("directory", help="the directory the VP wrote the JITR files to")
    parser.add_argument("--depth", type=int, required=True,
                        help="the --trace-depth the VP ran with")
    args = parser.parse_args(argv)
    trees = jitr.load(args.directory)
    if not trees:
        print(f"no JITR trees in {args.directory}")
        return 1
    problems = check(trees, args.depth)
    for line in problems[:50]:
        print(line)
    if len(problems) > 50:
        print(f"... and {len(problems) - 50} more")
    print(f"{len(trees)} trees, {len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    raise SystemExit(main(sys.argv[1:]))
