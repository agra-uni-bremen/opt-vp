"""Read a JITR output directory into plain Python values.

A JITR directory holds one JSON file per root instruction, `<program><MNEMONIC>.json`. Each
file is one execution sequence tree: every node is one executed instruction, and the path from
the root to a node is one window of consecutively executed instructions.

`load(directory)` returns {mnemonic: root node}, where a node is the parsed JSON object with
its `children` list left in place. `walk(root)` yields (path, node, parent) for every node,
where `path` is the tuple of mnemonics from the root to that node and `parent` is None at the
root.
"""

import json
from pathlib import Path


def load(directory):
    """Return {mnemonic: root node} for every tree in `directory`."""
    trees = {}
    for path in sorted(Path(directory).glob("*.json")):
        root = json.loads(path.read_text())
        if "format_version" not in root:
            continue  # not a JITR tree, for example a sequences export
        trees[root["instruction"]] = root
    return trees


def walk(node, path=(), parent=None):
    """Yield (path, node, parent) for `node` and every node below it, depth first."""
    path = path + (node["instruction"],)
    yield path, node, parent
    for child in node.get("children", []):
        yield from walk(child, path, node)


def register_counts(node):
    """{pc: count} of a node, from `register_sets`."""
    return {int(pc): entry["count"] for pc, entry in node.get("register_sets", {}).items()}


def predecessors(node):
    """{pc: {predecessor pc: count}} of a node, from `register_sets`."""
    return {int(pc): {int(p): c for p, c in entry.get("predecessors", {}).items()}
            for pc, entry in node.get("register_sets", {}).items()}
