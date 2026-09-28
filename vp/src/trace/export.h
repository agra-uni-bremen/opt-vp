#pragma once

// The exporters. One per format.
//
// A note on `std::cout`. The dot and csv exporters still redirect `std::cout` at a file
// while they run, because the node classes write through `std::cout` themselves
// (`InstructionNodeR::tree_to_dot`, `to_csv`). Threading a stream through the node
// hierarchy belongs with rewriting that hierarchy, so it is left for then. The two JSON
// exporters, which build a value and write it in one place, take a stream and do not touch
// `std::cout`.

#include "trace/report.h"
#include "trace/trace.h"

#include <functional>
#include <ostream>
#include <string>
#include <vector>

//! Write one dot graph per tree, or one combined graph when there is no output directory.
void export_dot(const TraceReport &report);

//! Write the per-node csv dump of every tree.
void export_csv(const TraceReport &report);

//! Write the top sequences with their coverage as csv. Does nothing when no file is set.
void export_coverage_csv(const TraceReport &report, const std::vector<Path> &sequences,
                         const std::function<float(const ScoreParams)> &score_function);

//! Write the discovered sequences, their sub sequences and their variants as one JSON file.
void export_sequences(const TraceReport &report,
                      const std::vector<std::vector<PathNode>> &sequences,
                      const std::vector<std::vector<std::vector<PathNode>>> &sub_sequences,
                      const std::vector<std::vector<std::vector<PathNode>>> &variants);

//! Write the JITR trace: one JSON file per tree. This is the export RETrace reads.
void export_trees(const TraceReport &report);
