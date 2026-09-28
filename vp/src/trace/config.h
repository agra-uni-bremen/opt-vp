#pragma once

// What to record and what to write out. Set once, from the command line, before the
// simulation starts.
//
// Every platform builds one of these with `Options::trace_config()` and hands it to its
// core's tracer. To add a tracing option, add a field here, parse it in
// vp/src/platform/common/options.cpp, and copy it across in `Options::trace_config()`.
// No platform needs to change.

#include <string>

struct TraceConfig {
	/**
	 * Length of the longest window the tracer records, which is also the depth of the trees.
	 *
	 * Zero keeps the depth the VP was compiled with, `INSTRUCTION_TREE_DEPTH`. A larger value
	 * is clamped to it, because the nodes hold fixed size arrays of that length. Tree depth has a large impact on performance.
	 */
	unsigned int depth = 0;

	//! One JSON file per tree following the JITR schema. The export the RETrace frontend reads.
	bool write_trees = false;
	//! One dot graph per tree, plus a memory map. Dot graph rendering  is generally replaced by the trace explorer in RETrace
	bool write_dot = false;
	/**
	 * Omit any branch of a dot graph carrying less than this share of its tree's weight.
	 * Zero draws every branch, which is unreadable for anything but a tiny program.
	 */
	float graph_branch_threshold = 0.05f;
	//! One csv row per node of every tree.
	bool write_csv = false;
	//! The best sequences with their sub sequences and branch variants, as one JSON file.
	bool write_sequences = false;
	//! The best sequences with their coverage, as csv. Needs `coverage_csv_file`.
	bool write_coverage_csv = false;

	/**
	 * Where the coverage csv goes. A relative path is taken relative to `output_directory`,
	 * an absolute one as given.
	 */
	std::string coverage_csv_file;
	//! How many sequences to keep per tree, and how many rows the coverage csv gets.
	unsigned int coverage_top_n = 10;
	/**
	 * How similar two sequences may be before the second is dropped as a near duplicate,
	 * between 0 and 1. See `is_similar_sequence` in analysis.h.
	 */
	float coverage_similarity_threshold = 0.2f;

	//! Take commands after the report instead of returning. See interactive.cpp.
	bool interactive = false;
	//! Shared library the interactive mode loads scoring functions from.
	std::string scoring_library = "./vp/build/lib/libfunctions.so";

	/**
	 * Directory every export is written into, with a trailing separator, or empty to write to
	 * standard output. Use `set_output_directory`, which appends the separator: filenames are
	 * built by concatenation, so a missing one silently writes into the parent directory.
	 */
	std::string output_directory;
	//! Path of the simulated program. Its basename goes into every exported filename.
	std::string input_program;
	//! Distinguishes the files of one core from another's on a multicore platform.
	unsigned int hart_id = 0;

	//! True when anything at all has to be written to a file.
	bool writes_any_file() const {
		return write_trees || write_dot || write_csv || write_sequences || write_coverage_csv;
	}

	//! Set `output_directory`, appending the separator the exporters rely on.
	void set_output_directory(const std::string &directory) {
		output_directory = directory;
		if (!output_directory.empty() && output_directory.back() != '/') {
			output_directory.push_back('/');
		}
	}
};
