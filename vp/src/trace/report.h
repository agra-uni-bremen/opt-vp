#pragma once

// The post simulation report: analyse the execution sequence trees, print the best
// sequences and write whichever exports were asked for.
//
// This used to be `ISS::show()` and its `output_*` helpers, written out once in
// vp/src/core/rv32/iss.cpp and again in vp/src/core/rv64/iss.cpp. None of it depends on
// the register width: it works on `InstructionNodeR`, `Path` and `std::ofstream`. It now
// exists once, and an ISS reaches it by filling in a `TraceReport` and calling
// `run_trace_report`.

#include "trace/trace.h"

#include <array>
#include <cstdint>
#include <list>
#include <map>
#include <string>
#include <tuple>

//! Where a memory access came from: address -> (last write pc, last read pc). The dot
//! export prints it; nothing else reads it.
using MemoryAccessMap = std::map<uint64_t, std::tuple<uint64_t, uint64_t>>;

/**
 * Everything the report needs from the core that produced the trees.
 *
 * The trees are borrowed, not copied: the report prunes and extends them in place when
 * the interactive mode asks it to, and the caller owns them for the whole call.
 */
struct TraceReport {
	//! One tree per instruction that ever started a window. Borrowed from the core.
	std::list<InstructionNodeR> *trees = nullptr;

	//! Instructions the core retired. Every coverage figure is a share of this.
	uint64_t retired_instructions = 0;
	//! Steps the tracer saw. Equal to the above unless something skipped the tracer.
	uint64_t stepped_instructions = 0;
	//! Cycles the core reported, printed as-is.
	uint64_t cycles = 0;

	//! Borrowed, may be null. Only the dot export reads it.
	const MemoryAccessMap *memory_accesses = nullptr;

	//! Directory the exports are written to, with a trailing separator, or empty for
	//! standard output. Filenames are built by concatenation, so the separator matters.
	std::string output_directory;
	//! Path of the program that was simulated. Its basename goes into every filename.
	std::string input_program;

	bool write_dot = false;
	bool write_csv = false;
	//! The sequence and variant export, `--export-sequences`.
	bool write_sequences = false;
	//! The JITR export, `-e`. This is the one RETrace reads.
	bool write_trees = false;
	bool write_coverage_csv = false;

	std::string coverage_csv_file;
	unsigned int coverage_top_n = 10;
	float coverage_similarity_threshold = 0.2f;

	//! Stay open after the report and take commands. See interactive.cpp.
	bool interactive = false;
	//! Shared library the interactive mode loads scoring functions from.
	std::string scoring_library = "./vp/build/lib/libfunctions.so";
};

/**
 * Insert the windows still sitting in the ring buffer into their trees.
 *
 * During the run a window is inserted only once its oldest instruction is about to be
 * overwritten, so at the end the last `trace_depth - 1` windows have not been inserted
 * yet. Call this once before reading the trees. It advances `ring_buffer_index`.
 */
void flush_ring_buffer(std::list<InstructionNodeR> &trees,
                       std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> &steps,
                       uint8_t &ring_buffer_index);

/**
 * Analyse the trees, print the best sequences and write the requested exports.
 *
 * Prints to standard output. Writes files under `report.output_directory`. Returns when
 * the report is done, or when the interactive mode is quit.
 */
void run_trace_report(TraceReport &report);

//! The basename of a path, which is what every exported filename is built from.
std::string program_basename(const std::string &path);

/**
 * Stay open after the report and take single letter commands: run the analysis again with
 * scoring functions loaded from a shared library, reload that library, prune the trees,
 * or export. See interactive.cpp. `run_trace_report` calls this when `report.interactive`
 * is set; nothing else should.
 */
void run_interactive(TraceReport &report);
