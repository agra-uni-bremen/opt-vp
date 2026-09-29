#pragma once

// The post simulation report: analyse the execution sequence trees, print the best
// sequences and write whichever exports were asked for.
//
// A core does not call this directly. It fills in a `TraceConfig`, hands it to its
// `Tracer`, and calls `Tracer::report`, which builds the `TraceReport` below.

#include "trace/config.h"
#include "trace/trace.h"

#include <cstdint>
#include <list>
#include <string>

/**
 * One run of the report: the settings, the trees to read, and the counters to divide by.
 *
 * The trees are borrowed for the length of the call. The interactive mode prunes and
 * extends them in place, so the caller must not read them while the report runs.
 */
struct TraceReport {
	const TraceConfig &config;
	std::list<InstructionNode> &trees;

	//! Instructions the core retired. Every coverage figure is a share of this.
	uint64_t retired_instructions = 0;
	//! Steps the tracer saw, printed beside the above so a mismatch is visible.
	uint64_t stepped_instructions = 0;
	//! Cycles the core reported, printed as given.
	uint64_t cycles = 0;
	//! Borrowed, may be null. Only the dot export reads it.
	const MemoryAccessMap *memory_accesses = nullptr;
};

//! Analyse the trees, print the best sequences and write the requested exports.
void run_trace_report(TraceReport &report);

/**
 * Take single letter commands: run the analysis again with scoring functions loaded from a
 * shared library, reload that library, prune the trees, or export. Returns on `q` or when
 * standard input closes.
 */
void run_interactive(TraceReport &report);

//! The basename of a path, which is what every exported filename is built from.
std::string program_basename(const std::string &path);

/**
 * Goes into every exported filename so the cores of a multicore platform do not overwrite
 * each other's files. Empty for hart 0, `-hart<n>` for the rest.
 */
std::string hart_suffix(const TraceConfig &config);
