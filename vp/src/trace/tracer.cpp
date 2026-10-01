#include "trace/tracer.h"

#include "trace/report.h"

#include <cassert>

void Tracer::configure(const TraceConfig &config) {
	config_ = config;
	//a VP built with -DNO_EXECUTION_TRACING records nothing and overrides what the command line said 
	//the report reads it to decide what it can still say.
	config_.enabled = TRACING_COMPILED_IN && config.enabled;
	enabled_ = config_.enabled;
	records_memory_map_ = config_.enabled && config_.write_dot;
	// Clamps to the depth the VP was compiled with and warns if the request was out of range.
	set_trace_depth(config_.depth);
}

InstructionNode &Tracer::start_tree(Opcode::Mapping op) {
	assert(op < Opcode::NUMBER_OF_INSTRUCTIONS);
	trees_.emplace_back(op, 0, NODE_TYPE::NODE);
	roots_[op] = &trees_.back();
	return trees_.back();
}

void Tracer::flush() {
	if (!enabled()) {
		return;
	}
	// During the run a window is inserted only once its oldest instruction is about to be
	// overwritten, so the last trace_depth - 1 windows have not reached their trees yet.
	// Walk the rest of the buffer and insert them, shortest window last.
	for (size_t offset = 1; offset < trace_depth; offset++) {
		index_ = (index_ + 1) % trace_depth;

		Opcode::Mapping oldest_op = steps_[index_].last_executed_instruction;
		if (!oldest_op) {
			continue;  // the buffer was never completely filled, skip the empty slot
		}
		tree_for(oldest_op).insert_rb(steps_, index_, offset);
	}
}

void Tracer::report(uint64_t retired, uint64_t stepped, uint64_t cycles,
                    const MemoryAccessMap *memory_accesses) {
	TraceReport report{config_, trees_, retired, stepped, cycles, memory_accesses};
	run_trace_report(report);
}
