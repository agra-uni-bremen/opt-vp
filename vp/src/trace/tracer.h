#pragma once

// Records core execution as a set of execution sequence trees, one tree per
// instruction.
//
// A core owns one `Tracer` and accesses it in three places per executed instruction:
//
//     tracer.begin_step();              // before the instruction runs
//     ExecutionInfo &step = tracer.step();
//     step.last_executed_pc = ...;      // fill instruction info
//     tracer.end_step();                // after
//
// `begin_step` inserts the sequence that ends at the slot about to be overwritten, so a
// sequence reaches its tree only once every instruction in it has run. The last few sequences
// are therefore still in the buffer when the program ends. `flush` inserts the remaining ones.
//
// The three functions are defined here rather than in the .cpp. They run once
// per simulated instruction, which is hundreds of millions of times per run.

// Build the VP with -DNO_EXECUTION_TRACING to leave the integration out of the core
// altogether. `TRACING_COMPILED_IN` is what the core tests; it folds the runtime check away,
// so nothing of the tracer is left in the executed code.
#ifdef NO_EXECUTION_TRACING
#define TRACING_COMPILED_IN false
#else
#define TRACING_COMPILED_IN true
#endif

#include "trace/config.h"
#include "trace/trace.h"

#include <array>
#include <cstdint>
#include <list>

class Tracer {
  public:
	Tracer() = default;

	/**
	 * Apply the settings. Call once before the first instruction, because depth constraints 
	 * the ring buffer and cannot change once windows have been recorded.
	 */
	void configure(const TraceConfig &config);

	const TraceConfig &config() const {
		return config_;
	}

	//! One tree per instruction that started a window.
	std::list<InstructionNode> &trees() {
		return trees_;
	}

	// ------------------------------------------------------------------ hot path

	/**
	 * Whether to record anything. The core tests this before every piece of tracing work, so
	 * `--no-trace` costs one predictable branch per instruction (about 0.3% overhead compared to compile time macros, which I think is a reasonable price for the flexibility).
	 */
	bool enabled() const {
		return TRACING_COMPILED_IN && enabled_;
	}

	/**
	 * Whether the core has to keep the map of every address it touched.
	 *
	 * Only the dot export reads it, and building it is an ordered map insert on every load and
	 * every store.
	 */
	bool records_memory_map() const {
		return TRACING_COMPILED_IN && records_memory_map_;
	}

	/**
	 * Insert the sequence that ends at the slot about to be overwritten.
	 *
	 * Does nothing while the buffer is still filling up, which is the first `depth` steps of
	 * a run: an unwritten slot holds `Opcode::UNDEF` (0).
	 */
	void begin_step() {
		Opcode::Mapping oldest_op = steps_[index_].last_executed_instruction;
		if (oldest_op) {
			tree_for(oldest_op).insert_rb(steps_, index_);
		}
	}

	//! The slot describing the instruction that just ran. Filled by the core.
	ExecutionInfo &step() {
		return steps_[index_];
	}

	//! The slot the previous step filled in, which is where a predecessor pc comes from.
	const ExecutionInfo &previous_step() const {
		return steps_[index_ > 0 ? index_ - 1 : trace_depth - 1];
	}

	//! The step `back` positions before the current one. Used for progress output.
	const ExecutionInfo &recent_step(unsigned back) const {
		return steps_[(index_ + trace_depth - back) % trace_depth];
	}

	//! Move on to the next slot. Wrapping by comparison rather than by division is faster. 
	void end_step() {
		if (++index_ >= trace_depth) {
			index_ = 0;
		}
	}

	// ------------------------------------------------------------------ end of run

	/**
	 * Insert the sequences still sitting in the ring buffer. Call once after the simulation and
	 * before reading the trees, or the last `depth - 1` sequences are missing.
	 */
	void flush();

	/**
	 * Analyse the trees, print the best sequences and write the requested exports.
	 *
	 * `retired` is the instruction count every coverage figure is a share of, `stepped` is
	 * what the tracer saw, and `memory_accesses` is only read by the dot export.
	 */
	void report(uint64_t retired, uint64_t stepped, uint64_t cycles,
	            const MemoryAccessMap *memory_accesses);

  private:
	//! The tree for an opcode. Looked up by opcode rather than searched for: this runs once per
	//! executed instruction, and a real program reaches a hundred trees.
	InstructionNode &tree_for(Opcode::Mapping op) {
		InstructionNode *root = roots_[op];
		return root ? *root : start_tree(op);
	}

	/**
	 * Start the tree for an opcode, on its first occurrence.
	 *
	 * The root of a tree is a plain node whatever its opcode, so it carries no memory or branch
	 * payload. `trees_` is a list, so a root keeps its address as further trees are added.
	 */
	InstructionNode &start_tree(Opcode::Mapping op);

	TraceConfig config_;
	//! The last `trace_depth` steps, oldest at `index_`.
	std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> steps_{};
	//! The trees in the order they were started, which is the order the exports write them in.
	std::list<InstructionNode> trees_;
	//! The same trees indexed by their root opcode. Null until an opcode occurs.
	std::array<InstructionNode *, Opcode::NUMBER_OF_INSTRUCTIONS> roots_{};
	uint8_t index_ = 0;
	bool enabled_ = true;
	bool records_memory_map_ = false;
};
