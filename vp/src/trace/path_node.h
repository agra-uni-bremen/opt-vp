#pragma once

// One node of an identified sequence used by the sequences export.
//
// A `PathNode` is built after the simulation, from an `InstructionNode` on a chosen path, and
// carries only what that export needs. It is separate from trace.h so that the json library
// stays out of every file that includes the tracer. (Gives a good improvement in compile time)

#include "core/common/instr.h"
#include "trace/trace.h"

#include "lib/json/single_include/nlohmann/json.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

struct PathNode {
	Opcode::Mapping instruction;
	uint64_t weight;

	float score_bonus = 0.0;
	float score_multiplier = 1.0;
	float inverse_dependency_score = 0.0;

	//! How often this node ran at each pc. Taken from the node itself, so it is filled whatever
	//! the depth of the node on the path.
	std::map<uint64_t, int> program_counters;

	std::vector<int> true_dependencies;  //offset back to the node this one reads a register from
	std::set<int8_t> anti_dependencies;
	std::set<int8_t> output_dependencies;

	//! Whatever the node's payload adds, merged into `to_json`. Memory nodes put their access
	//! counts and peripherals here.
	nlohmann::json extra_fields = nlohmann::json::object();

	PathNode(Opcode::Mapping instr, uint64_t wt, float score_b, float score_m, float inv_d,
	         std::map<uint64_t, int> pcs, const std::array<bool, INSTRUCTION_TREE_DEPTH> &dep_true,
	         std::set<int8_t> dep_out, std::set<int8_t> dep_anti);

	nlohmann::json to_json() const;
};
