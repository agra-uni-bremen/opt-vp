#pragma once

// Architecture-independent helpers for analyzing/scoring discovered instruction sequences
// (used by both the rv32 and rv64 ISS implementations).

#include "trace.h"

#include <array>
#include <list>
#include <string>
#include <vector>

struct LoadedLibrary {
	void* handle;  // Handle to the loaded library
	std::array<ScoreFunction, SF_BATCH_SIZE> functions;
};

// Loads a shared library exposing a `score_functions` symbol of type
// std::array<ScoreFunction, SF_BATCH_SIZE> and returns it together with the library handle.
LoadedLibrary load_scoring_functions(const std::string& libraryPath);

// Evaluates every provided score function against every instruction tree and prints the best
// sequences found for each score function (used by the interactive analysis mode).
void analyze_trees(std::array<ScoreFunction, SF_BATCH_SIZE> score_functions, std::list<InstructionNodeR> instruction_trees);

double similarity_jaccard_positions(const std::vector<Opcode::Mapping>& a, const std::vector<Opcode::Mapping>& b);

size_t levenshtein_distance(const std::vector<Opcode::Mapping>& a, const std::vector<Opcode::Mapping>& b);

double normalized_levenshtein_similarity(const std::vector<Opcode::Mapping>& a, const std::vector<Opcode::Mapping>& b);

bool is_similar_sequence(const Path& candidate, const Path& existing, float threshold);

// Filters a score-sorted list of paths down to at most `top_n` entries, skipping candidates
// that are too similar (per `is_similar_sequence`/SIMILARITY_ALGORITHM) to an already-kept entry.
std::vector<Path> filter_top_sequences(const std::vector<Path>& sorted_by_score, size_t top_n, float threshold);

std::string opcode_sequence_to_string(const Path& seq);
