#include "trace_analysis.h"

#include <algorithm>
#include <cstdio>
#include <dlfcn.h>
#include <iostream>
#include <sstream>

LoadedLibrary load_scoring_functions(const std::string& libraryPath){
	void* library = dlmopen(-1, libraryPath.c_str(), RTLD_NOW); // | RTLD_GLOBAL
	std::cout << "Loading library from: " << libraryPath << std::endl;

	std::array<ScoreFunction, SF_BATCH_SIZE>* score_functions_ptr; 
	if (!library) {
		std::cerr << "Error loading the library: " << dlerror() << std::endl;
		//exit(EXIT_FAILURE);
		return {NULL, nullptr};
	}

	score_functions_ptr = 
		(std::array<ScoreFunction, SF_BATCH_SIZE>*)dlsym(library, "score_functions");

	if (!score_functions_ptr) {
		std::cerr << "Error getting score_functions pointer: " << dlerror() << std::endl;
		dlclose(library);
		//exit(EXIT_FAILURE);
		return {library, nullptr};
	}
	return {library, *score_functions_ptr};
}

void analyze_trees(std::array<ScoreFunction, SF_BATCH_SIZE> score_functions, std::list<InstructionNodeR> instruction_trees){
	std::vector<std::vector<Path>> best_sequences_for_sf; 
	uint32_t score_function_index = 0;
	for ([[maybe_unused]] auto &&score_function : score_functions)
	{
		std::vector<Path> tmp_best_sequences; 
		auto _sf = score_functions[score_function_index];
		for(InstructionNodeR& tree : instruction_trees){
			Path p = tree.extend_path(
				{1, 0, 1.0, 0, -1, Opcode::Mapping::UNDEF, _sf});
			tmp_best_sequences.push_back(p);
		}
		best_sequences_for_sf.push_back(tmp_best_sequences);
		std::sort(best_sequences_for_sf[score_function_index].begin(), best_sequences_for_sf[score_function_index].end(), 
		[_sf](Path a, Path b) -> bool{
			return a.get_score(_sf)>b.get_score(_sf);
		});
		score_function_index++;
	}
	printf("finished score function analysis\n");
	
	int sf_index = 0;
	for (auto &&sequences : best_sequences_for_sf)
	{
		printf("\nBest sequences for score function %d:\n", sf_index);
		for (size_t i = 0; i < std::min(static_cast<size_t>(3), sequences.size()); i++) //print the best 3 sequences for each score function
		{
			sequences[i].show();
		}
		sf_index++;
	}
}

double similarity_jaccard_positions(const std::vector<Opcode::Mapping>& a,
				const std::vector<Opcode::Mapping>& b) {
	const size_t max_len = std::max(a.size(), b.size());
	if (max_len == 0) {
		return 1.0;
	}
	const size_t min_len = std::min(a.size(), b.size());
	size_t matches = 0;
	for (size_t i = 0; i < min_len; ++i) {
		if (a[i] == b[i]) {
			++matches;
		}
	}
	return static_cast<double>(matches) / static_cast<double>(max_len);
}

size_t levenshtein_distance(const std::vector<Opcode::Mapping>& a,
				const std::vector<Opcode::Mapping>& b) {
	const size_t n = a.size();
	const size_t m = b.size();
	std::vector<size_t> prev(m + 1, 0);
	std::vector<size_t> curr(m + 1, 0);
	for (size_t j = 0; j <= m; ++j) {
		prev[j] = j;
	}
	for (size_t i = 1; i <= n; ++i) {
		curr[0] = i;
		for (size_t j = 1; j <= m; ++j) {
			const size_t cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
			curr[j] = std::min({prev[j] + 1, curr[j - 1] + 1, prev[j - 1] + cost});
		}
		std::swap(prev, curr);
	}
	return prev[m];
}

double normalized_levenshtein_similarity(const std::vector<Opcode::Mapping>& a,
				const std::vector<Opcode::Mapping>& b) {
	const size_t max_len = std::max(a.size(), b.size());
	if (max_len == 0) {
		return 1.0;
	}
	const size_t dist = levenshtein_distance(a, b);
	return 1.0 - (static_cast<double>(dist) / static_cast<double>(max_len));
}

bool is_similar_sequence(const Path& candidate, const Path& existing, float threshold) {
	if (threshold <= 0.0f) {
		return false;
	}
	double similarity = 0.0;
	#if SIMILARITY_ALGORITHM == 1
	similarity = similarity_jaccard_positions(candidate.opcodes, existing.opcodes);
	#endif
	#if SIMILARITY_ALGORITHM == 2
	similarity = normalized_levenshtein_similarity(candidate.opcodes, existing.opcodes);
	#endif
	return (similarity >= threshold);
}

std::vector<Path> filter_top_sequences(const std::vector<Path>& sorted_by_score,
				size_t top_n, float threshold) {
	std::vector<Path> filtered;
	if (top_n == 0) {
		return filtered;
	}
	for (const auto& seq : sorted_by_score) {
		bool too_similar = false;
		for (const auto& kept : filtered) {
			if (is_similar_sequence(seq, kept, threshold)) {
				too_similar = true;
				break;
			}
		}
		if (!too_similar) {
			filtered.push_back(seq);
			if (filtered.size() >= top_n) {
				break;
			}
		}
	}
	return filtered;
}

std::string opcode_sequence_to_string(const Path& seq) {
	std::ostringstream stream;
	for (const auto opcode : seq.opcodes) {
		const char* opcode_string = "UNKWN";
		if (opcode < Opcode::mappingStr.size()) {
			opcode_string = Opcode::mappingStr[opcode];
		}
		stream << opcode_string << " -> ";
	}
	return stream.str();
}
