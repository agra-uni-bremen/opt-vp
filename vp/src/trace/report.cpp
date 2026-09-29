#include "trace/report.h"

#include "trace/analysis.h"
#include "trace/export.h"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <vector>

namespace {

//! The score the report ranks sequences by: how many instructions the sequence covers.
float default_score(ScoreParams p) {
	// TODO update to use true weight instead
	double score = static_cast<double>(p.length) * static_cast<double>(p.weight);
	return static_cast<float>(score);
}

//! Find the tree a path starts in. Every path came out of a tree, so this always succeeds.
InstructionNode *find_tree(std::list<InstructionNode> &trees, Opcode::Mapping root) {
	for (InstructionNode &tree : trees) {
		if (tree.instruction == root) {
			return &tree;
		}
	}
	return nullptr;
}

std::vector<Path> sorted_by_score_descending(const std::vector<Path> &sequences,
                                             const ScoreFunction &score) {
	std::vector<Path> sorted = sequences;
	std::sort(sorted.begin(), sorted.end(),
	          [&score](const Path &a, const Path &b) { return a.get_score(score) > b.get_score(score); });
	return sorted;
}

//! Every opcode the program never executed, which is every opcode with no tree.
size_t count_unused_instructions(std::list<InstructionNode> &trees) {
	size_t unused = 0;
	for (size_t i = Opcode::Mapping::ADD; i < Opcode::Mapping::NUMBER_OF_INSTRUCTIONS; i++) {
		Opcode::Mapping op = static_cast<Opcode::Mapping>(i);
		if (find_tree(trees, op) == nullptr) {
			unused++;
		}
	}
	return unused;
}

/**
 * Walk every discovered sequence, turn it into path nodes, and when the sequence export is
 * on also collect its sub sequences and its branch variants.
 *
 * The sub sequence and variant work is the expensive part and runs only for the sequence
 * export, which is what `report.config.write_sequences` guards.
 */
void collect_sequence_nodes(TraceReport &report, const std::vector<Path> &sequences,
                            const ScoreFunction &score,
                            std::vector<std::vector<PathNode>> &out_sequences,
                            std::vector<std::vector<std::vector<PathNode>>> &out_sub_sequences,
                            std::vector<std::vector<std::vector<PathNode>>> &out_variants) {
	printf("\n -----------------\n| Best Sequences |\n -----------------\n");

	for (const Path &p : sequences) {
		InstructionNode *found_tree = find_tree(report.trees, p.opcodes[0]);
		if (found_tree == nullptr) {
			// This should never happen: a path exists only because a tree held it.
			printf("[ERROR] Could not find matching tree for discovered path\nOpcode: %s\n",
			       Opcode::mappingStr[p.opcodes[0]]);
			continue;
		}

		// path_to_path_nodes and find_variant_branch both walk the tree, so keep this order.
		std::vector<PathNode> full_path = found_tree->path_to_path_nodes(p, 0);

		std::vector<BranchingPoint> variant_starting_points;
		if (report.config.write_sequences) {
			variant_starting_points = found_tree->find_variant_branch(p, 0);
			// sort variant starting points, so we can extend the top N
			std::sort(variant_starting_points.begin(), variant_starting_points.end(),
			          [](BranchingPoint a, BranchingPoint b) { return a.ratio > b.ratio; });

			#ifdef log_variants
			for (auto &&v : variant_starting_points) {
				printf("Variant Branching Point: %s at %d, with ratio %.4f\n",
				       Opcode::mappingStr[v.instruction], v.depth, v.ratio);
			}
			#endif
		}

		out_sequences.push_back(full_path);

		if (!report.config.write_sequences) {
			continue;
		}

		// force extend to the top N variant starting points
		std::vector<Path> variants;
		size_t max_variants = std::min<size_t>(MAX_VARIANTS, variant_starting_points.size());
		for (size_t i = 0; i < max_variants; i++) {
			// convert to Path first (up to new branching point and then force extension)
			variants.push_back(found_tree->extend_path({1, 0, 1.0, static_cast<int>(i),
			                                            variant_starting_points[i].depth,
			                                            variant_starting_points[i].instruction, score}));
		}
		printf("-------------------------------------------\n");

		std::vector<Path> sub_sequences = p.end_of_sequence->force_path_extension(p, score);
		#ifdef log_variants
		printf("-------------------------------------------\n");
		printf("Sub Sequences (%zu)\n[\n", sub_sequences.size());
		#endif

		std::vector<std::vector<PathNode>> sub_sequence_nodes;
		for (const Path &subseq : sub_sequences) {
			#ifdef log_variants
			printf(" - Sub Sequence:\n");
			subseq.show(" - ");
			printf(" - ,\n");
			#endif
			sub_sequence_nodes.push_back(found_tree->path_to_path_nodes(subseq, 0));
		}
		out_sub_sequences.push_back(sub_sequence_nodes);

		std::vector<std::vector<PathNode>> variant_nodes;
		for (const Path &variant : variants) {
			#ifdef log_variants
			printf(" - Variant Sequence:\n");
			variant.show(" - ");
			printf(" - ,\n");
			#endif
			variant_nodes.push_back(found_tree->path_to_path_nodes(variant, 0));
		}
		out_variants.push_back(variant_nodes);
		#ifdef log_variants
		printf("]\n-------------------------------------discovered_sequences_node_list------\n");
		#endif
	}
}

}  // namespace

std::string program_basename(const std::string &path) {
	return path.substr(path.find_last_of("/\\") + 1);
}

std::string hart_suffix(const TraceConfig &config) {
	return config.hart_id == 0 ? std::string() : "-hart" + std::to_string(config.hart_id);
}

void run_trace_report(TraceReport &report) {
	std::cout << "execution statistics: (" << report.trees.size() << " Trees)" << std::endl;

	if (report.config.write_dot) {
		export_dot(report);
	}
	if (report.config.write_csv) {
		export_csv(report);
	}
	if (report.config.write_trees) {
		export_trees(report);
	}

	// find the best instruction sequences for each tree
	ScoreFunction score = default_score;
	std::vector<Path> discovered_sequences;
	printf("start analysis\n");
	int tree_index = 0;
	for (InstructionNode &tree : report.trees) {
		std::vector<Path> top_paths = tree.extend_top_paths(
		    {1, 0, 1.0, tree_index, -1, Opcode::Mapping::UNDEF, score}, report.config.coverage_top_n);
		discovered_sequences.insert(discovered_sequences.end(), top_paths.begin(), top_paths.end());
		tree_index++;
		printf(".");
	}
	printf("-> analyzed all trees\n");

	// sort ascending, so the most relevant sequence is the last one printed and is the one
	// the summary at the end reports
	std::sort(discovered_sequences.begin(), discovered_sequences.end(),
	          [&score](const Path &a, const Path &b) { return a.get_score(score) < b.get_score(score); });

	{
		std::vector<Path> sequences_sorted = sorted_by_score_descending(discovered_sequences, score);
		printf("\n ----------------------------\n| Best Sequences (Coverage) |\n ----------------------------\n");
		for (size_t idx = 0; idx < std::min<size_t>(4, sequences_sorted.size()); ++idx) {
			sequences_sorted[idx].show();
		}
	}

	if (report.config.write_coverage_csv) {
		std::vector<Path> sequences_sorted = sorted_by_score_descending(discovered_sequences, score);
		std::vector<Path> filtered = filter_top_sequences(sequences_sorted,
		                                                  static_cast<size_t>(report.config.coverage_top_n),
		                                                  report.config.coverage_similarity_threshold);
		export_coverage_csv(report, filtered, score);
	}

	std::vector<std::vector<PathNode>> sequence_nodes;
	std::vector<std::vector<std::vector<PathNode>>> sub_sequence_nodes;
	std::vector<std::vector<std::vector<PathNode>>> variant_nodes;
	collect_sequence_nodes(report, discovered_sequences, score, sequence_nodes, sub_sequence_nodes,
	                       variant_nodes);

	if (report.config.write_sequences) {
		export_sequences(report, sequence_nodes, sub_sequence_nodes, variant_nodes);
	}

	float total_percent = 1.0;
	if (discovered_sequences.empty()) {
		printf("[Warning] Ringbuffer was not filled at least once. \nThis means, the whole program "
		       "fits into one sequence. \nYou probably want to execute a longer program or decrease "
		       "the tree bound.\n");
	} else {
		const Path &best = discovered_sequences.back();
		total_percent = static_cast<float>(best.minimum_weight) * static_cast<float>(best.length) /
		                static_cast<float>(report.retired_instructions);
		std::cout << "inverse dependency score " << best.inverse_dependency_score << std::endl;
		std::cout << "partially normalized potential " << best.get_normalized_score() << std::endl;
		std::cout << "normalized potential " << best.get_normalized_score() * total_percent * 100
		          << std::endl;

		size_t unused = count_unused_instructions(report.trees);
		std::cout << "\n[Unused Instructions]" << std::endl;
		if (unused > 0) {
			std::cout << unused << std::endl;
		} else {
			std::cout << "- NONE -" << std::endl;
		}

		if (report.config.interactive) {
			run_interactive(report);
		}
	}

	std::cout << "total instructions: " << report.retired_instructions << "("
	          << report.stepped_instructions << ")"
	          << " [" << total_percent * 100 << "]" << std::endl;
	std::cout << "total cycles: " << report.cycles << std::endl;

	if (discovered_sequences.empty()) {
		std::cout << "Best sequence: "
		          << "NONE (the whole program fits into one sequence)"
		          << "\nLength: " << report.retired_instructions << "\nWeight: " << 1
		          << "\n%:      [" << total_percent * 100 << "]"
		          << "\nTotal:  " << report.retired_instructions << "\nNP:     1.0" << std::endl;
	} else {
		const Path &best = discovered_sequences.back();
		std::cout << "Best sequence: " << Opcode::mappingStr[best.opcodes[0]]
		          << "\nLength: " << best.length << "\nWeight: " << best.minimum_weight
		          << "\n%:      [" << total_percent * 100 << "]"
		          << "\nTotal:  " << report.retired_instructions
		          << "\nNP:     " << best.get_normalized_score() << std::endl;
	}
}
