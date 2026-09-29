#include "trace/export.h"

#include <cstdio>
#include <fstream>
#include <iostream>

// Both exporters build a complete JSON value and then write it, so they use their own stream
// and leave `std::cout` alone.

void export_sequences(const TraceReport &report,
                      const std::vector<std::vector<PathNode>> &sequences,
                      const std::vector<std::vector<std::vector<PathNode>>> &sub_sequences,
                      const std::vector<std::vector<std::vector<PathNode>>> &variants) {
	nlohmann::json top_level_json;
	int idx = 0;
	printf("Converting Full Paths to JSON\n");
	for (auto &&seq : sequences) {
		printf(".");
		nlohmann::json path_node_json_array = nlohmann::json::array();
		for (auto &&node : seq) {
			path_node_json_array.push_back(node.to_json());
		}
		std::string key = "Sequence" + std::to_string(idx);
		top_level_json[key] = path_node_json_array;

		int sub_idx = 0;
		for (auto &&sub_seq : sub_sequences.at(idx)) {
			nlohmann::json sub_sequence_json_array = nlohmann::json::array();
			for (auto &&node : sub_seq) {
				sub_sequence_json_array.push_back(node.to_json());
			}
			top_level_json[key + "-" + std::to_string(sub_idx)] = sub_sequence_json_array;
			sub_idx++;
		}

		int variant_idx = 0;
		for (auto &&variant_seq : variants.at(idx)) {
			nlohmann::json variant_sequence_json_array = nlohmann::json::array();
			for (auto &&node : variant_seq) {
				variant_sequence_json_array.push_back(node.to_json());
			}
			top_level_json[key + "v" + std::to_string(variant_idx)] = variant_sequence_json_array;
			variant_idx++;
		}
		idx++;
	}
	printf("\n");

	if (report.config.output_directory.empty()) {
		std::cout << top_level_json.dump(JSON_INDENT) << std::endl;
		return;
	}

	std::string single_output_filename = report.config.output_directory + std::string("sequences_") +
	                                     program_basename(report.config.input_program) +
	                                     hart_suffix(report.config) + std::string(".json");
	std::cout << "writing json to directory " << report.config.output_directory << std::endl;
	std::cout << "Sequence file: " << single_output_filename << std::endl;

	std::ofstream output(single_output_filename);
	output << top_level_json.dump(JSON_INDENT) << std::endl;
}

void export_trees(const TraceReport &report) {
	std::cout << "writing json to directory " << report.config.output_directory << std::endl;

	std::string application_name = program_basename(report.config.input_program);
	for (InstructionNode &tree : report.trees) {
		std::string single_output_filename = report.config.output_directory + application_name +
		                                     std::string(Opcode::mappingStr[tree.instruction]) +
		                                     hart_suffix(report.config) + std::string(".json");

		nlohmann::ordered_json tree_json = nlohmann::ordered_json::object();
		// the format version goes first so a reader can dispatch on it before parsing the tree
		tree_json["format_version"] = TRACE_FORMAT_VERSION;
		tree_json.update(tree.to_json());

		std::ofstream output(single_output_filename);
		output << tree_json.dump(JSON_INDENT) << std::endl;
		printf(".");
	}
	printf("Done\n");
}
