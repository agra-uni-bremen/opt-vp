#include "trace/analysis.h"
#include "trace/export.h"

#include <bitset>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>


void export_csv(const TraceReport &report) {
	if (report.config.output_directory.empty()) {
		std::cout << "[ERROR] No output directory set for csv export" << std::endl;
		return;
	}

	std::streambuf *cout_save = std::cout.rdbuf();
	std::string application_name = program_basename(report.config.input_program);

	std::cout << "writing csv files to directory " << report.config.output_directory << std::endl;
	std::string single_output_filename = report.config.output_directory + std::string("Execution_Trees_") +
	                                     application_name + hart_suffix(report.config) + std::string(".csv");
	std::ofstream output(single_output_filename);
	std::cout.rdbuf(output.rdbuf());

	std::cout << "id;"
	          << "parentId;"
	          << "tree;"
	          << "instruction;"
	          << "weight;"
	          << "weightDifference;"
	          << "weightDifferenceMax;"
	          << "weightDifferenceTotal;"
	          << "length;"
	          << "lengthDifference;"
	          << "cycles;"
	          << "dependencyScore;"
	          << "dependencyScoreTotal;"
	          << "dependenciesTrue;"
	          << "dependenciesAnti;"
	          << "dependenciesOut;"
	          << "dependenciesTrueTotal;"
	          << "dependenciesAntiTotal;"
	          << "dependenciesOutTotal;"
	          << "children;"
	          << "childrenDifference;"
	          << "inputs;"
	          << "inputsTotal;"
	          << "outputs;"
	          << "outputsTotal;"
	          << "instructionTypes;"
	          << "branches;"
	          << "occurrenceStart;"
	          << "occurrenceBeginning;"
	          << "occurrenceMid;"
	          << "occurrenceEnd;"
	          << "programCounters;"
	          << "programCountersDifference" << std::endl;

	std::map<InstructionType, uint32_t> instruction_types;
	uint64_t total_max_weight = 0;
	for (InstructionNodeR &tree : report.trees) {
		if (tree.weight > total_max_weight) {
			total_max_weight = tree.weight;
		}
	}
	for (InstructionNodeR &tree : report.trees) {
		tree.to_csv({
		    report.retired_instructions,
		    Opcode::mappingStr[tree.instruction],
		    1,  // depth
		    0, 0, 0, 0, 0, 0,
		    instruction_types,
		    0,
		    tree.weight,
		    tree.weight,  // last_weight
		    total_max_weight,
		    tree.get_pc().size(),
		});
	}

	std::cout.rdbuf(cout_save);
	std::cout << "restored cout" << std::endl;
}

void export_coverage_csv(const TraceReport &report, const std::vector<Path> &sequences,
                         const std::function<float(const ScoreParams)> &score_function) {
	if (!report.config.write_coverage_csv || report.config.coverage_csv_file.empty()) {
		return;
	}

	// A relative path is taken as relative to the output directory, an absolute one as-is.
	std::string output_path = report.config.coverage_csv_file;
	if (output_path[0] != '/' && !report.config.output_directory.empty()) {
		output_path = report.config.output_directory + output_path;
	}

	std::ofstream output(output_path);
	if (!output.is_open()) {
		std::cerr << "[ERROR] Could not open coverage csv output: " << output_path << std::endl;
		return;
	}

	output << "sequence;length;weight;true_weight;score;top_pc;top_pc_count;coverage\n";

	for (const auto &seq : sequences) {
		std::map<uint64_t, int> pc_counts;
		if (seq.end_of_sequence) {
			pc_counts = seq.end_of_sequence->get_pc();
		}

		uint64_t top_pc = 0;
		int top_pc_count = 0;
		for (const auto &entry : pc_counts) {
			if (entry.second > top_pc_count) {
				top_pc = entry.first;
				top_pc_count = entry.second;
			}
		}

		uint64_t true_weight = 0;
		if (seq.end_of_sequence) {
			true_weight = seq.end_of_sequence->true_weight;
		}
		double coverage = 0.0;
		if (report.retired_instructions > 0) {
			coverage = (static_cast<double>(seq.length) * static_cast<double>(true_weight)) /
			           static_cast<double>(report.retired_instructions);
		}

		std::ostringstream pc_stream;
		pc_stream << "0x" << std::hex << top_pc;

		output << "\"" << opcode_sequence_to_string(seq) << "\";" << seq.length << ";"
		       << seq.minimum_weight << ";" << true_weight << ";" << seq.get_score(score_function)
		       << ";" << pc_stream.str() << ";" << top_pc_count << ";" << coverage << "\n";
	}
}
