#include "trace/export.h"

#include <ctime>
#include <fstream>
#include <iostream>

// `InstructionNode::tree_to_dot` writes through `std::cout`, so writing a graph to a file
// means pointing `std::cout` at that file for the duration. See the note in export.h.

namespace {

//! Where a load or a store touched memory: address, then the pc that last wrote and last read.
void write_memory_map(const MemoryAccessMap *accesses) {
	if (!accesses) {
		return;
	}
	std::cout << std::hex;
	for (const auto &n : *accesses) {
		std::cout << n.first << " [" << std::get<0>(n.second) << " | " << std::get<1>(n.second) << "]"
		          << std::endl;
	}
	std::cout << std::dec;
}

void write_graph_header() {
	std::cout << "digraph g{" << std::endl;
	// shape = record
	std::cout << "node [shape = plaintext, style=\"bold\", height = .5, colorscheme=rdpu9];"
	          << std::endl;  // pubu9
}

}  // namespace

void export_dot(const TraceReport &report) {
	if (report.config.output_directory.empty()) {
		// No directory to write to, so the whole thing goes to standard output as one graph.
		write_graph_header();
		for (InstructionNode &tree : report.trees) {
			tree.tree_to_dot(report.retired_instructions, report.config.graph_branch_threshold);
		}
		std::cout << "}" << std::endl;
		write_memory_map(report.memory_accesses);
		return;
	}

	std::streambuf *cout_save = std::cout.rdbuf();
	std::cout << "writing dot files to directory " << report.config.output_directory << std::endl;

	// one dot file per opcode, named after the tree's root instruction
	for (InstructionNode &tree : report.trees) {
		std::ofstream output(report.config.output_directory +
		                     std::string(Opcode::mappingStr[tree.instruction]) + hart_suffix(report.config) +
		                     std::string(".dot"));
		output << "// " << std::time(0) << std::endl;
		std::cout.rdbuf(output.rdbuf());

		write_graph_header();
		// don't use colorscheme greys9 for now
		std::cout << "edge [style=\"solid\", arrowsize = .5];" << std::endl;
		tree.tree_to_dot(report.retired_instructions, report.config.graph_branch_threshold);
		std::cout << "}" << std::endl;
	}

	std::ofstream memory_map(report.config.output_directory + std::string("memory_map") + hart_suffix(report.config) +
	                          std::string(".md"));
	memory_map << "// " << std::time(0) << std::endl;
	std::cout.rdbuf(memory_map.rdbuf());
	write_memory_map(report.memory_accesses);

	std::cout.rdbuf(cout_save);
	std::cout << "restored cout" << std::endl;
}
