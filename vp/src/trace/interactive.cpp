#include "trace/analysis.h"
#include "trace/export.h"
#include "trace/report.h"

#include <chrono>
#include <cstdio>
#include <dlfcn.h>
#include <iostream>
#include <string>

namespace {

/**
 * Unload the scoring function library, if one is loaded.
 *
 * The order matters. Every `std::function` copied out of the library has to be cleared
 * first, because destroying one after `dlclose` runs a destructor that no longer exists.
 * The null check matters too: `LoadedLibrary::handle` is null when nothing was loaded, and
 * `dlclose(nullptr)` corrupts the caller's stack.
 */
void unload(LoadedLibrary &lib, std::array<ScoreFunction, SF_BATCH_SIZE> &copies) {
	if (!lib.handle) {
		return;
	}
	for (auto &f : copies) {
		f = nullptr;
	}
	for (auto &f : lib.functions) {
		f = nullptr;
	}
	dlclose(lib.handle);
	lib.handle = nullptr;
}

}  // namespace

void run_interactive(TraceReport &report) {
	printf("start score function analysis\n");

	LoadedLibrary sf_lib;
	std::array<ScoreFunction, SF_BATCH_SIZE> score_functions;

	while (true) {
		std::cout << "\nEnter \n"
		          << "\t'a' to run tree analysis\n"
		          << "\t'b' to run analysis performance benchmark\n"
		          << "\t'r' to reload the library\n"
		          << "\t'p [% threshold]' to prune trees\n"
		          << "\t'd' to export as dot\n"
		          << "\t'e' for a full export as json\n"
		          << "\t'q' to quit\n"
		          << ":" << std::endl;
		std::string userInput;
		if (!(std::cin >> userInput) || userInput.empty()) {
			break;  // standard input closed
		}
		char mode = userInput[0];

		if (mode == 'r') {
			unload(sf_lib, score_functions);
			sf_lib = load_scoring_functions(report.config.scoring_library);
			score_functions = sf_lib.functions;
		} else if (mode == 'a') {
			analyze_trees(score_functions, report.trees);
		} else if (mode == 'b') {
			using std::chrono::duration;
			using std::chrono::high_resolution_clock;

			auto before = high_resolution_clock::now();
			for (size_t i = 0; i < 100; i++) {
				analyze_trees(score_functions, report.trees);
			}
			duration<double, std::milli> elapsed = high_resolution_clock::now() - before;
			std::cout << "Analysis of 300  SF took " << elapsed.count() << "ms\n"
			          << elapsed.count() / 300.0 << "ms on average per function" << std::endl;
		} else if (mode == 'q') {
			break;
		} else if (mode == 'p') {
			float prune_threshold = PRUNE_THRESHOLD_WEIGHT;
			if (userInput.length() > 1) {
				try {
					prune_threshold = std::stof(userInput.substr(1)) / 100.0;
					std::cout << "Start pruning trees with threshold " << prune_threshold << std::endl;
				} catch (const std::invalid_argument &) {
					std::cout << "No valid threshold specified. Set to default ("
					          << PRUNE_THRESHOLD_WEIGHT << ")" << std::endl;
				}
			}
			for (auto &&tree : report.trees) {
				printf("\tPruning with absolute threshold: %f\n", tree.weight * prune_threshold);
				tree.prune_tree(tree.weight * prune_threshold, 0);
			}
		} else if (mode == 'd') {
			printf("exporting trees to dot\n");
			export_dot(report);
		} else if (mode == 'e') {
			printf("exporting trees to json\n");
			export_trees(report);
		} else {
			std::cout << "Invalid input." << std::endl;
		}
	}

	unload(sf_lib, score_functions);
}
