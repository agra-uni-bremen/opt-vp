#include <iostream>
#include <array>
#include <functional>

#include "../core/common/trace.h"

// struct ScoreParams {
// 	Opcode::Mapping instr; 
// 	Opcode::Mapping tree; 
// 	uint64_t weight; 
// 	uint32_t length; 
// 	float dep_score;
// 	int32_t num_children;
// 	int32_t inputs;
// 	int32_t outputs; 
// 	float score_multiplier; 
// 	float score_bonus; 
// // uint32_t num_pcs;
// };

//TODO 
//define scoring function as sum 

	std::array<std::function<float(ScoreParams)>, 3> score_functions = {
        [](ScoreParams p) -> float {
			// printf("test\n\n\n\n");
			float score = ((p.length * p.weight) * p.score_multiplier 
				+ p.weight * p.score_bonus); //length * minimum_weight;
			return score;
		},
        [](ScoreParams p) -> float {
			float score = ((p.length * p.weight) * p.score_multiplier 
				+ p.weight * p.score_bonus) / (1 + p.dep_score); 
			return score;
		},
        [](ScoreParams p) -> float {
			float score = ((p.length * p.weight) * p.score_multiplier 
				+ p.weight * p.score_bonus) * (p.num_children + 1);
			return score;
		}
    };