#pragma once

// The interface between the analysis and a scoring function.
//
// A scoring function ranks a candidate sequence: the analysis offers it what it measured about
// the sequence and gets one number back, and the highest number wins. The default function is
// in `vp/src/trace/report.cpp`.
//
// This header is also the whole interface of a scoring library loaded with
// `--scoring-library` (see `vp/src/scoring_functions/`), which is why it is separate from
// `trace.h`: a plugin must agree with the VP about this struct and needs nothing else.

#include "core/common/instr.h"

#include <functional>

//! How many scoring functions a library exports, and the analysis evaluates.
#define SF_BATCH_SIZE 3

//! What the analysis measured about one candidate sequence.
struct ScoreParams {
	Opcode::Mapping instr;   //!< last instruction of the sequence
	Opcode::Mapping tree;    //!< first instruction, which is the root of its tree
	uint64_t weight;         //!< how often the sequence ran
	// uint64_t true_weight;
	uint32_t length;         //!< instructions in the sequence
	double dep_score;        //!< sum of 1/distance over the dependencies inside the sequence
	uint32_t num_children;
	uint32_t inputs;
	uint32_t outputs;
	float score_multiplier;  //!< product over the sequence, zero once it contains a branch
	float score_bonus;       //!< sum over the sequence, negative for a branch or a jump
	// uint32_t num_pcs;
};

using ScoreFunction = std::function<float(ScoreParams)>;
