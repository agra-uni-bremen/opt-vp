#pragma once

#include "instr.h"
#include <set>
#include <unordered_set>
#include <unordered_map>

#include <bitset>
#include <fstream>

#include "lib/json/single_include/nlohmann/json.hpp"

//maximum tree depth. Fixed at compile time so nodes can keep static arrays/bitsets.
//override from the build system with -DINSTRUCTION_TREE_DEPTH=<n>
#ifndef INSTRUCTION_TREE_DEPTH
#define INSTRUCTION_TREE_DEPTH 20
#endif
//version of the exported trace format, written into every exported tree.
//bump the minor version when fields are added, the major version when existing fields change meaning
//1.2: predecessor pcs per register_sets entry, immediates for every instruction that carries one,
//     real branch directions/offsets and per pc taken/not-taken counts
#define TRACE_FORMAT_VERSION "1.2"

#define JSON_INDENT -1
#define SIMILARITY_ALGORITHM 1 //1 for jaccard, 2 for levenshtein
#define MAX_VARIANTS 3
#define SF_BATCH_SIZE 3
#define PRUNE_THRESHOLD_WEIGHT 0.01 //threshold weight ratio for pruning branches 
//no pruning md5 d50 -> 0.356s per function for all trees
//threshold of 0.03 d50 -> 0.0233s
//threshold of 0.01 d50 -> 0.0533s 

#define O_STARTUP 1000
#define O_BEGINNING 10000
#define O_MID 3000000 //TODO use input instead
//#define O_END

#define trace_pcs
#define log_pcs
//#define output_stats //output a summary of the VP analysis 
// #define debug_register_dependencies
//#define debug_dependencies
// #define handle_self_modifying_code
#define trace_individual_registers
#define trace_parameter //trace instruction parameters like shift amount and branch/jump targets

//trace the decoded immediate of every instruction that carries one (ADDI, ANDI, LUI, load/store offsets, ...)
//this is what allows constant folding of immediates on the analysis side, but it adds one parameter entry
//per pc for most of the program -> noticeably larger traces. Disable with -DNO_TRACE_PARAMETER_IMMEDIATES
#ifndef NO_TRACE_PARAMETER_IMMEDIATES
#define trace_parameter_immediates
#endif

#if defined(trace_parameter) && !defined(trace_individual_registers)
#error "trace_parameter stores its values in the per pc register set entries, it needs trace_individual_registers"
#endif

//record which pc preceded each register_sets entry in the same dynamic execution.
//needed to prove (instead of guess) the pc path through a sequence. Disable with -DNO_TRACE_PREDECESSOR_PCS
#ifndef NO_TRACE_PREDECESSOR_PCS
#define trace_predecessor_pcs
#endif

//record the real taken/not-taken counts and branch offsets per pc for branch/jump nodes
//disable with -DNO_TRACE_BRANCH_OUTCOMES
#ifndef NO_TRACE_BRANCH_OUTCOMES
#define trace_branch_outcomes
#endif

//also track parameters on the root node of a tree. Costs one entry per pc executing that opcode
//(the root of a tree is reached by every occurrence of its instruction), so it is off by default.
//enable with -DTRACE_ROOT_PARAMETERS
//temporarily hardcode this as ON
// #ifdef TRACE_ROOT_PARAMETERS
#define trace_root_parameters
// #endif

//#define dot_pc_on_pruned_nodes

//effective tree depth used at runtime, always <= INSTRUCTION_TREE_DEPTH (see --trace-depth).
//node storage is sized by INSTRUCTION_TREE_DEPTH, only the first trace_depth entries are used.
extern uint32_t trace_depth;
//clamps to [2, INSTRUCTION_TREE_DEPTH] and warns if the requested depth is out of range.
//a depth of 0 keeps the compiled default. Must be called before the first instruction is traced.
void set_trace_depth(uint32_t depth);

// extern std::array<const char*, NUMBER_OF_INSTRUCTIONS> mappingStr;

// Opcode::Type getType(Opcode::Mapping mapping);

enum class InstructionType {
	UNKNOWN = 0,
	Arithmetic,
	Logic,
	Load_Store,
	Branch,
	LUI,
	Jump,
	Float_Compare,
	Float_R4,
};

enum class NODE_TYPE {
	BASE = 0,
	NODE = 1, 
	LEAF = 2, 
	BRANCH = 4, 
	MEMORY = 8, 
	BRANCH_R = NODE | BRANCH, 
	BRANCH_L = LEAF | BRANCH, 
	MEMORY_R = NODE | MEMORY, 
	MEMORY_L = LEAF | MEMORY, 
};

enum class AccessType {
	NONE=0,
	LOAD=1,
	STORE=2
};

enum class BranchOutcome : uint8_t {
	NONE=0, //not a branch/jump
	TAKEN=1,
	NOT_TAKEN=2
};

InstructionType getInstructionType(Opcode::Mapping mapping);

//sentinel for "this instruction did not produce a traced parameter"
//parameters are signed (negative immediates, branch targets above 2 GiB), so 0/-1 can not be used
constexpr int64_t NO_PARAMETER = INT64_MIN;

//decoded immediate of an instruction, or NO_PARAMETER if it does not carry one.
//branches and jumps are excluded on purpose: their parameter slot holds the actual target pc
//(see the ISS), and the encoded offset is reported through the branch outcome instead.
inline int64_t decode_traced_immediate(Opcode::Mapping op, Instruction& instr) {
	using namespace Opcode;
	switch (op) {
		//shift amounts live in the immediate field
		case SLLI:
		case SRLI:
		case SRAI:
			return instr.shamt();
		case SLLIW:
		case SRLIW:
		case SRAIW:
			return instr.shamt_w();
		//target pc is traced instead of the encoded offset
		case JAL:
		case JALR:
			return NO_PARAMETER;
		default:
			break;
	}
	switch (getType(op)) {
		case Type::I:
			return instr.I_imm();
		case Type::S:
			return instr.S_imm();
		case Type::U:
			return instr.U_imm();
		default:
			//R/R4 have no immediate, B is covered by the branch target/outcome,
			//UNKNOWN covers CSR, FENCE, ECALL, ... where the field is not an immediate
			return NO_PARAMETER;
	}
}

//one entry of the ring buffer of recently executed instructions.
//every member is initialized: the buffer is only partially filled during the first steps and the
//code detects that by checking for a zero opcode
struct ExecutionInfo {
	Opcode::Mapping last_executed_instruction = Opcode::UNDEF;
	uint64_t last_cycles = 0;
	uint8_t last_powermode = 0;

	std::tuple<uint16_t,uint16_t,uint16_t> last_registers = {0,0,0};
	uint64_t last_executed_pc = 0;
	//pc of the instruction executed directly before this one (0 if unknown, e.g. first step)
	uint64_t last_predecessor_pc = 0;

	uint64_t last_memory_read = 0;
	uint64_t last_memory_written = 0;
	AccessType last_memory_access_type = AccessType::NONE;
	uint64_t last_stack_pointer = 0;
	uint64_t last_frame_pointer = 0;
	const char* last_peripheral_name = nullptr;

	int64_t last_parameter = NO_PARAMETER;
	//actual outcome of a branch/jump and its encoded offset (I_imm for JALR, as its target is dynamic)
	BranchOutcome last_branch_outcome = BranchOutcome::NONE;
	int64_t last_branch_offset = 0;

	uint64_t last_step_id = 0;
};

struct StepInsertInfo {
    Opcode::Mapping op;
    uint64_t pc;
    int8_t true_dependency1;
    int8_t true_dependency2;
    std::bitset<INSTRUCTION_TREE_DEPTH> output_dependencies;
    std::bitset<INSTRUCTION_TREE_DEPTH> anti_dependencies;
	uint8_t rs1;
	uint8_t rs2;
	uint8_t rd;
	//int8_t rs3; //add for fused multiply instructions
    int8_t input1; //equal to rs1 if rs1 was not written to by another instruction in the sequence, -1 otherwise
    int8_t input2; 
    int8_t output;
    uint32_t depth;
    uint64_t step;
	uint64_t cycles;
	uint64_t memory_address;
	AccessType access_type;
	uint64_t stack_pointer;
	uint64_t frame_pointer;
	// Parameter tracking fields
	int64_t parameter; // shift amount, branch/jump target or decoded immediate (NO_PARAMETER if none)
	const char* peripheral_name = nullptr;
	uint64_t predecessor_pc;
	BranchOutcome branch_outcome;
	int64_t branch_offset;
};

struct ScoreParams {
	Opcode::Mapping instr; 
	Opcode::Mapping tree; 
	uint64_t weight; 
	uint32_t length; 
	double dep_score;
	uint32_t num_children;
	uint32_t inputs;
	uint32_t outputs; 
	float score_multiplier; 
	float score_bonus; 
// uint32_t num_pcs;
};

using ScoreFunction = std::function<float(ScoreParams)>;

struct StepUpdateInfo {
    int8_t dependency1;
    int8_t dependency2;
    std::bitset<INSTRUCTION_TREE_DEPTH> output_dependencies;
    std::bitset<INSTRUCTION_TREE_DEPTH> anti_dependencies;
	uint8_t rs1;
	uint8_t rs2;
	uint8_t rd;
    int8_t input1;
    int8_t input2;
	int8_t output;
    uint64_t pc;
    uint64_t step;
	uint64_t cycles;
	uint64_t memory_address;
	AccessType access_type;
	uint64_t stack_pointer;
	uint64_t frame_pointer;
	int64_t parameter; // shift amount, branch/jump target or decoded immediate (NO_PARAMETER if none)
	const char* peripheral_name = nullptr;
	uint64_t predecessor_pc;
	BranchOutcome branch_outcome;
	int64_t branch_offset;
};

class InstructionNode;

struct Path
{
	uint32_t length = 0;
	uint64_t minimum_weight = 0;
	float score_bonus = 0.0;
	float score_multiplier = 1.0;

	double inverse_dependency_score = 0.0;
	std::vector<uint64_t> path_hashes;
	std::vector<Opcode::Mapping> opcodes;
	InstructionNode* end_of_sequence;
	
	//double score = 0; //TODO should this be saved here? How should we handle initialization?

	float get_score(std::function <float(ScoreParams)> score_function) const {
		uint32_t num_children = 0; //TODO end_of_sequence test for type and count children  
		uint32_t inputs = 0; //TODO
		uint32_t outputs = 0; //TODO 
		ScoreParams params = {opcodes.back(), opcodes[0], 
		minimum_weight, length, inverse_dependency_score, 
		num_children, inputs, outputs, score_multiplier, score_bonus};
		float score_result = score_function(params);
		if(score_result < 0){
			printf("[ERROR] Score function returned negative score: %f\nweight: %lu\nlength: %u\n", score_result, minimum_weight, length);
			score_result = INFINITY;
		}
		return score_result;
		// length * minimum_weight * score_multiplier 
		// 		+ minimum_weight * score_bonus; //length * minimum_weight;
	}
	float get_normalized_score() const {
		return length / (1.0 + inverse_dependency_score);
	}

	void show() const {
		show("");
	}
	void show(const char* prefix) const {
		auto sf = [](const ScoreParams p) {
			float score = (p.length * p.weight) * p.score_multiplier 
				+ p.weight * p.score_bonus; //length * minimum_weight;
			return score;
		};
		show(prefix, sf);
	}

	void show(const char* prefix, std::function <float(const ScoreParams)> score_function) const {
		std::cout << prefix << "[Sequence]\n";
		std::cout << prefix << "Length: " << length << "\n";
		std::cout << prefix << "Weight: " << minimum_weight << "\n";
		std::cout << prefix << "Score:  " << get_score(score_function) << "\n";

		std::cout << prefix << "<Opcodes>\n" << prefix;
		for (auto &&opcode : opcodes)
		{
			const char* opcode_string = "UNKWN ";
			if(opcode < Opcode::mappingStr.size()){
				opcode_string = Opcode::mappingStr[opcode];
			}
			std::cout << opcode_string << " --> ";

		}

		std::cout << std::endl;


		std::cout << prefix << "Last Path Hash: " << path_hashes.back() << std::endl;
	}

	void to_csv_stats(std::ostream& output_file, uint64_t total_instructions) const;
};


//used to represent a node in an identified path/sequence
//used for exporting identified sequences 
struct PathNode {
    Opcode::Mapping instruction;
    uint64_t weight;

	//uint64_t cycles;
    //uint64_t subtree_hash;
	float score_bonus = 0.0;
	float score_multiplier = 1.0;
	float inverse_dependency_score = 0.0;

	//usually only for leaf nodes, but we can calculate this from following all branches from the current node
	std::map<uint64_t, int> program_counters;

	std::vector<int> true_dependencies; //offset to previous node this node has a true dependency to
	std::set<int8_t> anti_dependencies;
	std::set<int8_t> output_dependencies;

	//additional per-node info attached by specialized node types (e.g. memory/peripheral access info), merged into to_json() output
	nlohmann::json extra_fields = nlohmann::json::object();

	//also save registers?

    // Constructor to initialize from an InstructionNode
    PathNode(Opcode::Mapping instr, uint64_t wt, float score_b, float score_m, float inv_d, 
				std::map<uint64_t, int> pcs, std::array<bool, 
				INSTRUCTION_TREE_DEPTH> dep_true, std::set<int8_t> dep_out, std::set<int8_t> dep_anti);

	nlohmann::json to_json();
};

struct CsvParams {
    uint64_t total_instructions;
    const char* tree;
    uint32_t depth;
    double last_dep_score;
    uint32_t true_dep;
    uint32_t anti_dep;
    uint32_t out_dep;
    std::bitset<32> total_inputs;
	std::bitset<32> total_outputs;
    std::map<InstructionType, uint32_t> instruction_types;
    uint64_t parent_hash;
    uint64_t max_weight;
	uint64_t last_weight;
    uint64_t total_max_weight;
    uint64_t max_pcs;
};

struct PathExtensionParams {
    uint32_t length;
    float score_bonus;
    float score_multiplier;
    int32_t tree_id;
    int32_t force_extension_depth;
    Opcode::Mapping force_instruction;
	std::function <float(const ScoreParams)> score_function;
};

struct BranchingPoint
{
	uint8_t depth = 0;
	Opcode::Mapping instruction = Opcode::UNDEF;
	int64_t weight = 0;
	double ratio = 0.0;
	InstructionNode* starting_point;
};

struct RegisterSet
{
	int8_t rs1 = -1;
	int8_t rs2 = -1;
	int8_t rd = -1;

	RegisterSet(int8_t rs1, int8_t rs2, int8_t rd)
        : rs1(rs1), rs2(rs2), rd(rd) {}
};
#ifdef trace_parameter
//one traced parameter value of a pc and how often it was seen with that value
struct ParameterCounter {
	int64_t value;
	uint64_t count;
};
#endif

struct RegisterSetCounter {
	int count;
	RegisterSet regset;
	#ifdef trace_predecessor_pcs
	//pc this entry was reached from -> how often. A node usually has one or two distinct predecessors
	std::map<uint64_t, uint64_t> predecessors;
	#endif
	#ifdef trace_parameter
	//parameter values seen at this pc. Almost always a single entry (a constant immediate), so a
	//linear scan is cheaper than hashing, and it shares the lookup of the register set entry
	std::vector<ParameterCounter> parameters;
	#endif
	RegisterSetCounter(int8_t rs1, int8_t rs2, int8_t rd)
		: count(1), regset(rs1, rs2, rd) {}

	#ifdef trace_parameter
	void count_parameter(int64_t value){
		for (ParameterCounter& entry : parameters) {
			if(entry.value == value){
				entry.count++;
				return;
			}
		}
		parameters.push_back({value, 1});
	}
	#endif
};

uint64_t hash_tree(Opcode::Mapping instruction, uint64_t parent_hash);

class InstructionNode{
	public:
		InstructionNode(){

		}

		InstructionNode(Opcode::Mapping instruction, uint64_t parent_hash)
				: instruction(instruction), weight(0){
					subtree_hash = ((parent_hash << 6) | (parent_hash >> 58)) ^ instruction;
		}

		Opcode::Mapping instruction;
		//the number of times this node occurred
		uint64_t weight;
		//weight counting only unique paths that contain this node
		//exclude paths that already contain this node with another prefix (e.g. ADD -> SUB -> ADD -> SUB)
		//used to calculate the coverage of a sequence (length * true_weight) 
		//using the normal weight can lead to overestimation of coverage (sequence > 100% coverage)
		uint64_t true_weight = 0;

		//last step id this node occurred in
		//used to lock true_weight  to prevent counting the same path multiple times
		//update last_occurrence when true_weight is updated
		//a node at this depth ends a window of depth+1 instructions, so the next window is
		//disjoint from the last counted one only if its step id > last_occurrence + depth
		uint64_t last_occurrence = 0;

		uint64_t total_cycles = 0;
		//sum of the step ids this node occurred in
		//dividing by weight results in the average region of the programs lifetime this node occurs most frequently
		//uint64_t sum_step_ids = 0;
		std::array<uint64_t, 4> occurrence = {0,0,0,0}; //O_STARTUP, O_BEGINNING, O_MID, O_END
		//array of negative offsets to last node that writes to rs1 or rs2 (1=this node depends on tree[current-index])
		//very likely this only marks 2 values for longer (unique) paths
		//must be zero initialized here: a node is created with new, which leaves a plain member
		//array indeterminate, and the garbage shows up as dependencies that never occurred
		std::array<bool, INSTRUCTION_TREE_DEPTH> dependencies_true_ = {}; //value at offset 0 is ignored
		//index of other nodes this node has a anti/output dependency to
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_anti_;
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_output_;

		std::bitset<32> inputs_;
		std::bitset<32> outputs_;


		#ifdef trace_individual_registers
		std::map<uint64_t, RegisterSetCounter> register_sets;
		#endif

		uint64_t subtree_hash = 0;

		//additional metrics


		virtual InstructionNode* insert(const StepInsertInfo& p) = 0;

		virtual float get_score_bonus(){
			using namespace Opcode;
			switch (instruction)
			{
			case BEQ:
			case BNE:
			case BLT:
			case BLTU:
			case BGE:
			case BGEU:
			case JAL:
			case JALR:
				//branch instruction calling base score function
				//see get_score_multiplier()
				return -1.0;
				break;
			default:
		#ifndef dependency_score
			return 0.0;
		#else
			//check for any dependencies
			uint64_t dependencies_count = 0;
			for (size_t i = 0; i < trace_depth; i++)
			{
			if(dependencies_true_[i]){
				dependencies_count++;
			}
			}

			if(dependencies_count>0){
				return -(float)dependencies_count/10.0;
			}else{
				return 2.0;
			}
			return 0.0;

		#endif
				break;
			}
		}
		virtual float get_score_multiplier(){
			using namespace Opcode;
			switch (instruction)//TODO probably just use a lookup table
			{
			case ADD:
				return 1.0;
				break;
			case BEQ:
			case BNE:
			case BLT:
			case BLTU:
			case BGE:
			case BGEU:
			case JAL:
			case JALR:
				//branch instruction calling base score function
				//this should only happen with the root node which is currently always a base R node
				//in case the root node is a branch, multiply global score by one, 
				//but set score bonus to -1
				//this essentially removes the node from the tree, which means there should always be 
				//exactly one other sequence with the same score and identical instructions, excluding the branch
				return 1.0;
				break;
			
			default:
				break;
			}
			
			return 1.0;
		}


		virtual double get_inv_dep_score(){
			using namespace Opcode;
			double result = 0.0;
			switch (instruction)
			{
			case BEQ:
			case BNE:
			case BLT:
			case BLTU:
			case BGE:
			case BGEU:
				return result;
				break;
			default:

			//check for any dependencies
			//TODO: include depth so we do not have to iterate over unnecessary empty entries
			for (size_t i = 1; i < trace_depth; i++)
			{
			if(dependencies_true_[i]){
				result += 1/(double)i; //i should never be 0 as a node does not depend on itself
			}
			if(dependencies_output_[i]){
				result += 1/(double)i;
			}
			if(dependencies_anti_[i]){
				result += 1/(double)i;
			}
			}

				return result;
				break;
			}
		}

		uint32_t count_true_dependencies(){
			uint32_t result = 0;
			for (size_t i = 0; i < trace_depth; i++)
			{
			if(dependencies_true_[i]){
				result++;
			}
			}
			return result;
		}

		void print(){
			const char* instruction_string = "UNKWN ";
			if(instruction < Opcode::mappingStr.size()){
				instruction_string = Opcode::mappingStr[instruction];
			}
			std::cout << "--------\n";
			std::cout << "](" << instruction_string << ")[\n";
			std::cout << "--" << weight << "--\n";
			_print(1);
		}

		virtual void _print(uint8_t level) = 0;

		void tree_to_dot(uint64_t total_instructions){
			std::stringstream dot_stream; 
			std::stringstream connections_stream; 

			//now defined in iss
			//dot_stream << "digraph g{" << std::endl;
			//dot_stream << "node [shape = record, style=\"bold\", height = .5, colorscheme=rdpu9];" << std::endl; //pubu9

			dot_stream << "//Nodes" << std::endl;
			connections_stream << "//Connections" << std::endl;

			const char* instruction_string = "UNKWN ";
			if(instruction < Opcode::mappingStr.size()){
				instruction_string = Opcode::mappingStr[instruction];
			}

			to_dot(instruction_string, "", 0, 0, 0, 
				dot_stream,connections_stream,
				weight,total_instructions, 
				true, 0.05);


			dot_stream << connections_stream.str();

			std::cout << dot_stream.str() << std::endl;
		}
		
		virtual nlohmann::ordered_json to_json(){
			
			nlohmann::ordered_json jsonNode;
			jsonNode["instruction"] = Opcode::mappingStr[instruction];
			jsonNode["type"] = get_node_type();
			jsonNode["weight"] = weight;
			jsonNode["true_weight"] = true_weight;
			jsonNode["subtree_hash"] = subtree_hash;

			#ifdef trace_individual_registers
				nlohmann::json jsonRegisterSets = nlohmann::json::object();
				for (const auto& entry : register_sets) {
					uint64_t key = entry.first;
					const RegisterSetCounter& rsc = entry.second;
					nlohmann::json jsonEntry = { {"count", rsc.count}, {"rs1", rsc.regset.rs1}, {"rs2", rsc.regset.rs2}, {"rd", rsc.regset.rd} };
					#ifdef trace_predecessor_pcs
					if (!rsc.predecessors.empty()) {
						nlohmann::json jsonPredecessors = nlohmann::json::object();
						for (const auto& predecessor : rsc.predecessors) {
							jsonPredecessors[std::to_string(predecessor.first)] = predecessor.second;
						}
						jsonEntry["predecessors"] = jsonPredecessors;
					}
					#endif
					jsonRegisterSets[std::to_string(key)] = jsonEntry;
				}
				jsonNode["register_sets"] = jsonRegisterSets;
			#endif 
			//convert dependencies
			std::vector<int> true_dependencies; //offset to previous node this node has a true dependency to
			std::set<int8_t> anti_dependencies;
			std::set<int8_t> output_dependencies;

			for (size_t i = 1; i < trace_depth; i++){
					if(dependencies_true_[i]){
						true_dependencies.push_back(i);
					}
					if (dependencies_anti_[i]) {
						anti_dependencies.insert(i);
					}
					if (dependencies_output_[i]) {
						output_dependencies.insert(i);
					}
			}

			std::set<int8_t> inputs;
			std::set<int8_t> outputs;

			for (size_t i = 0; i < 32; i++){
					if(inputs_[i]){
						inputs.insert(i);
					}
					if (outputs_[i]) {
						outputs.insert(i);
					}
			}

			nlohmann::json jsonDependencies1 = true_dependencies;
			jsonNode["dependencies_true"] = jsonDependencies1;
			nlohmann::json jsonDependencies2 = anti_dependencies;
			jsonNode["dependencies_anti"] = jsonDependencies2;
			nlohmann::json jsonDependencies3 = output_dependencies;
			jsonNode["dependencies_output"] = jsonDependencies3;

			jsonNode["inputs"] = inputs;
			jsonNode["outputs"] = outputs;

			jsonNode["occurrence"] = occurrence;
			
			#ifdef trace_parameter
			//same shape as before the values moved into register_sets: [[pc, [[value, count], ...]], ...]
			nlohmann::json jsonParameters = nlohmann::json::array();
			for (const auto& entry : register_sets) {
				if(entry.second.parameters.empty()){
					continue;
				}
				nlohmann::json jsonValues = nlohmann::json::array();
				for (const ParameterCounter& value : entry.second.parameters) {
					jsonValues.push_back({value.value, value.count});
				}
				jsonParameters.push_back({entry.first, jsonValues});
			}
			jsonNode["parameters"] = jsonParameters;
			#endif

			return jsonNode;
	}

		//might be easier to use a struct but this way its harder to miss a parameter
		virtual std::stringstream csv_format(uint64_t parent_hash, const char* tree, const char* instruction_string,
										uint64_t last_weight, uint64_t max_weight, uint64_t total_max_weight, uint32_t depth, 
										double current_dep_score, double current_total_dep_score, 
										uint32_t current_true_dep, uint32_t current_anti_dep, uint32_t current_out_dep, 
										uint32_t total_true_dep, uint32_t total_anti_dep, uint32_t total_out_dep, 
										uint32_t num_children, uint32_t num_current_total_inputs, uint32_t num_current_total_outputs, 
										uint32_t num_branches, uint64_t number_of_pcs, uint64_t max_pcs){
			std::stringstream csv_stream; 
			csv_stream << subtree_hash << ";" //ID
					<< parent_hash << ";" //parent subtree hash
					<< tree << ";" //Tree
					<< instruction_string << ";" //This instruction 
					<< weight << ";"
					<< true_weight << ";"
					<< last_weight - weight << ";"
					<< max_weight - weight << ";"
					<< total_max_weight - weight << ";"
					<< depth << ";" //Length
					<< trace_depth - depth << ";" //Length
					<< -1 << ";" //cycles used by sequence for one iteration
					<< current_dep_score << ";"
					<< current_total_dep_score << ";"
					<< current_true_dep << ";" //dependencies_true_ TODO convert to bitset
					<< current_anti_dep << ";"
					<< current_out_dep << ";"
					<< total_true_dep << ";" 
					<< total_anti_dep << ";" 
					<< total_out_dep << ";" 
					<< num_children << ";" //children
					<< Opcode::NUMBER_OF_INSTRUCTIONS - num_children << ";"
					<< inputs_.count() << ";"
					<< num_current_total_inputs << ";"
					<< outputs_.count() << ";"
					<< num_current_total_outputs << ";"
					<< -1 << ";" //TODO Instruction Types
					<< num_branches << ";" //Number of Branches
					<< occurrence[0] << ";"
					<< occurrence[1] << ";"
					<< occurrence[2] << ";"
					<< occurrence[3] << ";"
					<< number_of_pcs << ";"
					<< max_pcs - number_of_pcs << ";";

			return csv_stream;
		}

		virtual void to_csv(const CsvParams& p){
			std::stringstream csv_stream; 
			std::map<InstructionType, uint32_t> _instruction_types = 
											p.instruction_types;

			double current_dep_score = get_inv_dep_score();
			double current_total_dep_score = p.last_dep_score + current_dep_score;

			uint32_t current_true_dep = count_true_dependencies();
			uint32_t current_anti_dep = dependencies_anti_.count();
			uint32_t current_out_dep = dependencies_output_.count();

			uint32_t total_true_dep = p.true_dep +  current_true_dep;
			uint32_t total_anti_dep = p.anti_dep + current_anti_dep;
			uint32_t total_out_dep = p.out_dep + current_out_dep;

			std::bitset<32> current_total_inputs = p.total_inputs | inputs_;
			std::bitset<32> current_total_outputs = p.total_outputs | outputs_;

			uint64_t number_of_pcs = get_pc().size();
			_instruction_types[getInstructionType(instruction)]++;

			const char* instruction_string = "UNKWN ";
			if(instruction < Opcode::mappingStr.size()){
				instruction_string = Opcode::mappingStr[instruction];
			}
				csv_stream = csv_format(p.parent_hash, p.tree, instruction_string, p.last_weight ,p.max_weight, p.total_max_weight, 
								p.depth, current_dep_score, current_total_dep_score, 
								current_true_dep, current_anti_dep, current_out_dep, 
								total_true_dep, total_anti_dep,total_out_dep, 
								0, current_total_inputs.count(), 
								current_total_outputs.count(), _instruction_types[InstructionType::Branch], 
								number_of_pcs, p.max_pcs);

			std::cout << csv_stream.str() << std::endl;
		}

		virtual std::map<uint64_t, int> get_pc() = 0;

		virtual std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) = 0;

		float dot_hue(uint depth){
			return (float)depth/(float)trace_depth;
		}
		float dot_sat(uint depth, uint depender){
			//TODO higher saturation and value for instr with multiple children depending on it
			return 1.0;
		}
		float dot_val(uint depth, uint depender){
			return 1.0;
		}
		//dependencies should be 0 if not applicable
		virtual void update_weight(const StepUpdateInfo& p, uint32_t depth = 0){
			weight++; 
			total_cycles += p.cycles;
			//sum_step_ids += p.step; //TODO add curent step
			
			// Update true_weight only if this window does not overlap the last counted one.
			// p.step is the step id of the window's last instruction and the window is
			// depth+1 instructions long, so it must start after the last counted one ended.
			// The first occurrence is always counted: it can end at step id depth, which the
			// comparison alone would reject.
			if (true_weight == 0 || (last_occurrence + depth) < p.step) {
				true_weight++;
				last_occurrence = p.step;
			}

			#ifdef trace_individual_registers
			//everything that is tracked per pc shares this single lookup, it runs for every node of
			//every executed instruction
			auto it = register_sets.find(p.pc);
			if(it == register_sets.end()) {
				it = register_sets.emplace(p.pc, RegisterSetCounter{static_cast<int8_t>(p.rs1), static_cast<int8_t>(p.rs2), static_cast<int8_t>(p.rd)}).first;
			} else {
				it->second.count++;
			}
			#ifdef trace_predecessor_pcs
			//which pc this occurrence was actually reached from, so later, a pc path can be proven instead of guessed
			it->second.predecessors[p.predecessor_pc]++;
			#endif
			#ifdef trace_parameter
			//record the value the ISS decoded for this instruction: shift amount, branch/jump target or,
			//with trace_parameter_immediates, the decoded immediate. Values may be negative.
			if (p.parameter != NO_PARAMETER
					#ifndef trace_root_parameters
					//tracking the root node costs one entry per pc executing this opcode with little benefit
					&& depth > 0
					#endif
				) {
				it->second.count_parameter(p.parameter);
			}
			#endif
			#endif
			#ifdef handle_self_modifying_code
			else{
				auto it = register_sets.find(p.pc);
				RegisterSet rset = it->second;
				if(rset.rs1 != p.input1 || rset.rs2 != p.input2 || rset.rd != p.output){
					printf("detected binary modification (%x -> %x)\n"), p.pc, 1-p.pc;
					register_sets.emplace(1-p.pc, RegisterSet{p.input1, p.input2, p.output});
				}
			}
			#endif
			// if(p.step > O_MID){
			// 	occurrence[3]++;
			// }else if(p.step > O_BEGINNING){
			// 	occurrence[2]++;
			// }else if(p.step > O_STARTUP){
			// 	occurrence[1]++;
			// }else{
			// 	occurrence[0]++;
			// }
			if(p.dependency1>0){
				dependencies_true_[p.dependency1] = true;
			}else if(p.input1>=0){
				inputs_.set(p.input1, true);
			}
			if(p.dependency2>0){
				dependencies_true_[p.dependency2] = true;
			}else if(p.input2>=0){
				inputs_.set(p.input2, true);
			}
			if(p.output > 0){//ignore outputs for zero reg
				outputs_.set(p.output, true);
			}
			dependencies_anti_ |= p.anti_dependencies;
    		dependencies_output_ |= p.output_dependencies;
		}

		//called recursively for children and extends path if new score > old score
		//force_extension always extends the path with child at 
		//depth == force_extension_depth and child_instruction == instruction
		//if force extension == -1 -> don't force any extension
		//non R Nodes ignore force_extension as they don't have any children
		virtual Path extend_path(const PathExtensionParams& p){
			Path max_path;
			max_path.length = p.length;
			max_path.minimum_weight = weight;

			//score multiplier for this singular node
			//we can't simply pass its weight * mult as score as the weight changes when extending the path  
			//instead track a bonus multiplier that adds mult * min_weight to the score 
			//can be negative
			float score_bonus_of_this_node = get_score_bonus();
			//global score multiplier of this node
			//including branches etc. should reduce the score of the whole path, so its not chosen
			float score_multiplier_of_this_node = get_score_multiplier();

			max_path.score_bonus = p.score_bonus + score_bonus_of_this_node;
			max_path.score_multiplier = 
			p.score_multiplier * score_multiplier_of_this_node; 
			max_path.inverse_dependency_score = get_inv_dep_score();

			max_path.opcodes.push_back(instruction);
			max_path.path_hashes.push_back(subtree_hash);

			max_path.end_of_sequence = this;

			return max_path;

		}

		virtual std::vector<Path> extend_top_paths(const PathExtensionParams& p, size_t top_k){
			if (top_k == 0)
				return {};

			Path path = extend_path(p);
			if (path.length == 0)
				return {};

			return {path};
		}

		//might happen if best sequence length == Max Tree Depth
		virtual std::vector<Path> force_path_extension(Path p, std::function <float(ScoreParams)> score_function){
			printf("Warning: Forcing path extension of non R Node\nConsider increasing the maximum tree depth\n");
			return {};
		}

		virtual std::vector<PathNode> path_to_path_nodes(Path path, uint depth){
			std::vector<PathNode> nodes; 

			std::set<int8_t> indices_anti;
			std::set<int8_t> indices_out;
			for (size_t i = 0; i < trace_depth; i++) {
				if (dependencies_anti_[i]) {
					indices_anti.insert(i);
				}
				if (dependencies_output_[i]) {
					indices_out.insert(i);
				}
			}
			//PathNode(Opcode::Mapping instr, uint64_t wt, float score_b, float score_m, float inv_d, std::map<uint64_t, int> pcs, std::array<bool, INSTRUCTION_TREE_DEPTH> deps) {
			PathNode n = PathNode(instruction, weight, get_score_bonus(), get_score_multiplier(), get_inv_dep_score(), 
									get_pc(), 
									dependencies_true_, indices_out, indices_anti);
			nodes.push_back(n);
			return nodes;
		} 

		//find a point in an existing sequence with the highest ratio between branch taken in the original sequence 
		// and another possible branch not taken, which would lead to a different sequence 
		virtual std::vector<BranchingPoint> find_variant_branch(Path path, uint8_t depth){
			return {}; //no possible branching points for leaf nodes
		}

		virtual int prune_tree(uint64_t weight_threshold, uint8_t depth){
			return 0;
		}

		virtual NODE_TYPE get_node_type(){
			return NODE_TYPE::BASE;
		}

};

class InstructionNodeR : virtual public InstructionNode{
	public:
		//static const NODE_TYPE node_type = NODE_TYPE::NODE;
		InstructionNodeR(Opcode::Mapping instruction, uint64_t parent_hash);

		std::list<InstructionNode*> children;

		//the ring buffer is only read, so it is passed by reference: it is copied once per executed
		//instruction otherwise, which is 120 bytes per tree depth step
		void insert_rb(const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_instructions, 
						uint32_t next_rb_index);
		void insert_rb(const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_instructions, 
						uint32_t next_rb_index, uint32_t offset);

		InstructionNode* insert(const StepInsertInfo& p) override;

		void _print(uint8_t level) override;

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) override;
		void to_csv(const CsvParams& p) override;

		nlohmann::ordered_json to_json() override;
		
		//finds the most promising optimization sequence for this tree by evaluating every possible sequence
		Path extend_path(const PathExtensionParams& p) override;
		std::vector<Path> extend_top_paths(const PathExtensionParams& p, size_t top_k) override;
		//extend existing path beyond its original endpoint
		//expects an existing path + first Node new path that should be extended
		//handles possible branch instructions and the calls extend_path() 
		std::vector<Path> force_path_extension(Path p, std::function <float(ScoreParams)> score_function) override;
		std::map<uint64_t, int> get_pc() override;

		std::vector<PathNode> path_to_path_nodes(Path path, uint depth) override; 
		std::vector<BranchingPoint> find_variant_branch(Path path, uint8_t depth) override;
		int prune_tree(uint64_t weight_threshold, uint8_t depth) override;
		NODE_TYPE get_node_type() override;
};

class InstructionNodeLeaf : virtual public InstructionNode{
	public:
		InstructionNodeLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc);

	std::map<uint64_t, int> pc_map;

	InstructionNode* insert(const StepInsertInfo& p) override;

	void _print(uint8_t level) override;

	std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) override;

	nlohmann::ordered_json to_json() override;

	virtual void update_weight(const StepUpdateInfo& p, uint32_t depth = 0) override;
	std::map<uint64_t, int> get_pc() override;
	NODE_TYPE get_node_type() override;
};

inline void Path::to_csv_stats(std::ostream& output_file, uint64_t total_instructions) const {
	for (auto&& opcode : opcodes) {
		const char* opcode_string = "UNKWN ";
		output_file << "\"";
		if (opcode < Opcode::mappingStr.size()) {
			opcode_string = Opcode::mappingStr[opcode];
		}
		output_file << opcode_string << " -> ";
	}
	output_file << "\"";
	uint64_t true_weight = 0;
	if (end_of_sequence) {
		true_weight = end_of_sequence->true_weight;
	}
	double coverage = 0.0;
	if (total_instructions > 0) {
		coverage = (static_cast<double>(length) * static_cast<double>(true_weight)) /
			static_cast<double>(total_instructions);
	}
	output_file << ";" << length << ";" << minimum_weight
		<< ";" << get_score([](const ScoreParams p) { return p.length * p.weight; })
		<< ";" << path_hashes.back() << ";" << coverage << "\n";
}

class MemoryNode{
	public: 
		bool is_store = false;
		//Opcode::MemoryRegion memory_location = Opcode::MemoryRegion::NONE;//Stack(current frame=1 else 2) or Heap (4) or both (3,5,6,7)
		std::unordered_map<uint64_t, std::unordered_map<uint64_t, Opcode::MemoryRegion>> memory_accesses;
		uint64_t last_access = 0;
		uint64_t access_offset_sum = 0;
		std::unordered_map<uint64_t, std::string> peripheral_by_address; //address -> peripheral name, for accesses that hit a registered peripheral region
		std::unordered_map<std::string, uint64_t> peripheral_access_counts; //peripheral name -> number of accesses

		MemoryNode(){};
		MemoryNode(bool is_store_instruction);

		nlohmann::ordered_json memory_to_json(){
			nlohmann::ordered_json json; 
			json["LS"] = is_store;
			json["Accesses"] = memory_accesses;
			json["OffsetSum"] = access_offset_sum;
			if(peripheral_access_counts.size()>0){
				json["Peripherals"] = peripheral_by_address;
				json["PeripheralAccessCounts"] = peripheral_access_counts;
			}
			return json;
		};

		void register_access(uint64_t pc, uint64_t address, AccessType access_type,uint64_t prev_access, 
								uint64_t stackpointer, uint64_t framepointer, const char* peripheral_name = nullptr);

};

//taken/not-taken counts and the encoded offset of one branch/jump pc
struct BranchOutcomeCounter {
	int64_t offset = 0; //encoded relative offset (B_imm/J_imm), or I_imm for JALR whose target is dynamic
	uint64_t taken = 0;
	uint64_t not_taken = 0;
};

class BranchNode{
	public:
		// std::unordered_map<uint64_t, std::unordered_set<uint64_t>> jump_targets; //Handled as a parameter
		//relative offset (as unsigned wraparound) -> how often the branch was taken with that offset
		std::map<uint64_t, uint64_t> relative_offsets;
		#ifdef trace_branch_outcomes
		std::map<uint64_t, BranchOutcomeCounter> branch_outcomes; //pc -> outcome counts
		#endif
		bool is_backward_jump = false;
		bool is_forward_jump = false;

		BranchNode(){};
		BranchNode(int64_t offset);

		nlohmann::ordered_json branch_to_json(){
			nlohmann::ordered_json json;
			json["Direction"] = (is_backward_jump * 1) + (is_forward_jump * 2);
			json["offsets"] = relative_offsets;
			#ifdef trace_branch_outcomes
			nlohmann::json jsonOutcomes = nlohmann::json::object();
			for (const auto& entry : branch_outcomes) {
				jsonOutcomes[std::to_string(entry.first)] = {
					{"offset", entry.second.offset},
					{"taken", entry.second.taken},
					{"not_taken", entry.second.not_taken}};
			}
			json["BranchOutcomes"] = jsonOutcomes;
			#endif
			return json;
		};

		//record the actual outcome of one dynamic execution of this branch/jump.
		//pc_relative is false for JALR, whose offset is relative to rs1 instead of the pc,
		//so it must not be folded into the pc relative Direction/offsets fields
		void register_branch(uint64_t pc, BranchOutcome outcome, int64_t offset, bool pc_relative);

};

class InstructionNodeMemory : public InstructionNodeR, virtual public MemoryNode{
	public: 
		InstructionNodeMemory(Opcode::Mapping instruction, uint64_t parent_hash, 
								uint64_t memory_access, bool is_store_instruction);

		void _print(uint8_t level) override;

		void update_weight(const StepUpdateInfo& p, uint32_t depth = 0) override;

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) override;

	nlohmann::ordered_json to_json() override{
		nlohmann::ordered_json additional_fields = MemoryNode::memory_to_json();
		nlohmann::ordered_json base_class_json = InstructionNodeR::to_json();
		base_class_json.update(additional_fields);
		return base_class_json;
	}
	std::vector<PathNode> path_to_path_nodes(Path path, uint depth) override;
	NODE_TYPE get_node_type() override;
};

class InstructionNodeMemoryLeaf : public InstructionNodeLeaf, virtual public MemoryNode{
	public: 
		InstructionNodeMemoryLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc, 
									uint64_t memory_access, bool is_store_instruction);


		void _print(uint8_t level) override;

		void update_weight(const StepUpdateInfo& p, uint32_t depth = 0) override;

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) override;

	nlohmann::ordered_json to_json() override{
		nlohmann::ordered_json additional_fields = MemoryNode::memory_to_json();
		nlohmann::ordered_json base_class_json = InstructionNodeLeaf::to_json();
		base_class_json.update(additional_fields);
		return base_class_json;
	}
	std::vector<PathNode> path_to_path_nodes(Path path, uint depth) override;
	NODE_TYPE get_node_type() override;
};


class InstructionNodeBranch : public InstructionNodeR, virtual public BranchNode{
	public: 
		InstructionNodeBranch(Opcode::Mapping instruction, uint64_t parent_hash, int64_t offset);

		float get_score_multiplier() override{
			using namespace Opcode;
			switch (instruction){
			case BEQ:
			case BGE:
			case BGEU:
			case BNE:
			case BLT:
			case BLTU:
			case JAL:
			case JALR:
				return 0.0;
				break;
			
			default:
				printf("[ERROR] Non branch node calling branch score function %d", instruction);
				break;
			}
			
			return 1.0;
		}
	void update_weight(const StepUpdateInfo& p, uint32_t depth = 0) override;
	NODE_TYPE get_node_type() override;
	nlohmann::ordered_json to_json() override{
		nlohmann::ordered_json additional_fields = BranchNode::branch_to_json();
		nlohmann::ordered_json base_class_json = InstructionNodeR::to_json();
		base_class_json.update(additional_fields);
		return base_class_json;
	}
};

class InstructionNodeBranchLeaf : public InstructionNodeLeaf, virtual public BranchNode{
	public: 
		InstructionNodeBranchLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc, 
									int64_t offset);


		float get_score_multiplier() override{
			using namespace Opcode;
			switch (instruction){
			case BEQ:
			case BGE:
			case BGEU:
			case BNE:
			case BLT:
			case BLTU:
			case JAL:
			case JALR:
				return 0.0;
				break;
			
			default:
				printf("[ERROR] Non branch leaf node calling branch score function %d", instruction);
				break;
			}
			
			return 1.0;
		}
	void update_weight(const StepUpdateInfo& p, uint32_t depth = 0) override;
	NODE_TYPE get_node_type() override;
	nlohmann::ordered_json to_json() override{
		nlohmann::ordered_json additional_fields = BranchNode::branch_to_json();
		nlohmann::ordered_json base_class_json = InstructionNodeLeaf::to_json();
		base_class_json.update(additional_fields);
		return base_class_json;
	}
};
