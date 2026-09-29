#pragma once

#include "core/common/instr.h"
#include "trace/score.h"
#include <set>
#include <tuple>
#include <unordered_set>
#include <unordered_map>

#include <bitset>
#include <fstream>
#include <new>
#include <vector>

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

//also record parameters on the root node of a tree. Every occurrence of an instruction reaches
//the root of its tree, so this costs one parameter entry per pc executing that opcode.
//on by default; -DTRACE_ROOT_PARAMETERS=OFF drops it, which shrinks traces and speeds up tracing
#ifdef TRACE_ROOT_PARAMETERS
#define trace_root_parameters
#endif

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

enum class NODE_TYPE : uint8_t {
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

//! Every address a load or a store touched, mapped to the pc that last wrote it and the pc
//! that last read it. The dot export writes it out as a memory map.
using MemoryAccessMap = std::map<uint64_t, std::tuple<uint64_t, uint64_t>>;

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

//! Everything one executed instruction contributes to the node it lands on.
//! One struct for the whole insert path: `insert_rb` fills it once per step of a window and
//! `insert` passes the same object down, rather than copying 128 bytes per node.
struct StepInfo {
    Opcode::Mapping op;
    uint64_t pc;
    //offset back to the instruction that wrote this one's first/second source register, or -1
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
    //! Position in the window, so also the depth of the node this describes. 0 is the root.
    uint32_t depth;
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
	uint64_t true_weight = 0;
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
		uint64_t sequence_weight = minimum_weight; //use minimum weight for now, switch to true_weight after proerly verifying the behavior
		ScoreParams params = {opcodes.back(), opcodes[0], 
		sequence_weight, length, inverse_dependency_score, 
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
				std::map<uint64_t, int> pcs,
				const std::array<bool, INSTRUCTION_TREE_DEPTH> &dep_true,
				std::set<int8_t> dep_out, std::set<int8_t> dep_anti);

	nlohmann::json to_json() const;
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


//! Memory addresses one load or store node touched, and the peripherals they hit.
//! Allocated with the node (see InstructionNode::create), so only load and store nodes pay for it.
class MemoryNode{
	public: 
		bool is_store = false;
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

//! Outcomes of one branch or jump node. Allocated with the node, like MemoryNode.
class BranchNode{
	public:
		//relative offset (as unsigned wraparound) -> how often the branch was taken with that offset
		std::map<uint64_t, uint64_t> relative_offsets;
		#ifdef trace_branch_outcomes
		std::map<uint64_t, BranchOutcomeCounter> branch_outcomes; //pc -> outcome counts
		#endif
		bool is_backward_jump = false;
		bool is_forward_jump = false;

		BranchNode(){};

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

//! Opcodes that get a branch payload: the six conditional branches and the two jumps.
inline bool is_branch_opcode(Opcode::Mapping op){
	using namespace Opcode;
	switch (op){
		case BEQ: case BNE: case BLT: case BLTU: case BGE: case BGEU:
		case JAL: case JALR:
			return true;
		default:
			return false;
	}
}

//! Which memory payload an opcode gets. Only the rv32 word and sub word accesses are marked,
//! so the rv64 and floating point accesses record no addresses.
enum class MemoryOpcode : uint8_t { NONE, LOAD, STORE };
inline MemoryOpcode memory_opcode(Opcode::Mapping op){
	using namespace Opcode;
	switch (op){
		case LB: case LBU: case LH: case LHU: case LW:
			return MemoryOpcode::LOAD;
		case SB: case SH: case SW:
			return MemoryOpcode::STORE;
		default:
			return MemoryOpcode::NONE;
	}
}

//! The type of a new node: the payload its opcode needs, plus LEAF when it ends a window.
//! A root node is a plain NODE whatever its opcode, so a tree of loads records no addresses at
//! its root. Tracer::tree_for is the only place that creates one.
inline NODE_TYPE node_type_for(Opcode::Mapping op, bool leaf){
	uint8_t type = leaf ? (uint8_t)NODE_TYPE::LEAF : (uint8_t)NODE_TYPE::NODE;
	if(memory_opcode(op) != MemoryOpcode::NONE){
		type |= (uint8_t)NODE_TYPE::MEMORY;
	}else if(is_branch_opcode(op)){
		type |= (uint8_t)NODE_TYPE::BRANCH;
	}
	return (NODE_TYPE)type;
}


/**
 * One instruction of an execution sequence tree: every window that reaches it ran this
 * instruction at this position, and the node counts and describes those runs.
 *
 * `node_type` says what the node is. A LEAF ends a window and has no children. A MEMORY node
 * also carries the addresses its instruction touched, a BRANCH node the outcomes it took.
 * Those two payloads live directly after the node in one allocation, so a node that needs
 * neither costs nothing for them; reach them with memory() and branch().
 *
 * Nodes live until the process ends. `create` therefore allocates raw storage and no
 * destructor ever runs.
 */
class InstructionNode{
	public:
		InstructionNode() = default;

		InstructionNode(Opcode::Mapping instruction, uint64_t parent_hash, NODE_TYPE type)
				: instruction(instruction), node_type(type){
					subtree_hash = ((parent_hash << 6) | (parent_hash >> 58)) ^ instruction;
		}

		//! Allocate a child node for `op`, with the payload the opcode needs, in one block.
		static InstructionNode* create(Opcode::Mapping op, uint64_t parent_hash, bool leaf);

		Opcode::Mapping instruction = Opcode::UNDEF;
		//! What this node carries. Fixed when the node is created, except that pruning turns a
		//! node into a leaf.
		NODE_TYPE node_type = NODE_TYPE::NODE;

		uint64_t subtree_hash = 0;

		//the number of times this node occurred
		uint64_t weight = 0;
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

		#ifdef trace_individual_registers
		//! Per pc: how often this node ran at that pc, its registers, its predecessors and the
		//! parameter values seen there. Keyed by pc, so it is also where get_pc() comes from.
		std::map<uint64_t, RegisterSetCounter> register_sets;
		#endif

		//! One child and the opcode that leads to it. Keeping the opcode next to the pointer is
		//! what makes the search below cheap: it reads one contiguous array instead of loading
		//! every child node to look at its opcode, which costs a cache miss per child.
		struct Child {
			Opcode::Mapping op;
			InstructionNode* node;
		};

		//! Empty on a leaf, and on an inner node no window has reached past yet.
		std::vector<Child> children;

		//entry i is set when this node reads a value written i instructions earlier in the window
		//(1 = written by the parent). Entry 0 is never set: a node does not depend on itself.
		//a byte per offset rather than a bit mask: the two writes on the insert path are then
		//independent stores instead of a read-modify-write of one shared word, which measures
		//about 1.5 percent faster over the embench set. Must be zero initialized here, because
		//a node is created with raw storage and a plain member array would be left indeterminate.
		std::array<bool, INSTRUCTION_TREE_DEPTH> dependencies_true_ = {};
		//index of other nodes this node has a anti/output dependency to
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_anti_;
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_output_;

		std::bitset<32> inputs_;
		std::bitset<32> outputs_;

		//how often this node ran in each phase of the program, which would say whether an
		//instruction belongs to startup, to initialization or to the real workload.
		//Never written: the phase boundaries are fixed step ids (O_STARTUP and the rest) and the
		//length of the run is unknown while it is being traced, so they cannot be placed.
		std::array<uint64_t, 4> occurrence = {0,0,0,0}; //O_STARTUP, O_BEGINNING, O_MID, O_END

		// ------------------------------------------------------------- what this node is

		//! Ends a window, so it has no children and the exports stop here.
		bool is_leaf() const {
			return (uint8_t)node_type & (uint8_t)NODE_TYPE::LEAF;
		}
		bool has_memory() const {
			return (uint8_t)node_type & (uint8_t)NODE_TYPE::MEMORY;
		}
		bool has_branch() const {
			return (uint8_t)node_type & (uint8_t)NODE_TYPE::BRANCH;
		}
		NODE_TYPE get_node_type() const {
			return node_type;
		}

		//! The payload after this node. Only call the one has_memory()/has_branch() allows.
		MemoryNode* memory();
		BranchNode* branch();

		//! Does this node read a value written `offset` instructions earlier in the window?
		bool depends_true_on(size_t offset) const {
			return dependencies_true_[offset];
		}

		//! How many earlier instructions in the window this node reads a value from.
		uint32_t true_dependency_count() const {
			uint32_t count = 0;
			for (size_t i = 0; i < trace_depth; i++) {
				if (dependencies_true_[i]) {
					count++;
				}
			}
			return count;
		}

		// ------------------------------------------------------------- recording

		/**
		 * Insert the window ending at `next_rb_index` into this tree, this node being its root.
		 *
		 * Works out the register and memory dependencies between the instructions of the window
		 * and walks down the tree, creating the nodes that do not exist yet. Runs once per
		 * executed instruction and its cost grows with the trace depth, so it is the hottest
		 * function of the tracer.
		 *
		 * The ring buffer is only read, so it is passed by reference: it is copied once per
		 * executed instruction otherwise, which is 120 bytes per tree depth step.
		 */
		void insert_rb(const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_instructions, 
						uint32_t next_rb_index);
		void insert_rb(const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_instructions, 
						uint32_t next_rb_index, uint32_t offset);

		//! Find or create the child for one step of a window and count the step on it.
		InstructionNode* insert(const StepInfo& p);

		//! Count one occurrence of this node and record what the instruction did.
		void update_weight(const StepInfo& p);

		// ------------------------------------------------------------- reading

		//! The pcs this node ran at and how often, taken from register_sets.
		std::map<uint64_t, int> get_pc() const;

		//! The largest number of distinct pcs any node in this subtree ran at, which is what the
		//! csv export compares a single node's count against.
		uint64_t max_pc_count() const;

		float get_score_bonus() const;
		float get_score_multiplier() const;
		double get_inv_dep_score() const;

		void print();
		void _print(uint8_t level);

		/**
		 * Write this tree as dot to standard output. `branch_threshold` omits any branch
		 * carrying less than that share of the tree's weight, which keeps a graph of a real
		 * program readable; pass 0 to draw every branch.
		 */
		void tree_to_dot(uint64_t total_instructions, float branch_threshold);

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold);

		nlohmann::ordered_json to_json();

		void to_csv(const CsvParams& p);

		//! Highest scoring sequence starting at this node, over every branch below it.
		Path extend_path(const PathExtensionParams& p);
		//! The `top_k` highest scoring sequences starting at this node.
		std::vector<Path> extend_top_paths(const PathExtensionParams& p, size_t top_k);
		//extend existing path beyond its original endpoint
		//expects an existing path + first Node new path that should be extended
		//handles possible branch instructions and the calls extend_path() 
		std::vector<Path> force_path_extension(Path p, std::function <float(ScoreParams)> score_function);

		std::vector<PathNode> path_to_path_nodes(Path path, uint depth);

		//find a point in an existing sequence with the highest ratio between branch taken in the original sequence 
		// and another possible branch not taken, which would lead to a different sequence 
		std::vector<BranchingPoint> find_variant_branch(Path path, uint8_t depth);

		//! Turn every branch below the given share of this node's weight into a leaf.
		int prune_tree(uint64_t weight_threshold, uint8_t depth);

	private:
		//! Where create() put the payload: directly after the node.
		void* payload_slot() {
			return reinterpret_cast<char*>(this) + sizeof(InstructionNode);
		}

		std::stringstream csv_format(uint64_t parent_hash, const char* tree, const char* instruction_string,
										uint64_t last_weight, uint64_t max_weight, uint64_t total_max_weight, uint32_t depth, 
										double current_dep_score, double current_total_dep_score, 
										uint32_t current_true_dep, uint32_t current_anti_dep, uint32_t current_out_dep, 
										uint32_t total_true_dep, uint32_t total_anti_dep, uint32_t total_out_dep, 
										uint32_t num_children, uint32_t num_current_total_inputs, uint32_t num_current_total_outputs, 
										uint32_t num_branches, uint64_t number_of_pcs, uint64_t max_pcs);
};

//the payloads follow the node in the same block, so both need the node's size, which is only
//known here
static_assert(sizeof(InstructionNode) % alignof(MemoryNode) == 0,
		"the memory payload directly after the node must stay aligned");
static_assert(sizeof(InstructionNode) % alignof(BranchNode) == 0,
		"the branch payload directly after the node must stay aligned");

inline MemoryNode* InstructionNode::memory() {
	return std::launder(reinterpret_cast<MemoryNode*>(payload_slot()));
}
inline BranchNode* InstructionNode::branch() {
	return std::launder(reinterpret_cast<BranchNode*>(payload_slot()));
}
