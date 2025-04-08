#pragma once

#include "instr.h"
#include <set>
#include <bitset>

#include "lib/json/single_include/nlohmann/json.hpp"

#define INSTRUCTION_TREE_DEPTH 10
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
//#define debug_register_dependencies
//#define debug_dependencies
// #define handle_self_modifying_code
//#define trace_individual_registers

#define single_trace_mode

//#define dot_pc_on_pruned_nodes

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

InstructionType getInstructionType(Opcode::Mapping mapping);

struct ExecutionInfo {
	Opcode::Mapping last_executed_instruction;
	uint64_t last_cycles;
	uint8_t last_powermode;

	std::tuple<uint16_t,uint16_t,uint16_t> last_registers;
	uint64_t last_executed_pc;

	uint64_t last_memory_read;
	uint64_t last_memory_written;
	AccessType last_memory_access_type;
	uint64_t last_stack_pointer;
	uint64_t last_frame_pointer;

	uint64_t last_step_id;
};

struct StepInsertInfo {
    Opcode::Mapping op;
    uint64_t pc;
    int8_t true_dependency1;
    int8_t true_dependency2;
    std::bitset<INSTRUCTION_TREE_DEPTH> output_dependencies;
    std::bitset<INSTRUCTION_TREE_DEPTH> anti_dependencies;
    int8_t input1;
    int8_t input2;
    int8_t output;
    uint32_t depth;
    uint64_t step;
	uint64_t cycles;
	uint64_t memory_address;
	AccessType access_type;
	uint64_t stack_pointer;
	uint64_t frame_pointer;
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
};

class InstructionNode;

struct Path
{
	uint32_t length = 0;
	uint64_t minimum_weight = 0;
	float score_bonus = 0;
	float score_multiplier = 1.0;

	double inverse_dependency_score = 0.0;
	std::vector<uint64_t> path_hashes;
	std::vector<Opcode::Mapping> opcodes;
	InstructionNode* end_of_sequence;

	//double score = 0; //TODO should this be saved here? How should we handle initialization?

	float get_score(std::function <float(ScoreParams)> score_function){
		uint32_t num_children = 0; //TODO end_of_sequence test for type and count children  
		uint32_t inputs = 0; //TODO
		uint32_t outputs = 0; //TODO 
		ScoreParams params = {opcodes.back(), opcodes[0], 
		minimum_weight, length, inverse_dependency_score, 
		num_children, inputs, outputs, score_multiplier, score_bonus};
		return score_function(params);
		// length * minimum_weight * score_multiplier 
		// 		+ minimum_weight * score_bonus; //length * minimum_weight;
	}
	float get_normalized_score(){
		return length / (1.0 + inverse_dependency_score);
	}

	void show(){
		show("");
	}
	void show(const char* prefix){
		auto sf = [](const ScoreParams p) {
			float score = (p.length * p.weight) * p.score_multiplier 
				+ p.weight * p.score_bonus; //length * minimum_weight;
			return score;
		};
		show(prefix, sf);
	}

	void show(const char* prefix, std::function <float(const ScoreParams)> score_function){
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
		// std::cout << "<Path Hashes>\n";
		// for (auto &&hash : path_hashes)	
		// {
		// 	std::cout << hash << " --> ";

		// }
		// std::cout << "\n\n";
	}
};


//used to represent a node in an identified path/sequence
//used for exporting identified sequences 
struct PathNode {
    Opcode::Mapping instruction;
    uint64_t weight;

	//uint64_t cycles;
    //uint64_t subtree_hash;
	float score_bonus;
	float score_multiplier;
	float inverse_dependency_score;

	//usually only for leaf nodes, but we can calculate this from following all branches from the current node
	std::map<uint64_t, int> program_counters;

	std::vector<int> true_dependencies; //offset to previous node this node has a true dependency to
	std::set<int8_t> anti_dependencies;
	std::set<int8_t> output_dependencies;

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

uint64_t hash_tree(Opcode::Mapping instruction, uint64_t parent_hash);

class InstructionNode{
	public:
		InstructionNode(){

		}

		InstructionNode(Opcode::Mapping instruction, uint64_t parent_hash)
				: instruction(instruction), weight(0){
					subtree_hash = ((parent_hash << 6) | (parent_hash >> 58)) ^ instruction;
					for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)//TODO remove should already be 0 initialized
					{
						dependencies_true_[i] = false;
						
					}
					dependencies_anti_.reset();
					dependencies_output_.reset();
					
		}

		Opcode::Mapping instruction;
		//the number of times this node occurred
		uint64_t weight;

		uint64_t total_cycles = 0;
		//sum of the step ids this node occurred in
		//dividing by weight results in the average region of the programs lifetime this node occurs most frequently
		//uint64_t sum_step_ids = 0;
		std::array<uint64_t, 4> occurrence = {0,0,0,0}; //O_STARTUP, O_BEGINNING, O_MID, O_END
		//array of negative offsets to last node that writes to rs1 or rs2 (1=this node depends on tree[current-index])
		//very likely this only marks 2 values for longer (unique) paths
		std::array<bool, INSTRUCTION_TREE_DEPTH> dependencies_true_; //value at offset 0 is ignored
		//index of other nodes this node has a anti/output dependency to
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_anti_;
		std::bitset<INSTRUCTION_TREE_DEPTH> dependencies_output_;

		std::bitset<32> inputs_;
		std::bitset<32> outputs_;

		#ifdef trace_individual_registers
		std::map<uint64_t, RegisterSet> register_sets;
		#endif

		uint64_t subtree_hash = 0;

		std::map<uint64_t, int> pc_map;

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
			for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)
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
			for (size_t i = 1; i < INSTRUCTION_TREE_DEPTH; i++)
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
			for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)
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
			jsonNode["subtree_hash"] = subtree_hash;
			jsonNode["PCs"] = pc_map;

			#ifdef trace_individual_registers
			nlohmann::json jsonRegisterSets = nlohmann::json::object();
			for (const auto& entry : register_sets) {
				uint64_t key = entry.first;
				const RegisterSet& rs = entry.second;
				jsonRegisterSets[std::to_string(key)] = { rs.rs1, rs.rs2, rs.rd };
			}

			jsonNode["register_sets"] = jsonRegisterSets;
			#endif 
			//convert dependencies
			std::vector<int> true_dependencies; //offset to previous node this node has a true dependency to
			std::set<int8_t> anti_dependencies;
			std::set<int8_t> output_dependencies;

			for (size_t i = 1; i < INSTRUCTION_TREE_DEPTH; i++){
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
					<< last_weight - weight << ";"
					<< max_weight - weight << ";"
					<< total_max_weight - weight << ";"
					<< depth << ";" //Length
					<< INSTRUCTION_TREE_DEPTH - depth << ";" //Length
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
			return (float)depth/(float)INSTRUCTION_TREE_DEPTH;
		}
		float dot_sat(uint depth, uint depender){
			//TODO higher saturation and value for instr with multiple children depending on it
			return 1.0;
		}
		float dot_val(uint depth, uint depender){
			return 1.0;
		}
		//dependencies should be 0 if not applicable
		virtual void update_weight(const StepUpdateInfo& p){
			weight++; 
			pc_map[p.pc]++;
			total_cycles += p.cycles;
			//sum_step_ids += p.step; //TODO add curent step
			#ifdef trace_individual_registers
			if(!register_sets.count(p.pc)){
				register_sets.emplace(p.pc, RegisterSet{p.input1, p.input2, p.output});
			}
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
			if(p.step > O_MID){
				occurrence[3]++;
			}else if(p.step > O_BEGINNING){
				occurrence[2]++;
			}else if(p.step > O_STARTUP){
				occurrence[1]++;
			}else{
				occurrence[0]++;
			}
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

		//might happen if best sequence length == Max Tree Depth
		virtual std::vector<Path> force_path_extension(Path p, std::function <float(ScoreParams)> score_function){
			printf("Warning: Forcing path extension of non R Node\nConsider increasing the maximum tree depth\n");
			return {};
		}

		virtual std::vector<PathNode> path_to_path_nodes(Path path, uint depth){
			std::vector<PathNode> nodes; 

			std::set<int8_t> indices_anti;
			std::set<int8_t> indices_out;
			for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++) {
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

		#ifdef single_trace_mode
		std::list<InstructionNodeR*> children;
		#else
		std::list<InstructionNode*> children; //can contain e.g. leaf nodes
		#endif

		#ifdef single_trace_mode
		InstructionNodeR* get_last();
		#endif
		void insert_rb(std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_instructions, 
						uint32_t next_rb_index);
		void insert_rb(std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_instructions, 
						uint32_t next_rb_index, uint32_t offset);

		#ifdef single_trace_mode
		InstructionNodeR* insert(const StepInsertInfo& p);
		#else
		InstructionNode* insert(const StepInsertInfo& p) override;
		#endif

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

	//std::map<uint64_t, int> pc_map;

	InstructionNode* insert(const StepInsertInfo& p) override;

	void _print(uint8_t level) override;

	std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold) override;

	nlohmann::ordered_json to_json() override;

	virtual void update_weight(const StepUpdateInfo& p);
	std::map<uint64_t, int> get_pc() override;
	NODE_TYPE get_node_type() override;
};

class MemoryNode{
	public: 
		bool is_store = false;
		//Opcode::MemoryRegion memory_location = Opcode::MemoryRegion::NONE;//Stack(current frame=1 else 2) or Heap (4) or both (3,5,6,7)
		std::map<uint64_t, std::set<std::pair<uint64_t, Opcode::MemoryRegion>>> memory_accesses;
		uint64_t last_access = 0;
		uint64_t access_offset_sum = 0;

		MemoryNode(){};
		MemoryNode(bool is_store_instruction);

		nlohmann::ordered_json memory_to_json(){
			nlohmann::ordered_json json; 
			json["LS"] = is_store;
			json["Accesses"] = memory_accesses;
			json["OffsetSum"] = access_offset_sum;
			return json;
		};

		void register_access(uint64_t pc, uint64_t address, AccessType access_type,uint64_t prev_access, 
								uint64_t stackpointer, uint64_t framepointer);

};

class BranchNode{
	public: 
		std::map<uint64_t, uint64_t> relative_offsets;
		bool is_backward_jump = false;
		bool is_forward_jump = false;

		BranchNode(){};
		BranchNode(int64_t offset);

		nlohmann::ordered_json branch_to_json(){
			nlohmann::ordered_json json; 
			json["Direction"] = (is_backward_jump * 1) + (is_forward_jump * 2);
			json["offsets"] = relative_offsets;
			return json;
		};

		void register_access(uint64_t pc, uint64_t address, AccessType access_type,uint64_t prev_access, 
								uint64_t stackpointer, uint64_t framepointer);

};

class InstructionNodeMemory : public InstructionNodeR, virtual public MemoryNode{
	public: 
		InstructionNodeMemory(Opcode::Mapping instruction, uint64_t parent_hash, 
								uint64_t memory_access, bool is_store_instruction);

		void _print(uint8_t level) override;

		void update_weight(const StepUpdateInfo& p) override;

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
	NODE_TYPE get_node_type() override;
};

class InstructionNodeMemoryLeaf : public InstructionNodeLeaf, virtual public MemoryNode{
	public: 
		InstructionNodeMemoryLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc, 
									uint64_t memory_access, bool is_store_instruction);


		void _print(uint8_t level) override;

		void update_weight(const StepUpdateInfo& p) override;

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
	NODE_TYPE get_node_type() override;
	nlohmann::ordered_json to_json() override{
		nlohmann::ordered_json additional_fields = BranchNode::branch_to_json();
		nlohmann::ordered_json base_class_json = InstructionNodeLeaf::to_json();
		base_class_json.update(additional_fields);
		return base_class_json;
	}
};
