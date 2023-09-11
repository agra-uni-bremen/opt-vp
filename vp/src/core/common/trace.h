#ifndef RISCV_ISA_TRACE_H
#define RISCV_ISA_TRACE_H

#include "instr.h"

#define INSTRUCTION_TREE_DEPTH 50



// extern std::array<const char*, NUMBER_OF_INSTRUCTIONS> mappingStr;

// Opcode::Type getType(Opcode::Mapping mapping);

struct ExecutionInfo {
	Opcode::Mapping last_executed_instruction;
	uint64_t last_cycles;
	uint8_t last_powermode;

	std::tuple<uint16_t,uint16_t,uint16_t> last_registers;
	uint64_t last_executed_pc;

	uint64_t last_memory_read;
	uint64_t last_memory_written;
};

struct Path
{
	uint32_t length = 0;
	uint64_t minimum_weight = 0;
	float score_bonus = 0;
	float score_multiplier = 1.0;

	double inverse_dependency_score = 0.0;
	std::vector<uint64_t> path_hashes;
	std::vector<Opcode::Mapping> opcodes;

	float get_score(){
		return length * minimum_weight * score_multiplier 
				+ minimum_weight * score_bonus; //length * minimum_weight;
	}
	float get_normalized_score(){
		return length / (1.0 + inverse_dependency_score);
	}

	void show(){
		std::cout << "[Path]\n";
		std::cout << "Length: " << length << "\n";
		std::cout << "Weight: " << minimum_weight << "\n";
		std::cout << "Score:  " << get_score() << "\n";

		std::cout << "<Opcodes>\n";
		for (auto &&opcode : opcodes)
		{
			const char* opcode_string = "UNKWN ";
			if(opcode < Opcode::mappingStr.size()){
				opcode_string = Opcode::mappingStr[opcode];
			}
			std::cout << opcode_string << " --> ";

		}

		std::cout << std::endl;


		std::cout << "Last Path Hash: " << path_hashes.back() << "\n";
		// std::cout << "<Path Hashes>\n";
		// for (auto &&hash : path_hashes)	
		// {
		// 	std::cout << hash << " --> ";

		// }
		std::cout << "\n\n";
	}
};

class InstructionNode{
	public:
		InstructionNode(){

		}

		InstructionNode(Opcode::Mapping instruction, uint64_t parent_hash)
				: instruction(instruction), weight(1){
					subtree_hash = (parent_hash << 6) + instruction;
					for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)//TODO remove should already be 0 initialized
					{
						dependencies[i] = false;
					}
					
		}

		Opcode::Mapping instruction;
		//the number of times this node occurred
		uint64_t weight;

		uint64_t total_cycles;
		//sum of the step ids this node occurred in
		//dividing by weight results in the average region of the programs lifetime this node occurs most frequently
		uint64_t sum_step_ids;
		//array of negative offsets to last node that writes to rs1 or rs2 (1=this node depends on tree[current-index])
		//very likely this only marks 2 values for longer (unique) paths
		std::array<bool, INSTRUCTION_TREE_DEPTH> dependencies; //value at offset 0 is ignored

		uint64_t subtree_hash = 0;

		virtual InstructionNode* insert(Opcode::Mapping op, uint64_t pc, 
								int8_t dependency1, int8_t dependency2, int8_t relative_overwrite, 
								uint8_t depth) = 0;

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
			if(dependencies[i]){
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
			if(dependencies[i]){
				result += 1/(double)i; //i should never be 0 as a node does not depend on itself
			}
			}

				return result;
				break;
			}
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
		void update_weight(int8_t dependency1, int8_t dependency2,  int8_t overwrite, uint64_t pc){
			weight++; 
			sum_step_ids += 1; //TODO add curent step
			if(dependency1>0){
				dependencies[dependency1] = true;
			}
			if(dependency2>0){
				dependencies[dependency2] = true;
			}
		}

		//called recursively for children
		virtual Path extend_path(uint32_t length, float score_bonus, float score_multiplier, uint32_t tree_id){
			Path max_path;
			max_path.length = length;
			max_path.minimum_weight = weight;

			//score multiplier for this singular node
			//we can't simply pass its weight * mult as score as the weight changes when extending the path  
			//instead track a bonus multiplier that adds mult * min_weight to the score 
			//can be negative
			float score_bonus_of_this_node = get_score_bonus();
			//global score multiplier of this node
			//including branches etc. should reduce the score of the whole path, so its not chosen
			float score_multiplier_of_this_node = get_score_multiplier();

			max_path.score_bonus = score_bonus + score_bonus_of_this_node;
			max_path.score_multiplier = 
			score_multiplier * score_multiplier_of_this_node; 
			max_path.inverse_dependency_score = get_inv_dep_score();

			max_path.opcodes.push_back(instruction);
			max_path.path_hashes.push_back(subtree_hash);

			return max_path;

		}

};

class InstructionNodeR : virtual public InstructionNode{
	public:
		InstructionNodeR(Opcode::Mapping instruction, uint64_t parent_hash);

		std::list<InstructionNode*> children;

		void insert_rb(std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_instructions, 
						uint8_t next_rb_index);

		InstructionNode* insert(Opcode::Mapping op, uint64_t pc, 
								int8_t dependency1, int8_t dependency2, int8_t relative_overwrite, 
								uint8_t depth);
		void _print(uint8_t level);

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold);
		Path extend_path(uint32_t length, float score_bonus, float score_multiplier, uint32_t tree_id);
};

class InstructionNodeLeaf : virtual public InstructionNode{
	public:
		InstructionNodeLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc);

	std::map<uint64_t, int> pc_map;

	InstructionNode* insert(Opcode::Mapping op, uint64_t pc, 
								int8_t dependency1, int8_t dependency2, int8_t relative_overwrite, 
								uint8_t depth);

	void _print(uint8_t level);

	std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold);
};

class MemoryNode{
	public: 
		bool is_store = false;
		Opcode::MemoryRegion memory_location = Opcode::MemoryRegion::NONE;//Stack(current frame=1 else 2) or Heap (4) or both (3,5,6,7)
		std::map<uint64_t, uint64_t> memory_accesses;
		uint64_t last_access = 0;
		uint64_t access_offset_sum = 0;

		MemoryNode(){};
		MemoryNode(bool is_store_instruction);

		void register_access(uint64_t address, uint64_t prev_writer, uint64_t prev_reader, 
								uint64_t stackpointer, uint64_t framepointer);

};

class BranchNode{
	public: 
		std::map<uint64_t, uint64_t> relative_offsets;
		bool is_backward_jump = false;
		bool is_forward_jump = false;

		BranchNode(){};
		BranchNode(int64_t offset);

		void register_access(uint64_t address, uint64_t prev_writer, uint64_t prev_reader, 
								uint64_t stackpointer, uint64_t framepointer);

};

class InstructionNodeMemory : public InstructionNodeR, virtual public MemoryNode{
	public: 
		InstructionNodeMemory(Opcode::Mapping instruction, uint64_t parent_hash, 
								uint64_t memory_access, bool is_store_instruction);

		void _print(uint8_t level);

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold);

};

class InstructionNodeMemoryLeaf : public InstructionNodeLeaf, virtual public MemoryNode{
	public: 
		InstructionNodeMemoryLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc, 
									uint64_t memory_access, bool is_store_instruction);


		void _print(uint8_t level);

		std::stringstream to_dot(const char* tree_op_name, const char* parent_name,
									uint depth, uint id, uint64_t parent_hash, 
									std::stringstream& dot_stream,  std::stringstream& connections_stream,
									uint64_t tree_weight, uint64_t total_instructions, 
									bool reduce_graph_output, float branch_omission_threshold);

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
};

#endif  // RISCV_ISA_TRACE_H
