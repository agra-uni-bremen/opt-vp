#include "trace.h"

#include <bitset>
#include "lib/json/single_include/nlohmann/json.hpp"

using namespace Opcode;

	int8_t register_dependencies_true[32] = {-1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1};

	// std::set<uint8_t> register_dependencies_output[32] = 
	// 								  {{},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{}};

	// std::set<uint8_t> register_dependencies_anti[32] = 
	// 								  {{},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{},
	// 								   {},{},{},{},{},{},{},{}};

	std::array<std::bitset<INSTRUCTION_TREE_DEPTH>, 32> register_dependencies_output;

	std::array<std::bitset<INSTRUCTION_TREE_DEPTH>, 32> register_dependencies_anti;
	// std::bitset<32> tmp_register_inputs;
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> memory_load;
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> memory_store;
	bool load_store_dirty = true; //fill with default during first iteration

namespace {
#if INSTRUCTION_TREE_DEPTH <= 64
	//! Reverse the bit order of a word. Seven masked shifts and a byte swap.
	inline uint64_t reverse_bits(uint64_t value){
		value = ((value >> 1) & 0x5555555555555555ull) | ((value & 0x5555555555555555ull) << 1);
		value = ((value >> 2) & 0x3333333333333333ull) | ((value & 0x3333333333333333ull) << 2);
		value = ((value >> 4) & 0x0F0F0F0F0F0F0F0Full) | ((value & 0x0F0F0F0F0F0F0F0Full) << 4);
		return __builtin_bswap64(value);
	}
#endif

	/**
	 * Turn positions in the window into distances back from position `current`.
	 *
	 * `positions` has bit j set for the instruction at position j of the window. A node records
	 * how far back a dependency reaches, so it needs bit `current - j` instead. Reversing the
	 * bit order produces every one of them at once, where walking the positions one at a time is
	 * what made insert_rb grow with the square of the trace depth.
	 *
	 * Position `current` itself is dropped: an instruction does not depend on itself.
	 */
	inline std::bitset<INSTRUCTION_TREE_DEPTH> positions_to_offsets(
			const std::bitset<INSTRUCTION_TREE_DEPTH>& positions, uint32_t current){
#if INSTRUCTION_TREE_DEPTH <= 64
		const uint64_t earlier = positions.to_ullong() & ((1ull << current) - 1);
		return std::bitset<INSTRUCTION_TREE_DEPTH>(reverse_bits(earlier) >> (63 - current));
#else
		//more than one word per mask, so walk the positions
		std::bitset<INSTRUCTION_TREE_DEPTH> offsets;
		for (uint32_t position = 0; position < current; position++) {
			if(positions[position]){
				offsets.set(current - position, true);
			}
		}
		return offsets;
#endif
	}
}

uint32_t trace_depth = INSTRUCTION_TREE_DEPTH;

void set_trace_depth(uint32_t depth){
	if(depth == 0){
		return; //keep the compiled default
	}
	if(depth > INSTRUCTION_TREE_DEPTH){
		printf("[WARNING] requested trace depth %u exceeds the compiled maximum of %u. "
				"Rebuild with -DINSTRUCTION_TREE_DEPTH=%u to use it. Using %u instead.\n",
				depth, (uint32_t)INSTRUCTION_TREE_DEPTH, depth, (uint32_t)INSTRUCTION_TREE_DEPTH);
		depth = INSTRUCTION_TREE_DEPTH;
	}
	if(depth < 2){
		printf("[WARNING] trace depth %u is too small (a tree needs at least a root and one child). Using 2 instead.\n", depth);
		depth = 2;
	}
	trace_depth = depth;
}

void InstructionNode::insert_rb(
				const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_steps_p, 
				uint32_t next_rb_index){
					insert_rb(last_executed_steps_p, next_rb_index, 0);
				}
void InstructionNode::insert_rb(
				const std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH>& last_executed_steps_p, 
				uint32_t next_rb_index, uint32_t offset){
	//keep the runtime depth in a local: it can not change while a sequence is inserted, but the
	//compiler has to assume the global could be written by any of the calls in the loop below
	const uint32_t depth = trace_depth;
	//printf("insert instructions from ringbuffer with len %ld\n", last_executed_instructions.size());
	
	//insert each element of the ringbuffer in order into the tree
	InstructionNode *current_node = this;

	// InstructionNode *inserted_nodes[INSTRUCTION_TREE_DEPTH] = {};
	// inserted_nodes[0] = this;

	//while the root node can not depend on any following node, we still need to call update weight
	//otherwise step id is not updated (but this is probably irrelevant for the root node)  
	//weight++;
	//this is now handled at the end of this function 

	//uint8_t rb_index_start = (next_rb_index+1)%INSTRUCTION_TREE_DEPTH;//next_rb_index is this op


	//reset dependency matrices
	for (size_t i = 0; i < 32; i++)
	{
		register_dependencies_true[i] = -1;
		register_dependencies_output[i].reset();
		register_dependencies_anti[i].reset();
	}

	if(load_store_dirty){
		for(size_t i = 0; i < depth; i++) {
			memory_load[i] = 0;
			memory_store[i] = 0;
			load_store_dirty = false;
		}
	}
	
	
	std::bitset<INSTRUCTION_TREE_DEPTH> anti_dependencies;
	std::bitset<INSTRUCTION_TREE_DEPTH> output_dependencies;
	
	//std::bitset<INSTRUCTION_TREE_DEPTH> memory_anti_dependencies;
	//std::bitset<INSTRUCTION_TREE_DEPTH> memory_output_dependencies;

	#ifdef debug_dependencies
	printf("---------------\nChecking dependencies\n---------------\n");
	#endif

	//walk the ring buffer from the oldest entry, wrapping around by subtraction instead of a
	//division: the index is always < 2*depth here
	uint32_t rb_index = next_rb_index;
	for (uint32_t i = 0; i < depth-offset; i++)//update the root node and insert all other nodes
	{
		const ExecutionInfo* current_step = &last_executed_steps_p[rb_index];
		if(++rb_index >= depth){
			rb_index = 0;
		}
		if(current_step->last_executed_instruction==Opcode::UNDEF){
			printf("[WARNING] trying to insert zero opcode into tree at index %d with offset %d\n", i, offset);
		}

		std::tuple<uint16_t,uint16_t,uint16_t> regs = current_step->last_registers;

		uint8_t rs1 = std::get<0>(regs);
		uint8_t rs2 = std::get<1>(regs);
		//uint8_t rs3 = std::get<0>(regs[rb_index]);
		uint8_t rd = std::get<2>(regs);
		
		int8_t tmp_true_dependency1 = -1;
		int8_t tmp_true_dependency2 = -1;

		[[maybe_unused]] int8_t memory_true_dependency = -1;
		[[maybe_unused]] int8_t memory_output_dependency = -1;
		[[maybe_unused]] int8_t memory_anti_dependency = -1;

		uint64_t l_read; 
		uint64_t l_store;

		int8_t tmp_input1 = -1;
		int8_t tmp_input2 = -1;
		int8_t tmp_output = -1;
		// int8_t tmp_input3 = -1;
		// tmp_register_inputs.reset();

		anti_dependencies.reset();
		output_dependencies.reset();
		//memory_anti_dependencies.reset();
		//memory_output_dependencies.reset();

		//calculate true register dependencies
		if(register_dependencies_true[rs1] > -1){ //maybe use >0 as instruction don't depend on zero reg
			if(i>0){//we don't need to check the root for dependencies
				tmp_true_dependency1 = i - register_dependencies_true[rs1]; //undef for values <0
			}
		}else{
			tmp_input1 = rs1;
			// tmp_register_inputs.set(rs1, true);//registers required as input for this sequence
		}
		if(register_dependencies_true[rs2] > -1){ 
			if(i>0){
				tmp_true_dependency2 = i - register_dependencies_true[rs2];
			}
		}else{
			tmp_input2 = rs2;
		}
		//int8_t true_dependency3 = -1; //TODO support R4 Fused Multiply Instructions


		Type type = getType(current_step->last_executed_instruction);
		if(type != Type::S){ //Store instructions don't use rd
			//Anti and output dependencies of this instruction: every earlier position of the
			//window that read rd, and every earlier position that wrote it. The node stores the
			//distance back rather than the position, which is what positions_to_offsets does.
			anti_dependencies |= positions_to_offsets(register_dependencies_anti[rd], i);
			output_dependencies |= positions_to_offsets(register_dependencies_output[rd], i);
		}

		//update access arrays after checking for dependencies
		//Load Store are an exception as dependencies are checked here to avid an extra type check

		#ifdef debug_dependencies
		printf("[%s]--\n", Opcode::mappingStr[current_step->last_executed_instruction]);
		#endif
		switch (type)
		{
		case Type::R :
			//rs1, rs2, rd
			register_dependencies_true[rd] = i;
			register_dependencies_output[rd].set(i, true);
			tmp_output = rd;

			register_dependencies_anti[rs1].set(i, true);
			register_dependencies_anti[rs2].set(i, true);
			break;

		case Type::I :
			/* rs1, rd */
			tmp_true_dependency2 = -1; //reset as rs2 is not used (unknown value in variable)
			rs2 = -1;
			tmp_input2 = -1;
			register_dependencies_true[rd] = i;
			register_dependencies_output[rd].set(i, true);
			tmp_output = rd;

			register_dependencies_anti[rs1].set(i, true);

			l_read = current_step->last_memory_read; 
			if(l_read){
				#ifdef debug_dependencies
				printf("  Reads memory: %lx\n", l_read);
				#endif
				load_store_dirty = true;
				// printf("read %s\n", Opcode::mappingStr[current_step->last_executed_instruction]);
				memory_load[i] = l_read;
				for (int j = i-1; j >= 0; j--)
				{
					if(memory_store[j] == l_read){
						//found Store instruction that accesses the same address
						memory_true_dependency = i-j; //TODO check again if this results in the correct index
						tmp_true_dependency2 = i-j; //just use default true_dependency for now
						#ifdef debug_dependencies
						printf("  found memory true dependency with offset -%d\n", i-j);
						#endif
						break;
					}
				}
					//we do not need to check for previous Load instructions 
					//as they do not interfere using memory and we already handle registers
					// if(memory_load[j] == l_read){
					// 	//found Load instruction that accesses the same address
					// 	break;
					// }
			}
			break;

		case Type::S :
			/* rs1, rs2, memory TODO*/
			// Store Instructions

			l_read = current_step->last_memory_read; 
			if(l_read != 0){
				printf("[ERROR] Found memory read for store instruction: %lx\n", l_read);
			}
			l_store = current_step->last_memory_written;
			#ifdef debug_dependencies
			printf("  Writes memory: %lx\n", l_store);
			#endif
			//first check for dependencies
			//if address !=0 (either store or load is always 0 as this identifies the access type)
			//start at index-1 and traverse backwards until identical address is found or end is reached
			// if(l_read>0){
			// 	memory_load[i] = l_read;
			// 	for (int j = i-1; j >= 0; j--)
			// 	{
			// 		if(memory_store[j] == l_read){
			// 			//found Store instruction that accesses the same address
			// 			memory_true_dependency = i-j; //TODO check again if this results in the correct index
			// 			printf("found memory true dependency with idx %d\n", i-j);
			// 			break;
			// 		}
			// 	}
			// }else{
				//if l_store>0
				//check stores
				if(l_store==0){
					//TODO should never be 0
					printf("[ERROR] memory operation without access\n");
				}else{
					memory_store[i] = l_store;
					for (int j = i-1; j >= 0; j--)
					{
						if(memory_store[j] == l_store){
							//found Store instruction that accesses the same address
							memory_output_dependency = i-j;
							output_dependencies.set(i - j,true);
							#ifdef debug_dependencies
							printf("  found memory output dependency with offset -%d\n", i-j);
							#endif
							break;
						}
					}
					//check loads
					for (int j = i-1; j >= 0; j--)
					{
						if(memory_load[j] == l_store){
							//found Load instruction that accesses the same address
							memory_anti_dependency = i-j;
							anti_dependencies.set(i - j,true);
							#ifdef debug_dependencies
							printf("  found memory anti dependency with offset -%d\n", i-j);
							#endif
							break;
						}
					}
				}
			// }
			
			

			//update arrays with accessed registers and memory
			register_dependencies_anti[rs1].set(i, true);
			register_dependencies_anti[rs2].set(i, true);
			load_store_dirty = true;
			break;

		case Type::U :
			/* rd */
			tmp_true_dependency1 = -1;
			tmp_true_dependency2 = -1;
			rs1 = -1;
			rs2 = -1;
			tmp_input1 = -1;
			tmp_input2 = -1;
			register_dependencies_true[rd] = i;
			register_dependencies_output[rd].set(i, true);
			tmp_output = rd;
			break;

		case Type::R4 :
			/* code */
			break;

		case Type::J :
			/* rd */
			tmp_true_dependency1 = -1;
			tmp_true_dependency2 = -1;
			rs1 = -1;
			rs2 = -1;
			tmp_input1 = -1;
			tmp_input2 = -1;
			register_dependencies_true[rd] = i;
			register_dependencies_output[rd].set(i, true);
			tmp_output = rd;
			break;

		case Type::B :
			/* Branch rs1, rs2 TODO add condition? */

			register_dependencies_anti[rs1].set(i, true);
			register_dependencies_anti[rs2].set(i, true);
			break;
		
		default:
			break;
		}

		register_dependencies_true[0] = -1; //reset zero register in case an instruction wrote to it

		//printf("Last regs:%d, %d -> %d\n",rs1, rs2, rd);
		//#define debug_register_dependencies
		#ifdef debug_register_dependencies
		// if(i==INSTRUCTION_TREE_DEPTH-2){
			for (int8_t j = 0; j < 32; j++)
			{
				int8_t val = register_dependencies_true[j];
				
				int8_t relative_val = (i - val + trace_depth)%trace_depth; //undef for values <0
				uint8_t color_fg = 249; //rs1+16;
				uint8_t color_bg = (232 + relative_val*23/(trace_depth-2))%256;
				
				if(j == tmp_true_dependency1){
										//color group + start offset + group rb_index
					color_fg = ((tmp_true_dependency1%30)/6)*36+16 + (tmp_true_dependency1%10)*6;
					color_bg = color_bg-180;
				}
				if(j == tmp_true_dependency2){
					color_fg = ((tmp_true_dependency2%30)/6)*36+21 + (tmp_true_dependency2%10)*6;
					color_bg = color_bg-216;
				}
				std::cout << "[" ;
				if(val>=0){
					if(relative_val<10){
						std::cout << ' ';
					}
					std::cout << "\033[38;5;" << +color_fg << "m\033[48;5;" << +color_bg << "m";
					std::cout << +relative_val << "\033[0m";
				}else{
						std::cout << "  " ;
				}
				std::cout << "]" ;
			}
			std::cout << '\r' << std::flush;
		// }
		#endif
		AccessType access_type = current_step->last_memory_access_type;
		uint64_t last_memory_access = -1;
			if(access_type==AccessType::STORE){
				last_memory_access = current_step->last_memory_written;
			}
			if (access_type==AccessType::LOAD){
				last_memory_access = current_step->last_memory_read;
		}			

		const StepInfo step = {
								current_step->last_executed_instruction,
								current_step->last_executed_pc,
								tmp_true_dependency1, tmp_true_dependency2,
								output_dependencies,
								anti_dependencies,
								rs1, rs2, rd,
								tmp_input1, tmp_input2, tmp_output,
								i,
								current_step->last_step_id,
								current_step->last_cycles,
								last_memory_access,
								access_type,
								current_step->last_stack_pointer,
								current_step->last_frame_pointer,
								current_step->last_parameter,
								current_step->last_peripheral_name,
								current_step->last_predecessor_pc,
								current_step->last_branch_outcome,
								current_step->last_branch_offset
								};

		if(i>0){//the root node already exists and is current_node
			current_node = current_node->insert(step);
		}else{
			//the root node can not depend on a following instruction, and the loop above leaves
			//the dependency fields empty for i == 0, so the same struct describes it
			update_weight(step);
		}
	}
}

namespace {
	//! Mnemonic of an opcode, or a marker when the value is out of range.
	const char* opcode_name(Opcode::Mapping op){
		return op < Opcode::mappingStr.size() ? Opcode::mappingStr[op] : "UNKWN ";
	}

	//! Colour of a node in the dot export: hue by depth, full saturation and value.
	float dot_hue(uint depth){
		return (float)depth/(float)trace_depth;
	}
}

InstructionNode* InstructionNode::create(Opcode::Mapping op, uint64_t parent_hash, bool leaf){
	const NODE_TYPE type = node_type_for(op, leaf);
	size_t payload_size = 0;
	if((uint8_t)type & (uint8_t)NODE_TYPE::MEMORY){
		payload_size = sizeof(MemoryNode);
	}else if((uint8_t)type & (uint8_t)NODE_TYPE::BRANCH){
		payload_size = sizeof(BranchNode);
	}

	//one block for the node and its payload: recording then reaches the payload without
	//following a pointer, and a node without one costs nothing for it
	void* block = ::operator new(sizeof(InstructionNode) + payload_size);
	InstructionNode* node = new (block) InstructionNode(op, parent_hash, type);
	if(node->has_memory()){
		new (node->payload_slot()) MemoryNode(memory_opcode(op) == MemoryOpcode::STORE);
	}else if(node->has_branch()){
		new (node->payload_slot()) BranchNode();
	}
	return node;
}

InstructionNode* InstructionNode::insert(const StepInfo& p){
	InstructionNode* found_child = nullptr;
	for (const Child& child : children){
		if(child.op == p.op){
			found_child = child.node;
			break;
		}
	}
	if(found_child==nullptr){
		//a step at the last position of a window ends it, so its node is a leaf
		found_child = create(p.op, subtree_hash, p.depth >= trace_depth-1);
		children.push_back({p.op, found_child});
	}

	found_child->update_weight(p);
	return found_child;
}

void InstructionNode::update_weight(const StepInfo& p){
	weight++;
	total_cycles += p.cycles;

	// Update true_weight only if this window does not overlap the last counted one.
	// p.step is the step id of the window's last instruction and the window is
	// depth+1 instructions long, so it must start after the last counted one ended.
	// The first occurrence is always counted: it can end at step id depth, which the
	// comparison alone would reject.
	if (true_weight == 0 || (last_occurrence + p.depth) < p.step) {
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
		#ifdef handle_self_modifying_code
		//the same pc running with other registers means the instruction at that address changed
		const RegisterSet& seen = it->second.regset;
		if(seen.rs1 != (int8_t)p.rs1 || seen.rs2 != (int8_t)p.rs2 || seen.rd != (int8_t)p.rd){
			printf("detected binary modification at pc %lx\n", p.pc);
		}
		#endif
	}
	#ifdef trace_predecessor_pcs
	//which pc this occurrence was actually reached from, so later, a pc path can be proven instead of guessed
	it->second.count_predecessor(p.predecessor_pc);
	#endif
	#ifdef trace_parameter
	//record the value the ISS decoded for this instruction: shift amount, branch/jump target or,
	//with trace_parameter_immediates, the decoded immediate. Values may be negative.
	if (p.parameter != NO_PARAMETER
			#ifndef trace_root_parameters
			//tracking the root node costs one entry per pc executing this opcode with little benefit
			&& p.depth > 0
			#endif
		) {
		it->second.count_parameter(p.parameter);
	}
	#endif
	#endif

	if(p.true_dependency1>0){
		dependencies_true_[p.true_dependency1] = true;
	}else if(p.input1>=0){
		inputs_.set(p.input1, true);
	}
	if(p.true_dependency2>0){
		dependencies_true_[p.true_dependency2] = true;
	}else if(p.input2>=0){
		inputs_.set(p.input2, true);
	}
	if(p.output > 0){//ignore outputs for zero reg
		outputs_.set(p.output, true);
	}
	dependencies_anti_ |= p.anti_dependencies;
	dependencies_output_ |= p.output_dependencies;

	//the payload of a load/store or of a branch. A node never has both.
	if(has_memory()){
		memory()->register_access(p.pc, p.memory_address, p.access_type, 0,
								p.stack_pointer, p.frame_pointer, p.peripheral_name);
	}else if(has_branch() && p.branch_outcome!=BranchOutcome::NONE){
		//JALR jumps relative to rs1 rather than to the pc, so its offset must not reach the
		//pc relative direction and offset histogram
		branch()->register_branch(p.pc, p.branch_outcome, p.branch_offset, instruction!=Opcode::JALR);
	}
}

uint64_t InstructionNode::max_pc_count() const {
	uint64_t largest = get_pc().size();
	for (const Child& child : children) {
		largest = std::max(largest, child.node->max_pc_count());
	}
	return largest;
}

std::map<uint64_t, int> InstructionNode::get_pc() const {
	std::map<uint64_t, int> pcs;
	#if defined(trace_pcs) && defined(trace_individual_registers)
	for (const auto& entry : register_sets) {
		pcs.emplace(entry.first, entry.second.count);
	}
	#endif
	return pcs;
}

float InstructionNode::get_score_bonus() const {
	if(is_branch_opcode(instruction)){
		//a branch or a jump inside a sequence makes the sequence worth nothing, see
		//get_score_multiplier. At the root of its own tree it is only penalised.
		return -1.0;
	}
#ifndef dependency_score
	return 0.0;
#else
	//reward an instruction that depends on nothing in the window, penalise a chain
	uint64_t dependencies_count = true_dependency_count();
	if(dependencies_count>0){
		return -(float)dependencies_count/10.0;
	}
	return 2.0;
#endif
}

float InstructionNode::get_score_multiplier() const {
	if(has_branch()){
		//zero, so no sequence that contains a branch or a jump can win. The same opcode at the
		//root of its tree has no branch payload and keeps a multiplier of one.
		return 0.0;
	}
	return 1.0;
}

double InstructionNode::get_inv_dep_score() const {
	using namespace Opcode;
	switch (instruction)
	{
	case BEQ:
	case BNE:
	case BLT:
	case BLTU:
	case BGE:
	case BGEU:
		return 0.0;
	default:
		break;
	}

	//closer dependencies weigh more: an instruction reading the value its parent wrote can not
	//be reordered, one reading a value from ten instructions back nearly always can
	double result = 0.0;
	for (size_t i = 1; i < trace_depth; i++)
	{
		if(depends_true_on(i)){
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
}

void InstructionNode::print(){
	std::cout << "--------\n";
	std::cout << "](" << opcode_name(instruction) << ")[\n";
	std::cout << "--" << weight << "--\n";
	_print(1);
}

void InstructionNode::_print(uint8_t level){
	for (int i = 1; i < level; i++) {
		std::cout << "\t";
	}
	std::cout << "[" << opcode_name(instruction) << "(" << weight << ")]" << std::endl;
	for (const Child& child : children) {
		child.node->_print(level + 1);
	}
}

void InstructionNode::tree_to_dot(uint64_t total_instructions, float branch_threshold){
	std::stringstream dot_stream; 
	std::stringstream connections_stream; 

	//the digraph header is written by the dot exporter, which knows the file
	dot_stream << "//Nodes" << std::endl;
	connections_stream << "//Connections" << std::endl;

	to_dot(opcode_name(instruction), "", 0, 0, 0, 
		dot_stream, connections_stream,
		weight, total_instructions, 
		branch_threshold > 0.0f, branch_threshold);

	dot_stream << connections_stream.str();

	std::cout << dot_stream.str() << std::endl;
}

std::stringstream InstructionNode::to_dot(const char* tree_op_name, const char* parent_name,
							uint depth, uint id, uint64_t parent_hash, 
							std::stringstream& dot_stream,  std::stringstream& connections_stream,
							uint64_t tree_weight, uint64_t total_instructions, 
							bool reduce_graph_output, float branch_omission_threshold){
	const char* label = opcode_name(instruction);
	std::stringstream name;

	if(depth==0){
		//the root is drawn as a record and carries its share of the whole program rather than
		//of its tree, which is what makes trees comparable to each other
		name << label;
		float per_weight = (float)weight/(float)total_instructions;
		std::stringstream top_label;
		top_label << "|{" << label << " | " << (float)(((int)(per_weight * 1000.0)) % 1000) / 10.0 
				<< " | " << weight << "/" << total_instructions << "}|";
		uint16_t color_index = std::min(11.0, (per_weight*2.0) * 10 + 0.1 + 1); //TODO is half of all instructions a good max?
		dot_stream << name.str()
				<< "[label=\"" << top_label.str() << "\", shape = record, color=" << color_index 
				<< ", colorscheme=spectral11" << "]" << std::endl;
	}else{
		//a leaf above the last level of the tree only exists because the tree was pruned
		const bool pruned = is_leaf() && depth < trace_depth-1;
		if(pruned){
			name << "pruned_";
		}
		name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << label;

		const float per_weight = (float)weight/(float)tree_weight;
		const uint16_t color_index = pruned ? 1 : (uint16_t)(per_weight * 8 + 0.1 + 1); //9 colors (1-9)

		dot_stream << name.str() 
			<< "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
			<< "<TR><TD><FONT COLOR=\"" << dot_hue(depth) << " 1 1\">" << label << "</FONT></TD></TR>";

		//true dependencies, coloured by the depth of the instruction the value comes from
		dot_stream << "<TR><TD>";
		for (size_t i = 1; i < trace_depth; i++)
		{
			if(depends_true_on(i)){
				dot_stream << "<FONT COLOR=\"" << dot_hue(depth-i) << " 1 1\">" 
						<< i << " \n" << "</FONT>";
			}
		}
		dot_stream << "</TD></TR>";

		dot_stream << "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
				<< std::hex << subtree_hash << std::dec << "</FONT></TD></TR>";

		if(has_memory()){
			dot_stream << "<TR><TD><FONT COLOR=\"0.2 0.8 1.000\" POINT-SIZE=\"10\">[";
			for (auto &&access : memory()->memory_accesses)
			{
				dot_stream << std::hex << access.first << ": {";
				for(auto &&pair : access.second){
					dot_stream << "(" << std::hex << (pair.first & 0xFFFF) << " - " << std::hex << (int)pair.second << "), ";
				}
				dot_stream << std::hex << access.first << "}" << std::dec;
			}
			dot_stream << std::dec << "]</FONT></TD></TR>";
		}

		if(is_leaf()){
			dot_stream << "<TR><TD><FONT COLOR=\"0.6 0.4 0.600\" POINT-SIZE=\"10\">" << std::hex;
			for (auto const& pc : get_pc())
			{
				dot_stream << pc.first << ":" << pc.second << " "; 
			}
			dot_stream << std::dec << "</FONT></TD></TR>";
		}

		if(pruned){
			dot_stream << "<TR><TD><FONT COLOR=\"0.8 0.0 0.1\" POINT-SIZE=\"14\"> pruned </FONT></TD></TR>";
		}

		dot_stream << "</TABLE>>, color=" << color_index << "]" << std::endl;

		connections_stream << parent_name << " -> " << name.str();
		connections_stream << "[label=\"" << weight << "\" decorate=true";

		if(!pruned && reduce_graph_output && per_weight<branch_omission_threshold){
			//draw the edge, but not the subtree behind it
			int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0)));
			connections_stream << " style=\"dashed\" color=\"gray" << shade
					<< "\" fontcolor=\"gray" << shade << "\"]" << std::endl;
			return name; 
		}
		connections_stream << "]" << std::endl;
	}

	uint child_index = 0;
	for (const Child& child : children) {
		child.node->to_dot(tree_op_name, name.str().c_str(), 
					depth + 1, child_index, subtree_hash, 
					dot_stream, connections_stream, 
					tree_weight, total_instructions,
					reduce_graph_output, branch_omission_threshold);
		child_index++;
	}

	return name;
}

nlohmann::ordered_json InstructionNode::to_json(){
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
			if (rsc.has_predecessors()) {
				nlohmann::json jsonPredecessors = nlohmann::json::object();
				for (const PcCounter& predecessor : rsc.predecessors_in_pc_order()) {
					jsonPredecessors[std::to_string(predecessor.pc)] = predecessor.count;
				}
				jsonEntry["predecessors"] = jsonPredecessors;
			}
			#endif
			jsonRegisterSets[std::to_string(key)] = jsonEntry;
		}
		jsonNode["register_sets"] = jsonRegisterSets;
	#endif 

	//dependencies as offsets back along the path, 1 being the parent
	std::vector<int> true_dependencies;
	std::set<int8_t> anti_dependencies;
	std::set<int8_t> output_dependencies;

	for (size_t i = 1; i < trace_depth; i++){
			if(depends_true_on(i)){
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

	//kept in the trace so the format does not move; see InstructionNode for why it is zeros
	jsonNode["occurrence"] = nlohmann::json::array({0, 0, 0, 0});

	#ifdef trace_parameter
	//[[pc, [[value, count], ...]], ...]
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

	if(is_leaf()){
		//a leaf is written with its keys sorted, which is what the unordered json type does
		nlohmann::json sorted = jsonNode;
		sorted["PCs"] = get_pc();
		nlohmann::ordered_json leaf_json = sorted;
		if(has_memory()){
			leaf_json.update(memory()->memory_to_json());
		}else if(has_branch()){
			leaf_json.update(branch()->branch_to_json());
		}
		return leaf_json;
	}

	nlohmann::ordered_json jsonChildren = nlohmann::ordered_json::array(); 
	for (const Child& child : children)
	{
		jsonChildren.push_back(child.node->to_json());
	}
	jsonNode["children"] = jsonChildren;

	if(has_memory()){
		jsonNode.update(memory()->memory_to_json());
	}else if(has_branch()){
		jsonNode.update(branch()->branch_to_json());
	}

	return jsonNode;
}

//might be easier to use a struct but this way its harder to miss a parameter
std::stringstream InstructionNode::csv_format(uint64_t parent_hash, const char* tree, const char* instruction_string,
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
			<< current_true_dep << ";"
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
			<< 0 << ";" //the four phase counters, always zero
			<< 0 << ";"
			<< 0 << ";"
			<< 0 << ";"
			<< number_of_pcs << ";"
			<< max_pcs - number_of_pcs << ";";

	return csv_stream;
}

void InstructionNode::to_csv(const CsvParams& p) {
	std::map<InstructionType, uint32_t> _instruction_types = p.instruction_types;

	double current_dep_score = get_inv_dep_score();
	double current_total_dep_score = p.last_dep_score + current_dep_score;

	uint32_t current_true_dep = true_dependency_count();
	uint32_t current_anti_dep = dependencies_anti_.count();
	uint32_t current_out_dep = dependencies_output_.count();

	uint32_t total_true_dep = p.true_dep +  current_true_dep;
	uint32_t total_anti_dep = p.anti_dep + current_anti_dep;
	uint32_t total_out_dep = p.out_dep + current_out_dep;

	std::bitset<32> current_total_inputs = p.total_inputs | inputs_;
	std::bitset<32> current_total_outputs = p.total_outputs | outputs_;

	uint64_t number_of_pcs = get_pc().size();
	_instruction_types[getInstructionType(instruction)]++;

	std::stringstream csv_stream = csv_format(p.parent_hash, p.tree, opcode_name(instruction),
							p.last_weight, p.max_weight, p.total_max_weight, 
							p.depth, current_dep_score, current_total_dep_score, 
							current_true_dep, current_anti_dep, current_out_dep, 
							total_true_dep, total_anti_dep, total_out_dep, 
							children.size(), current_total_inputs.count(), 
							current_total_outputs.count(), _instruction_types[InstructionType::Branch], 
							number_of_pcs, p.max_pcs);
	std::cout << csv_stream.str() << std::endl;

	for (const Child& child : children)
	{
		child.node->to_csv({
			p.total_instructions,
			p.tree,
			p.depth + 1,
			current_dep_score,
			total_true_dep,
			total_anti_dep,
			total_out_dep,
			current_total_inputs,
			current_total_outputs,
			_instruction_types,
			subtree_hash,
			p.max_weight,
			weight,//last weight
			p.total_max_weight,
			p.max_pcs
		});
	}
}

namespace {
	static Path build_node_path(InstructionNode& node, const PathExtensionParams& p) {
		Path path;
		path.length = p.length;
		path.minimum_weight = node.weight;
		path.true_weight = node.true_weight;
		path.score_bonus = p.score_bonus + node.get_score_bonus();
		path.score_multiplier = p.score_multiplier * node.get_score_multiplier();
		path.inverse_dependency_score = node.get_inv_dep_score();
		path.opcodes.push_back(node.instruction);
		path.path_hashes.push_back(node.subtree_hash);
		path.end_of_sequence = &node;
		return path;
	}

	static void prepend_node(Path& path, InstructionNode& node) {
		path.opcodes.insert(path.opcodes.begin(), node.instruction);
		path.path_hashes.insert(path.path_hashes.begin(), node.subtree_hash);
		path.inverse_dependency_score += node.get_inv_dep_score();
		path.minimum_weight = std::min(path.minimum_weight, node.weight);
	}

	static std::vector<Path> sort_and_trim_paths(std::vector<Path> paths,
											 const std::function<float(ScoreParams)>& score_function,
											 size_t top_k) {
		if (top_k == 0)
			return {};

		std::sort(paths.begin(), paths.end(),
			[&score_function](const Path& a, const Path& b) -> bool {
				return a.get_score(score_function) > b.get_score(score_function);
			});

		if (paths.size() > top_k)
			paths.resize(top_k);

		return paths;
	}
}

std::vector<Path> InstructionNode::extend_top_paths(const PathExtensionParams& p, size_t top_k){
	if (top_k == 0)
		return {};

	std::vector<Path> candidates;
	candidates.push_back(build_node_path(*this, p));

	for (const Child& child : children) {
		auto child_candidates = child.node->extend_top_paths({p.length + 1,
				p.score_bonus + get_score_bonus(),
				p.score_multiplier * get_score_multiplier(),
				p.tree_id,
				p.force_extension_depth,
				p.force_instruction,
				p.score_function},
			top_k);

		for (auto child_path : child_candidates) {
			prepend_node(child_path, *this);
			candidates.push_back(std::move(child_path));
		}
	}

	return sort_and_trim_paths(std::move(candidates), p.score_function, top_k);
}

//called recursively for children
Path InstructionNode::extend_path(const PathExtensionParams& p){
	
	//score multiplier for this singular node
	//we can't simply pass its weight * mult as score as the weight changes when extending the path  
	//instead track a bonus multiplier that adds mult * min_weight to the score 
	//can be negative
	float score_bonus_of_this_node = get_score_bonus();
	//global score multiplier of this node
	//including branches etc. should reduce the score of the whole path, so its not chosen
	float score_multiplier_of_this_node = get_score_multiplier();

	Path max_path;
	max_path.length = p.length; //was increased on call
	max_path.minimum_weight = weight; //weight of a child should always be smaller or equal
	max_path.true_weight = true_weight;
	max_path.score_bonus = p.score_bonus + score_bonus_of_this_node;
	max_path.score_multiplier = 
			p.score_multiplier * score_multiplier_of_this_node; 
	max_path.inverse_dependency_score = get_inv_dep_score();
	
	//(forward branch and backward branch out of scope have score multiplier = 0)
	max_path.opcodes.push_back(instruction);
	max_path.path_hashes.push_back(subtree_hash);

	max_path.end_of_sequence = this;

	Path max_child_path;
	Path child_path;
	for (const Child& child : children) {
		child_path = child.node->extend_path({p.length+1, 
				max_path.score_bonus, max_path.score_multiplier, p.tree_id, p.force_extension_depth, 
				p.force_instruction, p.score_function});
		if(max_child_path.length==0){ //should not happen with force_extension
			max_child_path = child_path;
		}else{
			if(static_cast<uint32_t>(p.force_extension_depth+1) == p.length && p.force_instruction == child.op){
				//force_extension is -1 if not forcing extension
				//if instruction == child->instruction
				//set child_path as new max_child without checking score and break
				max_child_path = child_path;
				break;
			}
			if(child_path.get_score(p.score_function) > max_child_path.get_score(p.score_function)){
				max_child_path = child_path;
			}
		}
	}

	if (max_child_path.length>0)
	{
		if((max_path.get_score(p.score_function) < max_child_path.get_score(p.score_function)) || static_cast<uint32_t>(p.force_extension_depth+1)==p.length){
			max_path.length = max_child_path.length;
			max_path.minimum_weight = max_child_path.minimum_weight;
			max_path.true_weight = max_child_path.true_weight;
			max_path.opcodes.insert(max_path.opcodes.end(), 
							max_child_path.opcodes.begin(), 
							max_child_path.opcodes.end());
			max_path.path_hashes.insert(max_path.path_hashes.end(), 
				max_child_path.path_hashes.begin(), 
				max_child_path.path_hashes.end());
			max_path.inverse_dependency_score += max_child_path.inverse_dependency_score;
			max_path.end_of_sequence = max_child_path.end_of_sequence;
		} 
	}

	return max_path;
}

std::vector<Path> InstructionNode::force_path_extension(const Path p, std::function <float(ScoreParams)> score_function){
	std::vector<Path> extended_sequences;
	if(is_leaf()){
		printf("Warning: the sequence ends at the maximum tree depth and can not be extended\n"
				"Consider increasing the maximum tree depth\n");
		return extended_sequences;
	}

	//handle sequences ending in a branch/jump
	//force extend the path to the branch/jump if the node is the only child
	Path path_origin = p;
	if(children.size() == 1 && (getInstructionType(children.front().op) == InstructionType::Branch ||  getInstructionType(children.front().op) == InstructionType::Jump)){
		InstructionNode* branch_child = children.front().node;
		if(branch_child->is_leaf()){ //we already reached the end of the tree
			printf("Warning: only child of sequence is a branch, but no further nodes to extend the sequence exist\n");
			return extended_sequences;
		}
		path_origin.length += 1;
		assert(path_origin.minimum_weight == branch_child->weight);

		float score_bonus_of_next_node = branch_child->get_score_bonus();

		path_origin.score_bonus += score_bonus_of_next_node;
		path_origin.inverse_dependency_score = branch_child->get_inv_dep_score();//TODO check if this is correct
		
		//(forward branch and backward branch out of scope have score multiplier = 0)
		path_origin.opcodes.push_back(branch_child->instruction);
		path_origin.path_hashes.push_back(branch_child->subtree_hash);

		return branch_child->force_path_extension(path_origin, score_function);
	}
	for (const Child& child : children)
	{
		Path max_path = p;
		Path child_path = child.node->extend_path({p.length+1, 
				p.score_bonus, p.score_multiplier, -1, -1, Opcode::Mapping::UNDEF, score_function});
		max_path.length = child_path.length;
		max_path.minimum_weight = child_path.minimum_weight;
		max_path.opcodes.insert(max_path.opcodes.end(), 
						child_path.opcodes.begin(), 
						child_path.opcodes.end());
		max_path.path_hashes.insert(max_path.path_hashes.end(), 
			child_path.path_hashes.begin(), 
			child_path.path_hashes.end());
		max_path.inverse_dependency_score += child_path.inverse_dependency_score;
		max_path.end_of_sequence = child_path.end_of_sequence;

		extended_sequences.push_back(max_path);
	}
	return extended_sequences;
}

std::vector<PathNode> InstructionNode::path_to_path_nodes(Path path, uint depth){
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

	PathNode node(instruction, weight, get_score_bonus(), get_score_multiplier(), get_inv_dep_score(), 
					get_pc(), dependencies_true_, indices_out, indices_anti);
	if(has_memory()){
		node.extra_fields = memory()->memory_to_json();
	}
	nodes.push_back(node);

	if(is_leaf() || (depth+1)>=path.length){
		return nodes;
	}

	InstructionNode* found_child = nullptr;
	for (const Child& child : children){
		if(child.op == path.opcodes[depth+1]){
			found_child = child.node;
			break;
		}
	}
	if(found_child==nullptr){
		printf("[ERROR] no children in tree that match discovered path"); //should not be possible
		return nodes;
	}
	std::vector<PathNode> child_nodes = found_child->path_to_path_nodes(path, depth+1); 
	nodes.insert(nodes.end(), child_nodes.begin(), child_nodes.end());
	return nodes;
}

//find a point in an existing sequence with the highest ratio between branch taken in the original sequence 
// and another possible branch not taken, which would lead to a different sequence 
std::vector<BranchingPoint> InstructionNode::find_variant_branch(Path path, uint8_t depth){
	//find branching point
	//then create new Path up to this point and extend_path()

	std::vector<BranchingPoint> branching_points; 

	if(is_leaf() || (uint32_t)(depth+1)>=path.length){
		return branching_points;
	}

	InstructionNode* found_child = nullptr;
	int64_t current_max_weight = -1;
	double current_max_ratio = -1.0;
	Opcode::Mapping current_instruction = Opcode::UNDEF;
	for (const Child& child : children){
		if(child.op == path.opcodes[depth+1]){
			found_child = child.node; //if child is on previous best path, continue variant search with that node
		}else{
			double current_ratio = (double)child.node->weight / (double)weight; //otherwise save possible branching point
			if(current_ratio>current_max_ratio){
				current_max_weight = child.node->weight;
				current_max_ratio = current_ratio;
				current_instruction = child.op;
			}
		}
	}

	if(current_max_weight>0){
		BranchingPoint bp = {depth, current_instruction, current_max_weight, current_max_ratio, this}; 
		branching_points.push_back(bp);
	}

	if(found_child==nullptr){
		printf("[ERROR] no children in tree that match discovered path"); //should not be possible
		return branching_points;
	}
	std::vector<BranchingPoint> child_branching_points = found_child->find_variant_branch(path, depth+1); 
	branching_points.insert(branching_points.end(), child_branching_points.begin(), child_branching_points.end());
	return branching_points;
}

int InstructionNode::prune_tree(uint64_t weight_threshold, uint8_t depth){
	bool pruned = false;
	for (const Child& child : children) {
		if (child.node->weight < weight_threshold && !child.node->is_leaf())
		{
			if(!pruned){
				printf("\t\tpruned branch at depth: %d\n", depth);
				pruned = true;
			}
			//turn the child into a leaf: what it counted stays, the subtree below it goes
			child.node->node_type = (NODE_TYPE)((((uint8_t)child.node->node_type) & ~(uint8_t)NODE_TYPE::NODE) 
											| (uint8_t)NODE_TYPE::LEAF);
			child.node->children.clear();
		}
		
		child.node->prune_tree(weight_threshold, depth+1);
	}
	return 0;
}

MemoryNode::MemoryNode(bool is_store_instruction) : is_store(is_store_instruction){
}

void MemoryNode::register_access(uint64_t pc, uint64_t address, 
	AccessType access_type, uint64_t prev_access, 
				uint64_t stackpointer, uint64_t framepointer, const char* peripheral_name){
	//last_access is never updated, so this sums the addresses rather than the distances
	//between them. The exported OffsetSum field has always meant that.
	uint64_t access_offset = abs((long int)(address-last_access));
	access_offset_sum += access_offset;

	Opcode::MemoryRegion memory_location = Opcode::MemoryRegion::NONE;
	if(peripheral_name != nullptr){
		//address hit a registered peripheral region - classify as PERIPHERAL instead of the stack/heap/frame heuristic
		memory_location = memory_location | Opcode::MemoryRegion::PERIPHERAL;
		std::string name(peripheral_name);
		peripheral_by_address[address] = name;
		peripheral_access_counts[name]++;
	}else if(framepointer>0 && address <= framepointer && address >= stackpointer){
		//address is in Frame
		memory_location = memory_location | Opcode::MemoryRegion::FRAME;
	}else{
		if(address<stackpointer){
			//address is not on the Stack
			memory_location = memory_location | Opcode::MemoryRegion::HEAP;
		}else{
			//address is on the Stack but not in Frame
			memory_location = memory_location | Opcode::MemoryRegion::STACK;
		}
	}
	auto& access_entry = memory_accesses[pc];
	access_entry[address] = memory_location;
}

void BranchNode::register_branch(uint64_t pc, BranchOutcome outcome, int64_t offset, bool pc_relative){
	#ifdef trace_branch_outcomes
	BranchOutcomeCounter& counter = branch_outcomes[pc];
	counter.offset = offset;
	if(outcome==BranchOutcome::TAKEN){
		counter.taken++;
	}else{
		counter.not_taken++;
	}
	#endif

	if(outcome!=BranchOutcome::TAKEN || !pc_relative){
		return; //direction and offset histogram only describe branches that were taken relative to the pc
	}
	if(offset!=0){
		relative_offsets[offset]++;
		if(offset<0){
			is_backward_jump = true;
		}else{
			is_forward_jump = true;
		}
	}
}


PathNode::PathNode(Opcode::Mapping instr, uint64_t wt, float score_b, float score_m, float inv_d, 
					std::map<uint64_t, int> pcs, 
					const std::array<bool, INSTRUCTION_TREE_DEPTH> &dep_true,
					std::set<int8_t> dep_out, std::set<int8_t> dep_anti) {
        instruction = instr;
        weight = wt;
        score_bonus = score_b;
		score_multiplier = score_m;
		inverse_dependency_score = inv_d;

		#ifdef log_pcs
		program_counters = pcs;
		#else
		program_counters = {};
		#endif

		for (size_t i = 1; i < trace_depth; i++)
			{
			if(dep_true[i]){
				true_dependencies.push_back(i);
			}
			// output_dependencies = dep_out;
			// anti_dependencies = dep_anti;
		}
		for (int offset : dep_out) //TODO check again if this is correct and refactor
		{
			// if(offset>0){
			// 	printf("[ERROR] Found output dependency offset to future node");
			// }
			output_dependencies.insert(offset);
		}
		for (int offset : dep_anti)
		{
			// if(offset>0){
			// 	printf("[ERROR] Found output dependency offset to future node");
			// }
			anti_dependencies.insert(offset);
		}
}

nlohmann::json PathNode::to_json() const {
		nlohmann::json jsonNode;
		jsonNode["instruction"] = Opcode::mappingStr[instruction];
		jsonNode["weight"] = weight;
		jsonNode["score_bonus"] = score_bonus;
		jsonNode["score_multiplier"] = score_multiplier;
		jsonNode["dependency_score"] = inverse_dependency_score;

		jsonNode["program_counters"] = program_counters;

		nlohmann::json jsonDependencies1 = true_dependencies;
        jsonNode["dependencies_true"] = jsonDependencies1;
		nlohmann::json jsonDependencies2 = anti_dependencies;
        jsonNode["dependencies_anti"] = jsonDependencies2;
		nlohmann::json jsonDependencies3 = output_dependencies;
        jsonNode["dependencies_output"] = jsonDependencies3;

		jsonNode.update(extra_fields);

		return jsonNode;

		// Convert the JSON object to a string
		// std::string jsonString = jsonNode.dump(4); // The argument adds indentation for pretty printing
	}

InstructionType getInstructionType(Opcode::Mapping mapping) {
	using namespace Opcode;
	switch (mapping) {
		case SLLI:
		case SRLI:
		case SRAI:
		case ADD:
		case SUB:
		case SLL:
		case SLT:
		case SLTU:
		case SRL:
		case SRA:
		case MUL:
		case MULH:
		case MULHSU:
		case MULHU:
		case DIV:
		case DIVU:
		case REM:
		case REMU:
		case ADDW:
		case SUBW:
		case SLLW:
		case SRLW:
		case SRAW:
		case MULW:
		case DIVW:
		case DIVUW:
		case REMW:
		case REMUW:
		case AMOSWAP_W:
		case AMOADD_W:
		case AMOXOR_W:
		case AMOAND_W:
		case AMOOR_W:
		case AMOMIN_W:
		case AMOMAX_W:
		case AMOMINU_W:
		case AMOMAXU_W:
		case LR_D:
		case SC_D:
		case AMOSWAP_D:
		case AMOADD_D:
		case AMOXOR_D:
		case AMOAND_D:
		case AMOOR_D:
		case AMOMIN_D:
		case AMOMAX_D:
		case AMOMINU_D:
		case AMOMAXU_D:
		case FADD_S:
		case FSUB_S:
		case FMUL_S:
		case FDIV_S:
		case FSQRT_S:
		case FSGNJ_S:
		case FSGNJN_S:
		case FSGNJX_S:
		case FMIN_S:
		case FMAX_S:
		case FCVT_W_S:
		case FCVT_WU_S:
		case FMV_X_W:
		case FCVT_S_W:
		case FCVT_S_WU:
		case FMV_W_X:
		case FCVT_L_S:
		case FCVT_LU_S:
		case FCVT_S_L:
		case FCVT_S_LU:
		case FADD_D:
		case FSUB_D:
		case FMUL_D:
		case FDIV_D:
		case FSQRT_D:
		case FSGNJ_D:
		case FSGNJN_D:
		case FSGNJX_D:
		case FMIN_D:
		case FMAX_D:
		case FCVT_S_D:
		case FCVT_D_S:
		case FCVT_W_D:
		case FCVT_WU_D:
		case FCVT_D_W:
		case FCVT_D_WU:
		case FCVT_L_D:
		case FCVT_LU_D:
		case FMV_X_D:
		case FCVT_D_L:
		case FCVT_D_LU:
		case FMV_D_X:
		case ADDI:
		case SLTI:
		case SLTIU:
		case ADDIW:
		case SLLIW:
		case SRLIW:
		case SRAIW:
			return InstructionType::Arithmetic;
		case OR:
		case AND:
		case XOR:
		case XORI:
		case ORI:
		case ANDI:
			return InstructionType::Logic;
		case JALR:
		case JAL:
			return InstructionType::Jump;
		case SB:
		case SH:
		case SW:
		case SD:
		case FSW:
		case FSD:
		case LR_W:
		case SC_W:
		case LB:
		case LH:
		case LW:
		case LD:
		case LBU:
		case LHU:
		case LWU:
		case FLW:
		case FLD:
			return InstructionType::Load_Store;
		case FEQ_D:
		case FLT_D:
		case FLE_D:
		case FCLASS_D:
		case FEQ_S:
		case FLT_S:
		case FLE_S:
		case FCLASS_S:
			return InstructionType::Float_Compare;
		case BEQ:
		case BNE:
		case BLT:
		case BGE:
		case BLTU:
		case BGEU:
			return InstructionType::Branch;
		case LUI:
		case AUIPC:
			return InstructionType::LUI;;
		case FMADD_S:
		case FMSUB_S:
		case FNMSUB_S:
		case FNMADD_S:
		case FMADD_D:
		case FMSUB_D:
		case FNMSUB_D:
		case FNMADD_D:
			return InstructionType::Float_R4;

		default:
			return InstructionType::UNKNOWN;
	}
}

