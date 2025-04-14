#include "trace.h"

#include <bitset>
#include "lib/json/single_include/nlohmann/json.hpp"

bool is_first_insert = true;

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

inline uint64_t hash_tree(Opcode::Mapping instruction, uint64_t parent_hash){
	return ((parent_hash << 6) | (parent_hash >> 58)) ^ instruction;
}

#ifdef single_trace_mode
InstructionNodeR* last_node;
#endif

#ifdef single_trace_mode
//this function finds the last element in a tree
//only used for single trace mode, where each node only has one child
InstructionNodeR* InstructionNodeR::get_last(){
	InstructionNodeR* last_node = this; 
	printf("get last %s ", Opcode::mappingStr[last_node->instruction]);
	while (last_node->children.size()>0)
	{
		last_node = children.back();
		printf("->%s", Opcode::mappingStr[last_node->instruction]);
	}
	printf("\n");
	
	return last_node;
}
#endif
//InstructionNodeR
InstructionNodeR::InstructionNodeR(Opcode::Mapping instruction, uint64_t parent_hash)
			: InstructionNode(instruction, parent_hash){
				// for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)//TODO remove should already be 0 initialized
				// 	{
				// 		dependencies_true_[i] = false;
				// 	}
}

void InstructionNodeR::insert_rb(
				std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_steps_p, 
				uint32_t next_rb_index){
					insert_rb(last_executed_steps_p, next_rb_index, 0);
				}
void InstructionNodeR::insert_rb(
				std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_steps_p, 
				uint32_t next_rb_index, uint32_t offset){
	//printf("insert instructions from ringbuffer with len %ld\n", last_executed_instructions.size());
	
	//insert each element of the ringbuffer in order into the tree
	#ifndef single_trace_mode

	InstructionNode *current_node = this;

	#else
	InstructionNodeR *current_node;
	if(!last_node){
		last_node = this;
	}
	current_node = last_node;//this->get_last();
	
	#endif

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
		for(size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++) {
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

	for (uint32_t i = 0; i < INSTRUCTION_TREE_DEPTH-offset; i++)//update the root node and insert all other nodes
	{
		uint8_t rb_index = (next_rb_index+i)%INSTRUCTION_TREE_DEPTH;
		ExecutionInfo* current_step = &last_executed_steps_p[rb_index];// [rb_index];
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

		int8_t memory_true_dependency = -1;
		int8_t memory_output_dependency = -1;
		int8_t memory_anti_dependency = -1;

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
			for (size_t offset_idx = 0; offset_idx < i; offset_idx++)
			{
				// calculate anti dependencies 
				if(register_dependencies_anti[rd][offset_idx]){
					anti_dependencies.set(i - offset_idx, true);
				}
				//calculate output dependencies 
				if(register_dependencies_output[rd][offset_idx]){
					output_dependencies.set(i - offset_idx,true);
				}
			}
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
				
				int8_t relative_val = (i - val + INSTRUCTION_TREE_DEPTH)%INSTRUCTION_TREE_DEPTH; //undef for values <0
				uint8_t color_fg = 249; //rs1+16;
				uint8_t color_bg = (232 + relative_val*23/(INSTRUCTION_TREE_DEPTH-2))%256;
				
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
	#ifndef single_trace_mode
		if(i>0){//the root node already exists and is current_node
			current_node = current_node->insert({				
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
									current_step->last_frame_pointer
									});
			// inserted_nodes[i] = current_node;
		}else{
			update_weight({-1, -1, 0, 0, //the root node does not have any dependencies
			rs1,rs2,rd,
			tmp_input1, tmp_input2, tmp_output,
			0, //depth is 0 as this is the root node
			current_step->last_step_id, 
			current_step->last_cycles, 
			last_memory_access,
			access_type,
			current_step->last_stack_pointer,
			current_step->last_frame_pointer
			});
		}
	#endif
	#ifdef single_trace_mode
	// if(i==INSTRUCTION_TREE_DEPTH-2){
	// 	printf("End of tree check: %s - %s", Opcode::mappingStr[current_node->instruction], Opcode::mappingStr[current_step->last_executed_instruction]);
	// }
	if((i==INSTRUCTION_TREE_DEPTH-1) || (is_first_insert && i>0)){//only insert the last node (as all other nodes are already in the tree)
		last_node = last_node->insert({				
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
								current_step->last_frame_pointer
								});
	}
	#endif
	}
	is_first_insert = false;
	
}
#ifndef single_trace_mode
InstructionNode* InstructionNodeR::insert(const StepInsertInfo& p){
	InstructionNode* found_child = NULL;
	for (auto child : children){
		
		if(child->instruction == p.op){
			found_child = child;
			break;
		}
	}
	if(found_child==NULL){
			switch (p.op)
			{//TODO probably faster to mark these at decoding time
			case Mapping::LB:
			case Mapping::LBU:
			case Mapping::LH:
			case Mapping::LHU:
			case Mapping::LW:
				if(p.depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeMemory(p.op, subtree_hash,0,false));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeMemoryLeaf(p.op, subtree_hash, p.pc, 0, false));
				}
				break;
			case Mapping::SB:
			case Mapping::SH:
			case Mapping::SW:
				if(p.depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeMemory(p.op, subtree_hash,0,true));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeMemoryLeaf(p.op, subtree_hash, p.pc,0,true));
				}
				break;
			
			case Mapping::BEQ:
			case Mapping::BNE:
			case Mapping::BLT:
			case Mapping::BLTU:
			case Mapping::BGE:
			case Mapping::BGEU:
			case Mapping::JAL:
			case Mapping::JALR:
				if(p.depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeBranch(p.op, subtree_hash,-4));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeBranchLeaf(p.op, subtree_hash, p.pc,-4));
				}
				break;

			default:
				//printf("create RNode for instruction: %s\n", mappingStr[op]);
				if(p.depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeR(p.op, subtree_hash));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeLeaf(p.op, subtree_hash, p.pc));
				}
				break;
			}
		
		found_child = children.back();
	}
#else
InstructionNodeR* InstructionNodeR::insert(const StepInsertInfo& p){
	InstructionNodeR* found_child = NULL;
	for (auto child : children){
		
		if(child->instruction == p.op){
			found_child = child;
			break;
		}
	}
	if(found_child==NULL){
			switch (p.op)
			{//TODO probably faster to mark these at decoding time
			case Mapping::LB:
			case Mapping::LBU:
			case Mapping::LH:
			case Mapping::LHU:
			case Mapping::LW:
					children.push_back(new InstructionNodeMemory(p.op, subtree_hash,0,false));
				break;
			case Mapping::SB:
			case Mapping::SH:
			case Mapping::SW:
					children.push_back(new InstructionNodeMemory(p.op, subtree_hash,0,true));
				break;
			
			case Mapping::BEQ:
			case Mapping::BNE:
			case Mapping::BLT:
			case Mapping::BLTU:
			case Mapping::BGE:
			case Mapping::BGEU:
			case Mapping::JAL:
			case Mapping::JALR:
					children.push_back(new InstructionNodeBranch(p.op, subtree_hash,-4));
				break;

			default:
				//printf("create RNode for instruction: %s\n", mappingStr[op]);
					children.push_back(new InstructionNodeR(p.op, subtree_hash));
				break;
			}
		
		found_child = children.back();
	}
#endif
	// else{
	// 
	// }

	//to simplify node creation, we just call update_weight after creation
	//printf("increase weight: %d", found_child->weight+1);
	found_child->update_weight({
								p.true_dependency1,
								p.true_dependency2,
								p.output_dependencies,
								p.anti_dependencies,
								p.rs1,
								p.rs2,
								p.rd,
								p.input1,
								p.input2,
								p.output,
								p.pc,
								p.step,
								p.cycles,
								p.memory_address,
								p.access_type,
								p.stack_pointer,
								p.frame_pointer
							});

	return found_child;
}

void InstructionNodeR::_print(uint8_t level){
	for (int i = 1; i < level; i++) {
		std::cout << "\t";
	}
	const char* instruction_string = "UNKWN ";
	if(instruction < Opcode::mappingStr.size()){
		instruction_string = Opcode::mappingStr[instruction];
	}
	std::cout << "[" << instruction_string << "(" << weight << ")]" << std::endl;
	for (auto child : children) {
		child->_print(level + 1);
	}
}

std::stringstream InstructionNodeR::to_dot(const char* tree_op_name, const char* parent_name,
							uint depth, uint id, uint64_t parent_hash, 
							std::stringstream& dot_stream,  std::stringstream& connections_stream,
							uint64_t tree_weight, uint64_t total_instructions, 
							bool reduce_graph_output, float branch_omission_threshold){
	const char* instruction_string = "UNKWN ";
	if(instruction < Opcode::mappingStr.size()){
		instruction_string = Opcode::mappingStr[instruction];
	}
	const char* label = instruction_string;
	std::stringstream name;

	float hue = dot_hue(depth);
	float saturation = dot_sat(depth, 0); 
	float value = dot_val(depth, 0);
	
	#if 0
	//record type
	// if(depth>0){
	// 	name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
		
	// 	connections_stream << parent_name << " -> " << name.str();
	// 	connections_stream << "[label=\"" << weight << "\"]" << std::endl;

	// 	dot_stream << name.str(); 
	// 	uint16_t color_index = ((float)weight/(float)tree_weight) * 8 + 0.1 + 1; //9 colors (1-9) TODO configure rounding
	// 	dot_stream << "[label=\"" << "{" << label << " | " << subtree_hash << "}" << "\", color=" << color_index << "]" << std::endl;
	// }else{
	// 	name << instruction_string;

	// 	dot_stream << name.str(); 
	// 	std::stringstream top_label;
	// 	float per_weight = (float)weight/(float)total_instructions;
	// 	top_label << "|{" << label << " | " << (float)(((int)(per_weight * 1000.0)) % 1000) / 10.0 << " | " << weight << "/" << total_instructions << "}|";
	// 	uint16_t color_index = std::min(11.0, (per_weight*2.0) * 10 + 0.1 + 1); //TODO is half of all instructions a good max?
	// 	dot_stream << "[label=\"" << top_label.str() << "\", shape = record, color=" << color_index << ", colorscheme=spectral11" << "]" << std::endl;
	// }
	#endif
	//HTML type
	if(depth>0){
		name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
		
		dot_stream << name.str(); 
		float per_weight = (float)weight/(float)tree_weight;
		uint16_t color_index = per_weight * 8 + 0.1 + 1; //9 colors (1-9) TODO configure rounding
		
		dot_stream 
		<< "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
		<< "<TR><TD><FONT COLOR=\"" << hue << " " << saturation << " " << value << "\">" 
		<< label 
		<< "</FONT></TD></TR>"

		<< "<TR><TD>";
		uint dependencies_count = 0;
		for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)
		{
			//printf("[%d]", dependencies_true_[i]);
			if(dependencies_true_[i]){
			// if(i>depth){
			// 	printf("error\n");
			// }
				dependencies_count++;
				dot_stream << "<FONT COLOR=\"" 
				<< dot_hue(depth-i) << " " << dot_sat(depth-i,0) << " " << dot_val(depth,0) << "\">" 
				<< i << " \n" << "</FONT>";
			}
			//TODO needs parent link
			//   splines=line;
	 		// e-> a[constraint=false, style="dashed" color="0.6 0.6 1.000"]
		}
		//printf("num dependencies_true_: %d\n", dependencies_count);
		dot_stream << "</TD></TR>";
		
		//inlcude used registers
		dot_stream << "<TR><TD>";

		/*uint register_count = 0;
		for (size_t i = 0; i < 32; i++) //Number of registers
		{
			//printf("[%d]", dependencies_true_[i]);
			if([i]){
			// if(i>depth){
			// 	printf("error\n");
			// }
				dependencies_count++;
				dot_stream << "<FONT COLOR=\"" 
				<< dot_hue(depth-i) << " " << dot_sat(depth-i,0) << " " << dot_val(depth,0) << "\">" 
				<< i << " \n" << "</FONT>";
			}
			//TODO needs parent link
			//   splines=line;
	 		// e-> a[constraint=false, style="dashed" color="0.6 0.6 1.000"]
		}
		//printf("num dependencies_true_: %d\n", dependencies_count);
		*/


		dot_stream << "</TD></TR>";

		dot_stream 
		<< "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
		<< std::hex << subtree_hash << std::dec 
		<< "</FONT></TD></TR></TABLE>" 
		<< ">, color=" 
		<< color_index << "]" 
		<< std::endl;

		connections_stream << parent_name << " -> " << name.str();
		connections_stream << "[label=\"" << weight << "\" decorate=true";

		if(reduce_graph_output && per_weight<branch_omission_threshold){
			int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0))) ;
			//printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
			connections_stream << " style=\"dashed\" color=\"gray" << shade
			<<  "\" fontcolor=\"gray" << shade 
			<< "\"]" <<std::endl; //std::min(95,(int)(100.0-per_weight*100.0))
			//skip adding children to graph
			return name; 
		}else{
			connections_stream << "]" << std::endl;
		}

	}else{
		name << instruction_string;

		dot_stream << name.str(); 
		std::stringstream top_label;
		float per_weight = (float)weight/(float)total_instructions;
		top_label << "|{" << label << " | " << (float)(((int)(per_weight * 1000.0)) % 1000) / 10.0 << " | " << weight << "/" << total_instructions << "}|";
		uint16_t color_index = std::min(11.0, (per_weight*2.0) * 10 + 0.1 + 1); //TODO is half of all instructions a good max?
		dot_stream << "[label=\"" << top_label.str() << "\", shape = record, color=" << color_index << ", colorscheme=spectral11" << "]" << std::endl;
	}

	//parent_hash = (parent_hash << 6) + instruction; //shifting by 6 should shift value outside opcode range 
	int child_index = 0;
	for (auto child : children) {
		child->to_dot(tree_op_name, name.str().c_str(), 
					depth + 1, child_index, subtree_hash, 
					dot_stream, connections_stream, 
					tree_weight,total_instructions,
					true, 0.05);
		child_index ++;
	}

	return name;
}

void InstructionNodeR::to_csv(const CsvParams& p) {
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

	// uint64_t cycles = 0;
	// switch (instruction)
	// {
	// case Opcode::LB:
	// case Opcode::LBU:
	// case Opcode::LH:
	// case Opcode::LHU:
	// case Opcode::LW:
	// case Opcode::SB:
	// case Opcode::SH:
	// case Opcode::SW:
	// 	cycles +=4;
	// 	break;
	// case Opcode::MUL:
	// case Opcode::MULH:
	// case Opcode::MULHU:
	// case Opcode::MULHSU:
	// case Opcode::DIV:
	// case Opcode::DIVU:
	// case Opcode::REM:
	// case Opcode::REMU:
	// 	cycles +=8;
	// 	break;
	// default:
	// 	cycles +=1;
	// 	break;
	// }

	csv_stream = csv_format(p.parent_hash, p.tree, instruction_string, p.last_weight, p.max_weight, p.total_max_weight, 
								p.depth, current_dep_score, current_total_dep_score, 
								current_true_dep, current_anti_dep, current_out_dep, 
								total_true_dep, total_anti_dep,total_out_dep, 
								children.size(), current_total_inputs.count(), 
								current_total_outputs.count(), _instruction_types[InstructionType::Branch], 
								number_of_pcs, p.max_pcs);
	std::cout << csv_stream.str() <<std::endl;

	for (auto &&child : children)
	{
		// csv_stream << std::endl;
		child->to_csv({
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

nlohmann::ordered_json InstructionNodeR::to_json(){
	nlohmann::ordered_json additional_fields;

	nlohmann::ordered_json jsonChildren = nlohmann::ordered_json::array(); 
	for (auto &&child : children)
	{
		jsonChildren.push_back(child->to_json());
	}
	additional_fields["children"] = jsonChildren;

	nlohmann::ordered_json base_class_json = InstructionNode::to_json();
	base_class_json.update(additional_fields);
	return base_class_json;
}

//called recursively for children
Path InstructionNodeR::extend_path(const PathExtensionParams& p){
	
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
	max_path.score_bonus = p.score_bonus + score_bonus_of_this_node;
	max_path.score_multiplier = 
			p.score_multiplier * score_multiplier_of_this_node; 
	max_path.inverse_dependency_score = get_inv_dep_score();
	
	//(forward branch and backward branch out of scope have score multiplier = 0)
	max_path.opcodes.push_back(instruction);
	max_path.path_hashes.push_back(subtree_hash);

	max_path.end_of_sequence = this;

	//printf("tree %d depth %d\r", tree_id, length);

	//handle children

	// if(typeid(*this)==typeid(InstructionNodeBranch)){ //override instead
	// 	return max_path;
	// }

	Path max_child_path;
	Path child_path;
	int child_index = 0;
	for (InstructionNode* child : children) {

		// if(typeid(*child)==typeid(InstructionNodeBranch)){
		// //printf("Found Branch in child\n\n\n");
		// //return max_path;
		
		// }
		child_path = child->extend_path({p.length+1, 
				max_path.score_bonus, max_path.score_multiplier, p.tree_id, p.force_extension_depth, 
				p.force_instruction, p.score_function});
		if(max_child_path.length==0){ //should not happen with force_extension
			max_child_path = child_path;
		}else{
			if(p.force_extension_depth+1 == p.length && p.force_instruction == child->instruction){
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
		child_index++;
	}

	if (max_child_path.length>0)
	{
		if((max_path.get_score(p.score_function) < max_child_path.get_score(p.score_function)) || p.force_extension_depth+1==p.length){
			max_path.length = max_child_path.length;
			max_path.minimum_weight = max_child_path.minimum_weight;
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

std::vector<Path> InstructionNodeR::force_path_extension(const Path p, std::function <float(ScoreParams)> score_function){
	std::vector<Path> extended_sequences;
	//handle sequences ending in a branch/jump
	//force extend the path to the branch/jump if the node is the only child
	Path path_origin = p;
	if(children.size() == 1 && (getInstructionType(children.front()->instruction) == InstructionType::Branch ||  getInstructionType(children.front()->instruction) == InstructionType::Jump)){
		InstructionNode* child = children.front();
		InstructionNodeBranch* branch_child = dynamic_cast<InstructionNodeBranch*>(child); //check whether we already reached the end of the tree
		if(branch_child == NULL){
			printf("Warning: only child of sequence is a branch, but no further nodes to extend the sequence exist\n");
			return extended_sequences;
		}
		path_origin.length += 1;
		assert(path_origin.minimum_weight == branch_child->weight);

		float score_bonus_of_next_node = branch_child->get_score_bonus();
		// float score_multiplier_of_next_node = branch_child->get_score_multiplier();

		path_origin.score_bonus += score_bonus_of_next_node;
		// path_origin.score_multiplier += score_multiplier_of_next_node; 
		path_origin.inverse_dependency_score = branch_child->get_inv_dep_score();//TODO check if this is correct
		
		//(forward branch and backward branch out of scope have score multiplier = 0)
		path_origin.opcodes.push_back(branch_child->instruction);
		path_origin.path_hashes.push_back(branch_child->subtree_hash);

		//TODO
		// extended_sequences.push_back(path_origin);
		return children.front()->force_path_extension(path_origin, score_function);
	}
	for (auto &&child : children)
	{
		Path max_path = p;
		Path child_path = child->extend_path({p.length+1, 
				p.score_bonus, p.score_multiplier, -1, -1, Opcode::Mapping::UNDEF, score_function});
		// if (child_path.length>0)
		// {
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
		// }


		extended_sequences.push_back(max_path);
	}
	return extended_sequences;
	
}

int InstructionNodeR::prune_tree(uint64_t weight_threshold, uint8_t depth){
	#ifndef single_trace_mode
	uint8_t child_idx = 0;
	bool pruned = false;
	for (InstructionNode*& child : children) {//iterate over the actual pointers
		if (child->weight < weight_threshold)
		{
			if(!pruned){
				printf("\t\tpruned branch at depth: %d\n", depth);
				pruned = true;

			}
			if(dynamic_cast<InstructionNodeLeaf*>(child)){
				return 1; //branch should be pruned, but child is leaf/end of branch anyway
			}
			//might be better to use copy constructors
			else if(InstructionNodeMemory* der_child = dynamic_cast<InstructionNodeMemory*>(child)){
				InstructionNodeMemoryLeaf* new_child = 
					new InstructionNodeMemoryLeaf(der_child->instruction, 
					der_child->subtree_hash, -1, 0, der_child->is_store);
					new_child->pc_map = der_child->get_pc();
					new_child->memory_accesses = der_child->memory_accesses;
					// delete child;
					child = new_child;//replace original child with leaf node
				
			}
			else if(InstructionNodeBranch* der_child = dynamic_cast<InstructionNodeBranch*>(child)){
				InstructionNodeBranchLeaf* new_child = 
					new InstructionNodeBranchLeaf(der_child->instruction, 
					der_child->subtree_hash, -1, 0);
					new_child->pc_map = der_child->get_pc();
					new_child->relative_offsets = der_child->relative_offsets;
					// delete child;
					child = new_child;
			}
			else if(InstructionNodeR* der_child = dynamic_cast<InstructionNodeR*>(child)){
				InstructionNodeLeaf* new_child = 
					new InstructionNodeLeaf(der_child->instruction, 
					der_child->subtree_hash, -1);
					new_child->pc_map = der_child->get_pc();
					// delete child;
					child = new_child;
			}
			else{
				printf("[ERROR] unknown type of child node\n");
				exit(1);
				return 3;
			}
		}
		
		child->prune_tree(weight_threshold, depth+1);
		child_idx++;
	}
	#else
		printf("[ERROR] pruning not supported for single trace mode");
	#endif
	return 0;
}

std::map<uint64_t, int> InstructionNodeR::get_pc(){
	std::map<uint64_t, int> pcs; 

	#ifdef trace_pcs
	for (InstructionNode* child : children) {
		std::map<uint64_t, int> child_pcs = child->get_pc();
		pcs.insert(child_pcs.begin(), child_pcs.end());
	}
	#endif
	return pcs;
}

InstructionNodeLeaf::InstructionNodeLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc)
		: InstructionNode(instruction, parent_hash){
	//pc_map.insert({pc,1}); //this is handled in the update step
}

InstructionNode* InstructionNodeLeaf::insert(const StepInsertInfo& p){
	printf("[Insert ERROR] Maximum depth reached");
	return NULL;
}

std::map<uint64_t, int> InstructionNodeLeaf::get_pc(){
	return pc_map;
}

void InstructionNodeLeaf::_print(uint8_t level){
		for (int i = 1; i < level; i++) {
			std::cout << "\t";
		}
		const char* instruction_string = "UNKWN ";
		if(instruction < Opcode::mappingStr.size()){
			instruction_string = Opcode::mappingStr[instruction];
		}
		std::cout << "[" << instruction_string << "(" << weight << ")]" << std::endl;
}

std::stringstream InstructionNodeLeaf::to_dot(const char* tree_op_name, const char* parent_name,
								uint depth, uint id, uint64_t parent_hash, 
								std::stringstream& dot_stream,  std::stringstream& connections_stream,
								uint64_t tree_weight, uint64_t total_instructions, 
								bool reduce_graph_output, float branch_omission_threshold){
		const char* instruction_string = "UNKWN ";
		if(instruction < Opcode::mappingStr.size()){
			instruction_string = Opcode::mappingStr[instruction];
		}
		const char* label = instruction_string;
		std::stringstream name;
		//HTML type
		if(depth>=INSTRUCTION_TREE_DEPTH-1){
			name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
			
			dot_stream << name.str(); 
			float per_weight = (float)weight/(float)tree_weight;
			uint16_t color_index = per_weight * 8 + 0.1 + 1; //9 colors (1-9) TODO configure rounding
			dot_stream << "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
			<< "<TR><TD>" 
			<< label 
			<< "</TD></TR>" 
			<< "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
			<< std::hex << subtree_hash << std::dec 
			<< "</FONT></TD></TR>" 

			<< "<TR><TD><FONT COLOR=\"0.6 0.4 0.600\" POINT-SIZE=\"10\">" << std::hex;
			for (auto const& x : pc_map)
			{
				dot_stream  << x.first << ":" << x.second << " "; 
			}
				
			dot_stream << std::dec 
			<< "</FONT></TD></TR></TABLE>" 

			<< ">, color=" 
			<< color_index << "]" 
			<< std::endl;

			connections_stream << parent_name << " -> " << name.str();
			connections_stream << "[label=\"" << weight << "\" decorate=true";

			if(reduce_graph_output && per_weight<branch_omission_threshold){
				int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0))) ;
				//printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
				connections_stream << " style=\"dashed\" color=\"gray" << shade
				<<  "\" fontcolor=\"gray" << shade 
				<< "\"]" <<std::endl; //std::min(95,(int)(100.0-per_weight*100.0))
				//skip adding children to graph
				return name; 
			}else{
				connections_stream << "]" << std::endl;
			}

		}else{ //should only happen wih pruned trees
			name << "pruned_" << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
			
			dot_stream << name.str(); 
			//float per_weight = (float)weight/(float)tree_weight;
			uint16_t color_index = 1; //9 colors (1-9) TODO configure rounding
			dot_stream << "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
			<< "<TR><TD>" 
			<< label 
			<< "</TD></TR>" 
			<< "<TR><TD><FONT COLOR=\"0.8 0.0 0.1\" POINT-SIZE=\"14\">" 
			<< std::hex << "pruned" << std::dec 
			<< "</FONT></TD></TR>" 

			#ifdef dot_pc_on_pruned_nodes
			<< "<TR><TD><FONT COLOR=\"0.6 0.4 0.600\" POINT-SIZE=\"10\">" << std::hex;
			for (auto const& x : pc_map)
			{
				dot_stream  << x.first << ":" << x.second << " "; 
			}
				
			dot_stream << std::dec 
			<< "</FONT></TD></TR>" 
			#endif
			<< "</TABLE>" << ">, color=" 
			<< color_index << "]" 
			<< std::endl;

			connections_stream << parent_name << " -> " << name.str();
			connections_stream << "[label=\"" << weight << "\" decorate=true";
			connections_stream << "]" << std::endl;

			dot_stream << std::dec << std::endl;
			//<< "Error: using leaf node inside tree" << std::endl;
			printf("[Warning]: using leaf node inside tree for dot (pruned?)\n");
			return name;
		}

		
		//dont process children as this is a leaf node

		return name;
}

void InstructionNodeLeaf::update_weight(const StepUpdateInfo& p){
		InstructionNode::update_weight(p);
		pc_map[p.pc]++; //if key does not exist, it is created with value 0 
	}

nlohmann::ordered_json InstructionNodeLeaf::to_json(){
	nlohmann::ordered_json additional_fields;
	//additional_fields["PCs"] = pc_map;

	nlohmann::json base_class_json = InstructionNode::to_json();
	base_class_json.update(additional_fields);
	return base_class_json;
}

MemoryNode::MemoryNode(bool is_store_instruction) : is_store(is_store_instruction){
}

void MemoryNode::register_access(uint64_t pc, uint64_t address, 
	AccessType access_type, uint64_t prev_access, 
				uint64_t stackpointer, uint64_t framepointer){
					uint64_t access_offset = abs((long int)(address-last_access));
					access_offset_sum += access_offset;
					int64_t accessor_diff = 0; 

		accessor_diff = prev_access - last_access;
	
	Opcode::MemoryRegion memory_location = Opcode::MemoryRegion::NONE;
	if(framepointer>0 && address <= framepointer && address >= stackpointer){
		//address is in Frame
		memory_location = memory_location | MemoryRegion::FRAME;
		//printf("FRAME Access %lx\n",address);
	}else{
		if(address<stackpointer){
			//address is not on the Stack
			memory_location = memory_location | MemoryRegion::HEAP;
			//printf("HEAP Access %lx\n",address);
		}else{
			//address is on the Stack but not in Frame
			memory_location = memory_location | MemoryRegion::STACK;
			//printf("STACK Access %lx\n",address);
		}
	}
	auto& access_entry = memory_accesses[pc]; //If the entry does not exist, it is created
	// Check if a pair with memory_location is already in the set
	bool found = false;
	for (const auto& entry : access_entry) {
		if (static_cast<Opcode::MemoryRegion>(entry.first) == memory_location) {
			found = true;
			break;
		}
	}

	// Add the pair (memory_location, memory_location) to the set if not already contained
	if (!found) {
		access_entry.insert({address, memory_location});
	}
}	

InstructionNodeMemory::InstructionNodeMemory(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t memory_access, bool is_store_instruction)
			: InstructionNode(instruction, parent_hash), 
			  MemoryNode(is_store_instruction),
			  InstructionNodeR(instruction, parent_hash){ 
}

void InstructionNodeMemory::_print(uint8_t level){
	for (int i = 1; i < level; i++) {
		std::cout << "\t";
	}
	const char* instruction_string = "UNKWN MEMORY ";
	if(instruction < Opcode::mappingStr.size()){
		instruction_string = Opcode::mappingStr[instruction];
	}
	std::cout << "[" << instruction_string << "(" << weight << ")]" << std::endl;
	for (auto child : children) {
		child->_print(level + 1);
	}
}

std::stringstream InstructionNodeMemory::to_dot(const char* tree_op_name, const char* parent_name,
							uint depth, uint id, uint64_t parent_hash, 
							std::stringstream& dot_stream,  std::stringstream& connections_stream,
							uint64_t tree_weight, uint64_t total_instructions, 
							bool reduce_graph_output, float branch_omission_threshold){
	const char* instruction_string = "UNKWN MEMORY ";
	if(instruction < Opcode::mappingStr.size()){
		instruction_string = Opcode::mappingStr[instruction];
	}
	const char* label = instruction_string;
	std::stringstream name;

	//HTML type
	if(depth>0){
		name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_Mem_" << instruction_string;
		
		dot_stream << name.str(); 
		float per_weight = (float)weight/(float)tree_weight;
		uint16_t color_index = per_weight * 8 + 0.1 + 1; //9 colors (1-9) TODO configure rounding
		
		dot_stream << "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
		<< "<TR><TD>" 
		<< label 
		<< "</TD></TR>" 

		<< "<TR><TD>";
		uint dependencies_count = 0;
		for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)
		{
			if(dependencies_true_[i]){
				dependencies_count++;
				dot_stream << "<FONT COLOR=\"" 
				<< dot_hue(depth-i) << " " << dot_sat(depth-i,0) << " " << dot_val(depth,0) << "\">" 
				<< i << " \n" << "</FONT>";
			}
		}
		dot_stream << "</TD></TR>"

		<< "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
		<< std::hex << subtree_hash << std::dec 
		<< "</FONT></TD></TR>"
		<< "<TR><TD><FONT COLOR=\"0.2 0.8 1.000\" POINT-SIZE=\"10\">";
		//<< (int)memory_location << ": ";
		dot_stream << "[";
		for (auto &&i : this->memory_accesses)
		{
			dot_stream << std::hex << i.first << ": {";
			for(auto &&pair : i.second){
				dot_stream << "(" << std::hex << (pair.first & 0xFFFF) << " - " << std::hex << (int)pair.second << "), ";
			}
			dot_stream << std::hex << i.first << "}" << std::dec;
		}
		dot_stream << std::dec << "]";
		dot_stream 
		<< "</FONT></TD></TR></TABLE>" 
		<< ">, color=" 
		<< color_index << "]" 
		<< std::endl;

		connections_stream << parent_name << " -> " << name.str();
		connections_stream << "[label=\"" << weight << "\" decorate=true";

		if(reduce_graph_output && per_weight<branch_omission_threshold){
			int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0))) ;
			//printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
			connections_stream << " style=\"dashed\" color=\"gray" << shade
			<<  "\" fontcolor=\"gray" << shade 
			<< "\"]" <<std::endl; //std::min(95,(int)(100.0-per_weight*100.0))
			//skip adding children to graph
			return name; 
		}else{
			connections_stream << "]" << std::endl;
		}

	}else{
		name << instruction_string;

		dot_stream << name.str(); 
		std::stringstream top_label;
		float per_weight = (float)weight/(float)total_instructions;
		top_label << "|{" << label << " | " << (float)(((int)(per_weight * 1000.0)) % 1000) / 10.0 << " | " << weight << "/" << total_instructions << "}|";
		uint16_t color_index = std::min(11.0, (per_weight*2.0) * 10 + 0.1 + 1); //TODO is half of all instructions a good max?
		dot_stream << "[label=\"" << top_label.str() << "\", shape = record, color=" << color_index << ", colorscheme=spectral11" << "]" << std::endl;
	}

	int child_index = 0;
	for (auto child : children) {
		child->to_dot(tree_op_name, name.str().c_str(), 
					depth + 1, child_index, subtree_hash, 
					dot_stream, connections_stream, 
					tree_weight,total_instructions,
					true, 0.05);
		child_index ++;
	}

	return name;
}

void InstructionNodeMemory::update_weight(const StepUpdateInfo& p){
	InstructionNode::update_weight(p);
	register_access(p.pc, p.memory_address, p.access_type, 0, p.stack_pointer, p.frame_pointer);
}

InstructionNodeMemoryLeaf::InstructionNodeMemoryLeaf(Mapping instruction, uint64_t parent_hash, uint64_t pc, 
			uint64_t memory_access, bool is_store_instruction)
			: InstructionNode(instruction, parent_hash), 
			  MemoryNode(is_store_instruction),
			  InstructionNodeLeaf(instruction, parent_hash, pc){
}

void InstructionNodeMemoryLeaf::_print(uint8_t level){
	printf("Not implemented");
}

void InstructionNodeMemoryLeaf::update_weight(const StepUpdateInfo& p){
	InstructionNode::update_weight(p);
	register_access(p.pc, p.memory_address, p.access_type, 0, p.stack_pointer, p.frame_pointer);
	pc_map[p.pc]++;
}

std::stringstream InstructionNodeMemoryLeaf::to_dot(const char* tree_op_name, const char* parent_name,
								uint depth, uint id, uint64_t parent_hash, 
								std::stringstream& dot_stream,  std::stringstream& connections_stream,
								uint64_t tree_weight, uint64_t total_instructions, 
								bool reduce_graph_output, float branch_omission_threshold){
									const char* instruction_string = "UNKWN ";
		if(instruction < Opcode::mappingStr.size()){
			instruction_string = Opcode::mappingStr[instruction];
		}
		const char* label = instruction_string;
		std::stringstream name;
		//HTML type
		if(depth>=INSTRUCTION_TREE_DEPTH-1){//standard leaf node
			name << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
			
			dot_stream << name.str(); 
			float per_weight = (float)weight/(float)tree_weight;
			uint16_t color_index = per_weight * 8 + 0.1 + 1; //9 colors (1-9) TODO configure rounding
			dot_stream << "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
			<< "<TR><TD>" 
			<< label 
			<< "</TD></TR>" 
			<< "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
			<< std::hex << subtree_hash << std::dec 
			<< "</FONT></TD></TR>"
			<< "<TR><TD><FONT COLOR=\"0.2 0.8 1.000\" POINT-SIZE=\"10\">";
			dot_stream << "[";
			for (auto &&i : this->memory_accesses)
			{
				dot_stream << std::hex << i.first << ": {";
				for(auto &&pair : i.second){
					dot_stream << "(" << std::hex << (pair.first & 0xFFFF) << " - " << std::hex << (int)pair.second << "), ";
				}
				dot_stream << std::hex << i.first << "}" << std::dec;
			}
			dot_stream << std::dec << "]" 

			<< "</FONT></TD></TR>"  

			<< "<TR><TD><FONT COLOR=\"0.6 0.4 0.600\" POINT-SIZE=\"10\">" << std::hex;
			for (auto const& x : pc_map)
			{
				dot_stream  << x.first << ":" << x.second << " "; 
			}
				
			dot_stream << std::dec 
			<< "</FONT></TD></TR></TABLE>" 

			<< ">, color=" 
			<< color_index << "]" 
			<< std::endl;

			connections_stream << parent_name << " -> " << name.str();
			connections_stream << "[label=\"" << weight << "\" decorate=true";

			if(reduce_graph_output && per_weight<branch_omission_threshold){
				int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0))) ;
				//printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
				connections_stream << " style=\"dashed\" color=\"gray" << shade
				<<  "\" fontcolor=\"gray" << shade 
				<< "\"]" <<std::endl; //std::min(95,(int)(100.0-per_weight*100.0))
				//skip adding children to graph
				return name; 
			}else{
				connections_stream << "]" << std::endl;
			}

		}else{//leaf node inside tree (only created by pruning the tree)
			name << "pruned_" << tree_op_name << "_d" << depth << "_c" << id << "_p" << parent_hash << "_" << instruction_string;
			
			dot_stream << name.str(); 
			//float per_weight = (float)weight/(float)tree_weight;
			uint16_t color_index = 1; //9 colors (1-9) TODO configure rounding
			dot_stream << "[label=<<TABLE BORDER=\"2\" CELLBORDER=\"0\" CELLSPACING=\"0\" CELLPADDING=\"0\">" 
			<< "<TR><TD>" << label << "</TD></TR>" 
			<< "<TR><TD><FONT COLOR=\"0.8 0.0 0.1\" POINT-SIZE=\"14\"> pruned </FONT></TD></TR>"

			#ifdef dot_pc_on_pruned_nodes
			<< "<TR><TD><FONT COLOR=\"0.6 0.4 0.600\" POINT-SIZE=\"10\">" << std::hex;
			for (auto const& x : pc_map)
			{
				dot_stream  << x.first << ":" << x.second << " "; 
			}
			
			dot_stream << std::dec 
			<< "</FONT></TD></TR>" 
			#endif
			<< "</TABLE>" << ">, color=" 
			<< color_index << "]" 
			<< std::endl;

			connections_stream << parent_name << " -> " << name.str();
			connections_stream << "[label=\"" << weight << "\" decorate=true";
			connections_stream << "]" << std::endl;

			dot_stream << std::dec << std::endl;
			printf("[Warning]: using leaf node inside tree for dot (pruned?)\n");
			return name;
		}

		//dont process children as this is a leaf node

		return name;
}

BranchNode::BranchNode(int64_t offset){
	if(offset!=0){
		relative_offsets[offset]++;
		if(offset<0){
			is_backward_jump = true;
		}else{
			is_forward_jump = true;
		}
	}
}

InstructionNodeBranch::InstructionNodeBranch(Mapping instruction, uint64_t parent_hash, int64_t offset): 
			  InstructionNode(instruction, parent_hash), 
			  BranchNode(offset),
			  InstructionNodeR(instruction, parent_hash){
			  }

InstructionNodeBranchLeaf::InstructionNodeBranchLeaf(Mapping instruction, uint64_t parent_hash, uint64_t pc, 
									int64_t offset): 
			  InstructionNode(instruction, parent_hash), 
			  BranchNode(offset),
			  InstructionNodeLeaf(instruction, parent_hash, pc){
			  }

std::vector<PathNode> InstructionNodeR::path_to_path_nodes(Path path, uint depth){
	std::vector<PathNode> nodes; 
	//const InstructionNode& node, float score_b, float score_m, float inv_d, std::map<uint64_t, int> pcs)
		
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

	PathNode n = PathNode(instruction, weight, get_score_bonus(), get_score_multiplier(), get_inv_dep_score(), 
							get_pc(), 
							dependencies_true_, indices_out, indices_anti);
	nodes.push_back(n);

	if((depth+1)>=path.length){ //path.opcodes[depth+1] == Opcode::UNDEF){
		return nodes;
	}
	else{

		InstructionNode* found_child = NULL;
		for (auto child : children){
			
			if(child->instruction == path.opcodes[depth+1]){
				found_child = child;
				break;
			}
		}
		if(found_child==NULL){
			printf("[ERROR] no children in tree that match discovered path"); //should not be possible
		}
		std::vector<PathNode> child_nodes = found_child->path_to_path_nodes(path, depth+1); 
		nodes.insert(nodes.end(), child_nodes.begin(), child_nodes.end());
		return nodes;
	}
}

//find a point in an existing sequence with the highest ratio between branch taken in the original sequence 
// and another possible branch not taken, which would lead to a different sequence 
std::vector<BranchingPoint> InstructionNodeR::find_variant_branch(Path path, uint8_t depth){//uint8_t variant_index
	//find branching point
	//then create new Path up to this point and extend_path()

	std::vector<BranchingPoint> branching_points; 

	if((uint32_t)(depth+1)>=path.length){ //path.opcodes[depth+1] == Opcode::UNDEF){
		return branching_points;
	}
	else{

		InstructionNode* found_child = NULL;
		int64_t current_max_weight = -1;
		double current_max_ratio = -1.0;
		Opcode::Mapping current_instruction = Opcode::UNDEF;
		for (auto child : children){
			
			if(child->instruction == path.opcodes[depth+1]){
				found_child = child; //if child is on previous best path, continue variant search with that node
			}else{
				double current_ratio = (double)child->weight / (double)weight; //otherwise save possible branching point
				if(current_ratio>current_max_ratio){
					current_max_weight = child->weight;
					current_max_ratio = current_ratio;
					current_instruction = child->instruction;
				}
			}
		}

		if(found_child==NULL){
			printf("[ERROR] no children in tree that match discovered path"); //should not be possible
		}

		if(current_max_weight>0){
			BranchingPoint bp = {depth, current_instruction, current_max_weight, current_max_ratio, this}; 
			branching_points.push_back(bp);
		}
		std::vector<BranchingPoint> child_branching_points = found_child->find_variant_branch(path, depth+1); 
		branching_points.insert(branching_points.end(), child_branching_points.begin(), child_branching_points.end());
		return branching_points;
	}
}

NODE_TYPE InstructionNodeR::get_node_type(){
	return NODE_TYPE::NODE;
}

NODE_TYPE InstructionNodeLeaf::get_node_type(){
	return NODE_TYPE::LEAF;
}

NODE_TYPE InstructionNodeBranch::get_node_type(){
	return NODE_TYPE::BRANCH_R;
}

NODE_TYPE InstructionNodeBranchLeaf::get_node_type(){
	return NODE_TYPE::BRANCH_L;
}

NODE_TYPE InstructionNodeMemory::get_node_type(){
	return NODE_TYPE::MEMORY_R;
}

NODE_TYPE InstructionNodeMemoryLeaf::get_node_type(){
	return NODE_TYPE::MEMORY_L;
}


PathNode::PathNode(Opcode::Mapping instr, uint64_t wt, float score_b, float score_m, float inv_d, 
					std::map<uint64_t, int> pcs, 
					std::array<bool, INSTRUCTION_TREE_DEPTH> dep_true, std::set<int8_t> dep_out, std::set<int8_t> dep_anti) {
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

		for (size_t i = 1; i < INSTRUCTION_TREE_DEPTH; i++)
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

nlohmann::json PathNode::to_json(){
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

