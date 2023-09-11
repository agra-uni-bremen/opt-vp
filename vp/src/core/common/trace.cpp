#include "trace.h"

using namespace Opcode;

//InstructionNodeR
InstructionNodeR::InstructionNodeR(Opcode::Mapping instruction, uint64_t parent_hash)
			: InstructionNode(instruction, parent_hash){
				for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH; i++)//TODO remove should already be 0 initialized
					{
						dependencies[i] = false;
					}
}

void InstructionNodeR::insert_rb(
				std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_steps, 
				uint8_t next_rb_index){
	//printf("insert instructions from ringbuffer with len %ld\n", last_executed_instructions.size());
	
	//insert each element of the ringbuffer in order into the tree
	InstructionNode *current_node = this;
	weight++;
	uint8_t rb_index_start = (next_rb_index+1)%INSTRUCTION_TREE_DEPTH;//next_rb_index is this op

	int8_t register_dependencies[32] = {-1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1,
									   -1,-1,-1,-1,-1,-1,-1,-1};
	

	for (size_t i = 0; i < INSTRUCTION_TREE_DEPTH-1; i++)//insert all other DEPTH-1 ops
	{
		uint8_t index = (rb_index_start+i)%INSTRUCTION_TREE_DEPTH;

		std::tuple<uint16_t,uint16_t,uint16_t> regs = last_executed_steps[index].last_registers;

		uint8_t rs1 = std::get<0>(regs);
		uint8_t rs2 = std::get<1>(regs);
		//uint8_t rs3 = std::get<0>(regs[index]);
		uint8_t rd = std::get<2>(regs);
		
		//calculate register dependencies
		int8_t dependency1 = -1;
		if(register_dependencies[rs1] > -1){ //maybe use >0 as instruction don't depend on zero reg
			dependency1 = (index - register_dependencies[rs1] 
							+ INSTRUCTION_TREE_DEPTH)%INSTRUCTION_TREE_DEPTH; //undef for values <0
		}
		int8_t dependency2 = -1;
		if(register_dependencies[rs2] > -1){ 
			dependency2 = (index - register_dependencies[rs2] 
							+ INSTRUCTION_TREE_DEPTH)%INSTRUCTION_TREE_DEPTH;
		}
		//int8_t dependency3 = -1; //TODO support R4 Fused Multiply Instructions

		//index of instruction for which its results was overwritten by this instruction
		int8_t reg_overwrite = register_dependencies[rd];
		
		//relative offset to current instruction
		int8_t relative_overwrite = -1;
		if (reg_overwrite>0)//undef for uninitialized or unknown register dependencies
		{
			relative_overwrite = (index - reg_overwrite 
				+ INSTRUCTION_TREE_DEPTH)%INSTRUCTION_TREE_DEPTH; 
		}

		Type type = getType(last_executed_steps[index].last_executed_instruction);

		switch (type)
		{
		case Type::R :
			//rs1, rs2, rd
			register_dependencies[rd] = index;
			break;

		case Type::I :
			/* rs1, rd */
			dependency2 = -1;
			register_dependencies[rd] = index;
			break;

		case Type::S :
			/* rs1, rs2, memory TODO*/
			break;

		case Type::U :
			/* rd */
			dependency1 = -1;
			dependency2 = -1;
			register_dependencies[rd] = index;
			break;

		case Type::R4 :
			/* code */
			break;

		case Type::J :
			/* rd */
			dependency1 = -1;
			dependency2 = -1;
			register_dependencies[rd] = index;
			break;

		case Type::B :
			/* Branch rs1, rs2 TODO add condition? */
			break;
		
		default:
			break;
		}

		register_dependencies[0] = -1; //reset zero register in case an instruction wrote to it

		//printf("Last regs:%d, %d -> %d\n",rs1, rs2, rd);
		//#define debug_register_dependencies
		#ifdef debug_register_dependencies
		// if(i==INSTRUCTION_TREE_DEPTH-2){
			for (int8_t j = 0; j < 32; j++)
			{
				int8_t val = register_dependencies[j];
				
				int8_t relative_val = (index - val + INSTRUCTION_TREE_DEPTH)%INSTRUCTION_TREE_DEPTH; //undef for values <0
				uint8_t color_fg = 249; //rs1+16;
				uint8_t color_bg = (232 + relative_val*23/(INSTRUCTION_TREE_DEPTH-2))%256;
				
				if(j == dependency1){
										//color group + start offset + group index
					color_fg = ((dependency1%30)/6)*36+16 + (dependency1%10)*6;
					color_bg = color_bg-180;
				}
				if(j == dependency2){
					color_fg = ((dependency2%30)/6)*36+21 + (dependency2%10)*6;
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

		//calculate memory dependencies TODO

		current_node = current_node->insert(
								last_executed_steps[index].last_executed_instruction, 
								last_executed_steps[index].last_executed_pc,
								dependency1,
								dependency2,
								relative_overwrite,
								i+1);
	}
	#ifdef debug_register_dependencies
	printf("\n");
	#endif

	
}

InstructionNode* InstructionNodeR::insert(Opcode::Mapping op, uint64_t pc, 
								int8_t dependency1, int8_t dependency2, int8_t relative_overwrite, 
								uint8_t depth){
	InstructionNode* found_child = NULL;
	for (auto child : children){
		
		if(child->instruction == op){
			found_child = child;
			break;
		}
	}
	if(found_child==NULL){
			switch (op)
			{//TODO mark these at decoding time
			case Mapping::LB:
			case Mapping::LBU:
			case Mapping::LH:
			case Mapping::LHU:
			case Mapping::LW:
				if(depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeMemory(op, subtree_hash,0,false));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeMemoryLeaf(op, subtree_hash, pc, 0, false));
				}
				break;

			case Mapping::SB:
			case Mapping::SH:
			case Mapping::SW:
				if(depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeMemory(op, subtree_hash,0,true));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeMemoryLeaf(op, subtree_hash, pc,0,true));
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
				if(depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeBranch(op, subtree_hash,-4));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeBranchLeaf(op, subtree_hash, pc,-4));
				}
				break;

			default:
				//printf("create RNode for instruction: %s\n", mappingStr[op]);
				if(depth < INSTRUCTION_TREE_DEPTH-1){
					children.push_back(new InstructionNodeR(op, subtree_hash));
				}else{
					//create leaf node instead
					children.push_back(new InstructionNodeLeaf(op, subtree_hash, pc));
				}
				break;
			}
		
		found_child = children.back();
	}else{
		//printf("increase weight: %d", found_child->weight+1);
		//TODO proper arguments
		//TODO should thi be called on object creation?
		//TODO dependencies are not registered at node creation
		found_child->update_weight(dependency1,dependency2,
									relative_overwrite,pc);
	}
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
			//printf("[%d]", dependencies[i]);
			if(dependencies[i]){
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
		//printf("num dependencies: %d\n", dependencies_count);
		


		dot_stream << "</TD></TR>";
		
		//inlcude used registers
		dot_stream << "<TR><TD>";

		/*uint register_count = 0;
		for (size_t i = 0; i < 32; i++) //Number of registers
		{
			//printf("[%d]", dependencies[i]);
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
		//printf("num dependencies: %d\n", dependencies_count);
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
			printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
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

//called recursively for children
Path InstructionNodeR::extend_path(uint32_t length, float score_bonus, float score_multiplier, uint32_t tree_id){
	
	//score multiplier for this singular node
	//we can't simply pass its weight * mult as score as the weight changes when extending the path  
	//instead track a bonus multiplier that adds mult * min_weight to the score 
	//can be negative
	float score_bonus_of_this_node = get_score_bonus();
	//global score multiplier of this node
	//including branches etc. should reduce the score of the whole path, so its not chosen
	float score_multiplier_of_this_node = get_score_multiplier();

	Path max_path;
	max_path.length = length; //was increased on call
	max_path.minimum_weight = weight; //weight of a child should always be smaller or equal
	max_path.score_bonus = score_bonus + score_bonus_of_this_node;
	max_path.score_multiplier = 
			score_multiplier * score_multiplier_of_this_node; 
	max_path.inverse_dependency_score = get_inv_dep_score();
	
	//(forward branch and backward branch out of scope have score multiplier = 0)
	max_path.opcodes.push_back(instruction);
	max_path.path_hashes.push_back(subtree_hash);

	//printf("tree %d depth %d\r", tree_id, length);

	//handle children

	// if(typeid(*this)==typeid(InstructionNodeBranch)){ //override instead
	// 	return max_path;
	// }

	Path max_child_path;
	Path child_path;
	for (InstructionNode* child : children) {

		// if(typeid(*child)==typeid(InstructionNodeBranch)){
		// //printf("Found Branch in child\n\n\n");
		// //return max_path;
		
		// }
		child_path = child->extend_path(length+1, 
				max_path.score_bonus, max_path.score_multiplier, tree_id);
		if(max_child_path.length==0){
			max_child_path = child_path;
		}else{
			if(child_path.get_score() > 
					max_child_path.get_score()){
				max_child_path = child_path;
			}
		}
	}

	if (max_child_path.length>0)
	{
		if(max_path.get_score() < max_child_path.get_score()){
			max_path.length = max_child_path.length;
			max_path.minimum_weight = max_child_path.minimum_weight;
			max_path.opcodes.insert(max_path.opcodes.end(), 
							max_child_path.opcodes.begin(), 
							max_child_path.opcodes.end());
			max_path.path_hashes.insert(max_path.path_hashes.end(), 
				max_child_path.path_hashes.begin(), 
				max_child_path.path_hashes.end());
			max_path.inverse_dependency_score += max_child_path.inverse_dependency_score;
		} 
	}

	return max_path;
}

InstructionNodeLeaf::InstructionNodeLeaf(Opcode::Mapping instruction, uint64_t parent_hash, uint64_t pc)
		: InstructionNode(instruction, parent_hash){
	pc_map.insert({pc,1});
	//todo increase/add with weight
}

InstructionNode* InstructionNodeLeaf::insert(Opcode::Mapping op, uint64_t pc, 
								int8_t dependency1, int8_t dependency2, int8_t relative_overwrite, 
								uint8_t depth){
	printf("[Insert ERROR] Maximum depth reached");
	return NULL;
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
				printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
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

			dot_stream << std::dec 
			<< "Error: using leaf node inside tree" << std::endl;
			printf("Error: using leaf node inside tree");
			return name;
		}

		
		//dont process children as this is a leaf node

		return name;
}

MemoryNode::MemoryNode(bool is_store_instruction) : is_store(is_store_instruction){
}

void MemoryNode::register_access(uint64_t address, 
				uint64_t prev_writer, uint64_t prev_reader, 
				uint64_t stackpointer, uint64_t framepointer){
	memory_accesses[address]++;
	uint64_t access_offset = abs((long int)(address-last_access));
	access_offset_sum += access_offset;
	int64_t accessor_diff = 0; 
	if(is_store){
		accessor_diff = prev_writer - last_access;
	}else{
		accessor_diff = prev_reader -last_access;
	}

	if(framepointer>0 && address <= framepointer && address >= stackpointer){
		//address is in Frame
		memory_location = memory_location | MemoryRegion::FRAME;
		printf("FRAME Access %lx\n",address);
	}else{
		if(address<stackpointer){
			//address is not on the Stack
			memory_location = memory_location | MemoryRegion::HEAP;
			printf("HEAP Access %lx\n",address);
		}else{
			//address is on the Stack but not in Frame
			memory_location = memory_location | MemoryRegion::STACK;
			printf("STACK Access %lx\n",address);
		}
	}

	printf("Register access: %lx (%lx | %lx) offset: %ld, diff: %ld", address, prev_writer, prev_reader, access_offset, accessor_diff);
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
		<< "<TR><TD><FONT COLOR=\"0.6 0.6 1.000\" POINT-SIZE=\"10\">" 
		<< std::hex << subtree_hash << std::dec 
		<< "</FONT></TD></TR>"
		<< "<TR><TD><FONT COLOR=\"0.2 0.8 1.000\" POINT-SIZE=\"10\">" 
		<< (int)memory_location
		<< "</FONT></TD></TR></TABLE>" 
		<< ">, color=" 
		<< color_index << "]" 
		<< std::endl;

		connections_stream << parent_name << " -> " << name.str();
		connections_stream << "[label=\"" << weight << "\" decorate=true";

		if(reduce_graph_output && per_weight<branch_omission_threshold){
			int shade = std::min(95,(int)(100.0-((per_weight/branch_omission_threshold)*100.0))) ;
			printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
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

InstructionNodeMemoryLeaf::InstructionNodeMemoryLeaf(Mapping instruction, uint64_t parent_hash, uint64_t pc, 
			uint64_t memory_access, bool is_store_instruction)
			: InstructionNode(instruction, parent_hash), 
			  MemoryNode(is_store_instruction),
			  InstructionNodeLeaf(instruction, parent_hash, pc){
}

void InstructionNodeMemoryLeaf::_print(uint8_t level){
	printf("Not implemented");
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
			<< "<TR><TD><FONT COLOR=\"0.2 0.8 1.000\" POINT-SIZE=\"10\">" 
			<< (int)memory_location
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
				printf("ommitting branch [%lx] with per_weight: %f\n",subtree_hash, per_weight);
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

			dot_stream << std::dec 
			<< "Error: using leaf node inside tree" << std::endl;
			printf("Error: using leaf node inside tree");
			return name;
		}

		
		//dont process children as this is a leaf node

		return name;
}

BranchNode::BranchNode(int64_t offset){
	relative_offsets[offset]++;
	if(offset<0){
		is_backward_jump = true;
	}else{
		is_forward_jump = true;
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

