#include "iss.h"

// to save *cout* format setting, see *ISS::show*
#include <boost/io/ios_state.hpp>
// for safe down-cast
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>

#include <dlfcn.h>
#include <cstdlib>

#include <chrono>

using namespace rv32;

#define RAISE_ILLEGAL_INSTRUCTION() raise_trap(EXC_ILLEGAL_INSTR, instr.data());

#define REQUIRE_ISA(X)          \
    if (!(csrs.misa.reg & X))   \
        RAISE_ILLEGAL_INSTRUCTION()

#define RD instr.rd()
#define RS1 instr.rs1()
#define RS2 instr.rs2()
#define RS3 instr.rs3()

const char *regnames[] = {
    "zero (x0)", "ra   (x1)", "sp   (x2)", "gp   (x3)", "tp   (x4)", "t0   (x5)", "t1   (x6)", "t2   (x7)",
    "s0/fp(x8)", "s1   (x9)", "a0  (x10)", "a1  (x11)", "a2  (x12)", "a3  (x13)", "a4  (x14)", "a5  (x15)",
    "a6  (x16)", "a7  (x17)", "s2  (x18)", "s3  (x19)", "s4  (x20)", "s5  (x21)", "s6  (x22)", "s7  (x23)",
    "s8  (x24)", "s9  (x25)", "s10 (x26)", "s11 (x27)", "t3  (x28)", "t4  (x29)", "t5  (x30)", "t6  (x31)",
};

int regcolors[] = {
#if defined(COLOR_THEME_DARK)
    0,  1,  2,  3,  4,  5,  6,  52, 8,  9,  53, 54, 55, 56, 57, 58,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
#elif defined(COLOR_THEME_LIGHT)
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 153, 154, 155, 156, 157, 158,
    116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131,
#else
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#endif
};

RegFile::RegFile() {
	memset(regs, 0, sizeof(regs));
}

RegFile::RegFile(const RegFile &other) {
	memcpy(regs, other.regs, sizeof(regs));
}

void RegFile::write(uint32_t index, int32_t value) {
	assert(index <= x31);
	assert(index != x0);
	regs[index] = value;
}

int32_t RegFile::read(uint32_t index) {
	if (index > x31)
		throw std::out_of_range("out-of-range register access");
	return regs[index];
}

uint32_t RegFile::shamt(uint32_t index) {
	assert(index <= x31);
	return BIT_RANGE(regs[index], 4, 0);
}

int32_t &RegFile::operator[](const uint32_t idx) {
	return regs[idx];
}

#if defined(COLOR_THEME_LIGHT) || defined(COLOR_THEME_DARK)
#define COLORFRMT "\e[38;5;%um%s\e[39m"
#define COLORPRINT(fmt, data) fmt, data
#else
#define COLORFRMT "%s"
#define COLORPRINT(fmt, data) data
#endif

void RegFile::show() {
	for (unsigned i = 0; i < NUM_REGS; ++i) {
		printf(COLORFRMT " = %8x\n", COLORPRINT(regcolors[i], regnames[i]), regs[i]);
	}
}

ISS::ISS(uint32_t hart_id, const char *output_filestr, const char *input_filestr,std::vector<uint64_t> input_hashes, bool use_E_base_isa) : systemc_name("Core-" + std::to_string(hart_id)) {
	csrs.mhartid.reg = hart_id;
	if (use_E_base_isa)
		csrs.misa.select_E_base_isa();

	sc_core::sc_time qt = tlm::tlm_global_quantum::instance().get();
	cycle_time = sc_core::sc_time(10, sc_core::SC_NS);

	output_filename = output_filestr;
	path_hashes = &input_hashes[0];
	input_filename = input_filestr;

	assert(qt >= cycle_time);
	assert(qt % cycle_time == sc_core::SC_ZERO_TIME);

	for (int i = 0; i < Opcode::NUMBER_OF_INSTRUCTIONS; ++i) instr_cycles[i] = cycle_time;

	const sc_core::sc_time memory_access_cycles = 4 * cycle_time;
	const sc_core::sc_time mul_div_cycles = 8 * cycle_time;

	instr_cycles[Opcode::LB] = memory_access_cycles;
	instr_cycles[Opcode::LBU] = memory_access_cycles;
	instr_cycles[Opcode::LH] = memory_access_cycles;
	instr_cycles[Opcode::LHU] = memory_access_cycles;
	instr_cycles[Opcode::LW] = memory_access_cycles;
	instr_cycles[Opcode::SB] = memory_access_cycles;
	instr_cycles[Opcode::SH] = memory_access_cycles;
	instr_cycles[Opcode::SW] = memory_access_cycles;
	instr_cycles[Opcode::MUL] = mul_div_cycles;
	instr_cycles[Opcode::MULH] = mul_div_cycles;
	instr_cycles[Opcode::MULHU] = mul_div_cycles;
	instr_cycles[Opcode::MULHSU] = mul_div_cycles;
	instr_cycles[Opcode::DIV] = mul_div_cycles;
	instr_cycles[Opcode::DIVU] = mul_div_cycles;
	instr_cycles[Opcode::REM] = mul_div_cycles;
	instr_cycles[Opcode::REMU] = mul_div_cycles;
	op = Opcode::UNDEF;

	ring_buffer_index = 0;

	block_on_wfi(false);
}

void ISS::exec_step() {
	assert(((pc & ~pc_alignment_mask()) == 0) && "misaligned instruction");

	try {
		uint32_t mem_word = instr_mem->load_instr(pc);
		instr = Instruction(mem_word);
	} catch (SimulationTrap &e) {
		op = Opcode::UNDEF;
		instr = Instruction(0);
		throw;
	}

	if (instr.is_compressed()) {
		op = instr.decode_and_expand_compressed(RV32);
		pc += 2;
        if (op != Opcode::UNDEF)
            REQUIRE_ISA(C_ISA_EXT);
    } else {
		op = instr.decode_normal(RV32);
		pc += 4;
	}

	uint64_t cycles_diff = _compute_and_get_current_cycles() - prev_cycles;

//Instead of updateing the entry at the start of the loop with data from the last iteration
//move this part to the end as memory accesses have to be calculated first
//otherwise memory accesses are off by 1
// //---------------------------------------------------------------------------------
// //                             save instruction info of last execution
// //---------------------------------------------------------------------------------
// 		last_executed_steps[ring_buffer_index].last_executed_instruction = op;
// 		last_executed_steps[ring_buffer_index].last_cycles = cycles_diff;
// 		last_executed_steps[ring_buffer_index].last_registers = {RS1,RS2,RD};
// 		last_executed_steps[ring_buffer_index].last_executed_pc = last_pc;
// 		last_executed_steps[ring_buffer_index].last_powermode = 0; //TODO
// 		last_executed_steps[ring_buffer_index].last_memory_read = 0;
// 		last_executed_steps[ring_buffer_index].last_memory_written = 0;
// 		last_executed_steps[ring_buffer_index].last_step_id = total_num_instr;

// 		if(std::get<1>(last_memory_access)==AccessType::STORE){//check if memory was accessed in the last execution step
// 			last_executed_steps[ring_buffer_index].last_memory_written = std::get<0>(last_memory_access); //fetch accessed addresses from persistent variable
// 			if(last_executed_steps[ring_buffer_index].last_memory_written==0){
// 				printf("ERROR ZERO write\n\n\n");
// 			}
// 			printf("WRITE %x (%s at idx:%d)\n",last_executed_steps[ring_buffer_index].last_memory_written, Opcode::mappingStr[op], ring_buffer_index);
// 		}else{
// 			if(std::get<1>(last_memory_access)==AccessType::LOAD)
// 			{
// 			//    printf("LOAD: %s\n", Opcode::mappingStr[op]);
// 			   last_executed_steps[ring_buffer_index].last_memory_read = std::get<0>(last_memory_access);
// 			   printf("LOAD %x, (%s at idx:%d)\n", last_executed_steps[ring_buffer_index].last_memory_read, Opcode::mappingStr[op], ring_buffer_index);
// 			}
			
// 		}
// 		last_memory_access = {0, AccessType::NONE};//reset last memory access

// //-------------------------------------------------------------------------
// //                          
// //-------------------------------------------------------------------------
// 	//updating ringbuffer done
// 	//update index
// 	ring_buffer_index = (ring_buffer_index+1)%INSTRUCTION_TREE_DEPTH;

	//insert into tree of oldest instruction, which will be overwritten in the next step
	Opcode::Mapping oldest_op = last_executed_steps[ring_buffer_index].last_executed_instruction;
	if(oldest_op){//ring buffer is still being filled if this is false
		//check if a tree for this op already exists
		InstructionNodeR* found_tree = NULL;
		for (InstructionNodeR& root : instruction_trees){
			
			if(root.instruction == oldest_op){
				found_tree = &root;
				break;
			}
		}
		if(found_tree!=NULL){
			//printf("-> %d\n",found_tree->instruction);
		}else{
			//printf("first occurance of op %d. Adding to list\n", oldest_op);
			instruction_trees.emplace_back(oldest_op, 0);
			found_tree = &instruction_trees.back();
		}
		//printf("found tree found or created for op %d", found_tree->instruction);

		//insert ringbuffer - this opcode into tree
		found_tree->insert_rb(last_executed_steps, 
							ring_buffer_index);
	}

	if (trace) {
		printf("core %2u: prv %1x: pc %8x: %s ", csrs.mhartid.reg, prv, last_pc, Opcode::mappingStr[op]);
		switch (Opcode::getType(op)) {
			case Opcode::Type::R:
				printf(COLORFRMT ", " COLORFRMT ", " COLORFRMT, COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
				       COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]));
				break;
			case Opcode::Type::I:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
				       COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]), instr.I_imm());
				break;
			case Opcode::Type::S:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]), instr.S_imm());
				break;
			case Opcode::Type::B:
				printf(COLORFRMT ", " COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
				       COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]), instr.B_imm());
				break;
			case Opcode::Type::U:
				printf(COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]), instr.U_imm());
				break;
			case Opcode::Type::J:
				printf(COLORFRMT ", 0x%x", COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]), instr.J_imm());
				break;
			default:;
		}
		puts("");
	}

		//printf("OP: %s\n", Opcode::mappingStr[op]);

	switch (op) {
		case Opcode::UNDEF:
			if (trace)
				std::cout << "[ISS] WARNING: unknown instruction '" << std::to_string(instr.data()) << "' at address '"
				          << std::to_string(last_pc) << "'" << std::endl;
			raise_trap(EXC_ILLEGAL_INSTR, instr.data());
			break;

		case Opcode::ADDI:
			regs[instr.rd()] = regs[instr.rs1()] + instr.I_imm();
			break;

		case Opcode::SLTI:
			regs[instr.rd()] = regs[instr.rs1()] < instr.I_imm();
			break;

		case Opcode::SLTIU:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) < ((uint32_t)instr.I_imm());
			break;

		case Opcode::XORI:
			regs[instr.rd()] = regs[instr.rs1()] ^ instr.I_imm();
			break;

		case Opcode::ORI:
			regs[instr.rd()] = regs[instr.rs1()] | instr.I_imm();
			break;

		case Opcode::ANDI:
			regs[instr.rd()] = regs[instr.rs1()] & instr.I_imm();
			break;

		case Opcode::ADD:
			regs[instr.rd()] = regs[instr.rs1()] + regs[instr.rs2()];
			break;

		case Opcode::SUB:
			regs[instr.rd()] = regs[instr.rs1()] - regs[instr.rs2()];
			break;

		case Opcode::SLL:
			regs[instr.rd()] = regs[instr.rs1()] << regs.shamt(instr.rs2());
			break;

		case Opcode::SLT:
			regs[instr.rd()] = regs[instr.rs1()] < regs[instr.rs2()];
			break;

		case Opcode::SLTU:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) < ((uint32_t)regs[instr.rs2()]);
			break;

		case Opcode::SRL:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> regs.shamt(instr.rs2());
			break;

		case Opcode::SRA:
			regs[instr.rd()] = regs[instr.rs1()] >> regs.shamt(instr.rs2());
			break;

		case Opcode::XOR:
			regs[instr.rd()] = regs[instr.rs1()] ^ regs[instr.rs2()];
			break;

		case Opcode::OR:
			regs[instr.rd()] = regs[instr.rs1()] | regs[instr.rs2()];
			break;

		case Opcode::AND:
			regs[instr.rd()] = regs[instr.rs1()] & regs[instr.rs2()];
			break;

		case Opcode::SLLI:
			regs[instr.rd()] = regs[instr.rs1()] << instr.shamt();
			break;

		case Opcode::SRLI:
			regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> instr.shamt();
			break;

		case Opcode::SRAI:
			regs[instr.rd()] = regs[instr.rs1()] >> instr.shamt();
			break;

		case Opcode::LUI:
			regs[instr.rd()] = instr.U_imm();
			break;

		case Opcode::AUIPC:
			regs[instr.rd()] = last_pc + instr.U_imm();
			break;

		case Opcode::JAL: {
			auto link = pc;
			pc = last_pc + instr.J_imm();
			trap_check_pc_alignment();
			regs[instr.rd()] = link;
		} break;

		case Opcode::JALR: {
			auto link = pc;
			pc = (regs[instr.rs1()] + instr.I_imm()) & ~1;
			trap_check_pc_alignment();
			regs[instr.rd()] = link;
		} break;

		case Opcode::SB: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			mem->store_byte(addr, regs[instr.rs2()]);
			log_memory_store(addr, last_pc);
		} break;

		case Opcode::SH: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<2, false>(addr);
			mem->store_half(addr, regs[instr.rs2()]);
			log_memory_store(addr, last_pc);
		} break;

		case Opcode::SW: {
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<4, false>(addr);
			mem->store_word(addr, regs[instr.rs2()]);
			log_memory_store(addr, last_pc);
			// std::cout << "log " << std::get<0>(last_memory_access) << std::endl;
		} break;

		case Opcode::LB: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			regs[instr.rd()] = mem->load_byte(addr);
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::LH: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<2, true>(addr);
			regs[instr.rd()] = mem->load_half(addr);
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::LW: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<4, true>(addr);
			regs[instr.rd()] = mem->load_word(addr);
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::LBU: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			regs[instr.rd()] = mem->load_ubyte(addr);
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::LHU: {
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<2, true>(addr);
			regs[instr.rd()] = mem->load_uhalf(addr);
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::BEQ:
			if (regs[instr.rs1()] == regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BNE:
			if (regs[instr.rs1()] != regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BLT:
			if (regs[instr.rs1()] < regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BGE:
			if (regs[instr.rs1()] >= regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BLTU:
			if ((uint32_t)regs[instr.rs1()] < (uint32_t)regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::BGEU:
			if ((uint32_t)regs[instr.rs1()] >= (uint32_t)regs[instr.rs2()]) {
				pc = last_pc + instr.B_imm();
				trap_check_pc_alignment();
			}
			break;

		case Opcode::FENCE:
		case Opcode::FENCE_I: {
			// not using out of order execution so can be ignored
		} break;

		case Opcode::ECALL: {
			if (sys) {
				sys->execute_syscall(this);
			} else {
				switch (prv) {
					case MachineMode:
						raise_trap(EXC_ECALL_M_MODE, last_pc);
						break;
					case SupervisorMode:
						raise_trap(EXC_ECALL_S_MODE, last_pc);
						break;
					case UserMode:
						raise_trap(EXC_ECALL_U_MODE, last_pc);
						break;
					default:
						throw std::runtime_error("unknown privilege level " + std::to_string(prv));
				}
			}
		} break;

		case Opcode::EBREAK: {
			// TODO: also raise trap and let the SW deal with it?
			status = CoreExecStatus::HitBreakpoint;
		} break;

		case Opcode::CSRRW: {
			auto addr = instr.csr();
			if (is_invalid_csr_access(addr, true)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[instr.rs1()];
				if (rd != RegFile::zero) {
					regs[instr.rd()] = get_csr_value(addr);
				}
				set_csr_value(addr, rs1_val);
			}
		} break;

		case Opcode::CSRRS: {
			auto addr = instr.csr();
			auto rs1 = instr.rs1();
			auto write = rs1 != RegFile::zero;
			if (is_invalid_csr_access(addr, write)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[rs1];
				auto csr_val = get_csr_value(addr);
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val | rs1_val);
			}
		} break;

		case Opcode::CSRRC: {
			auto addr = instr.csr();
			auto rs1 = instr.rs1();
			auto write = rs1 != RegFile::zero;
			if (is_invalid_csr_access(addr, write)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				auto rs1_val = regs[rs1];
				auto csr_val = get_csr_value(addr);
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val & ~rs1_val);
			}
		} break;

		case Opcode::CSRRWI: {
			auto addr = instr.csr();
			if (is_invalid_csr_access(addr, true)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto rd = instr.rd();
				if (rd != RegFile::zero) {
					regs[rd] = get_csr_value(addr);
				}
				set_csr_value(addr, instr.zimm());
			}
		} break;

		case Opcode::CSRRSI: {
			auto addr = instr.csr();
			auto zimm = instr.zimm();
			auto write = zimm != 0;
			if (is_invalid_csr_access(addr, write)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto csr_val = get_csr_value(addr);
				auto rd = instr.rd();
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val | zimm);
			}
		} break;

		case Opcode::CSRRCI: {
			auto addr = instr.csr();
			auto zimm = instr.zimm();
			auto write = zimm != 0;
			if (is_invalid_csr_access(addr, write)) {
                RAISE_ILLEGAL_INSTRUCTION();
			} else {
				auto csr_val = get_csr_value(addr);
				auto rd = instr.rd();
				if (rd != RegFile::zero)
					regs[rd] = csr_val;
				if (write)
					set_csr_value(addr, csr_val & ~zimm);
			}
		} break;

		case Opcode::MUL: {
            REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
			regs[instr.rd()] = ans & 0xFFFFFFFF;
		} break;

		case Opcode::MULH: {
            REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::MULHU: {
            REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = ((uint64_t)(uint32_t)regs[instr.rs1()]) * (uint64_t)((uint32_t)regs[instr.rs2()]);
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::MULHSU: {
            REQUIRE_ISA(M_ISA_EXT);
			int64_t ans = (int64_t)regs[instr.rs1()] * (uint64_t)((uint32_t)regs[instr.rs2()]);
			regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
		} break;

		case Opcode::DIV: {
            REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = -1;
			} else if (a == REG_MIN && b == -1) {
				regs[instr.rd()] = a;
			} else {
				regs[instr.rd()] = a / b;
			}
		} break;

		case Opcode::DIVU: {
            REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = -1;
			} else {
				regs[instr.rd()] = (uint32_t)a / (uint32_t)b;
			}
		} break;

		case Opcode::REM: {
            REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = a;
			} else if (a == REG_MIN && b == -1) {
				regs[instr.rd()] = 0;
			} else {
				regs[instr.rd()] = a % b;
			}
		} break;

		case Opcode::REMU: {
            REQUIRE_ISA(M_ISA_EXT);
			auto a = regs[instr.rs1()];
			auto b = regs[instr.rs2()];
			if (b == 0) {
				regs[instr.rd()] = a;
			} else {
				regs[instr.rd()] = (uint32_t)a % (uint32_t)b;
			}
		} break;

		case Opcode::LR_W: {
            REQUIRE_ISA(A_ISA_EXT);
			uint32_t addr = regs[instr.rs1()];
			trap_check_addr_alignment<4, true>(addr);
			regs[instr.rd()] = mem->atomic_load_reserved_word(addr);
			if (lr_sc_counter == 0)
			    lr_sc_counter = 17;  // this instruction + 16 additional ones, (an over-approximation) to cover the RISC-V forward progress property
		} break;

		case Opcode::SC_W: {
            REQUIRE_ISA(A_ISA_EXT);
			uint32_t addr = regs[instr.rs1()];
			trap_check_addr_alignment<4, false>(addr);
			uint32_t val = regs[instr.rs2()];
			regs[instr.rd()] = 1;  // failure by default (in case a trap is thrown)
			regs[instr.rd()] = mem->atomic_store_conditional_word(addr, val) ? 0 : 1;  // overwrite result (in case no trap is thrown)
			lr_sc_counter = 0;
		} break;

		case Opcode::AMOSWAP_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) {
				(void)a;
				return b;
			});
		} break;

		case Opcode::AMOADD_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a + b; });
		} break;

		case Opcode::AMOXOR_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a ^ b; });
		} break;

		case Opcode::AMOAND_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a & b; });
		} break;

		case Opcode::AMOOR_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return a | b; });
		} break;

		case Opcode::AMOMIN_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::min(a, b); });
		} break;

		case Opcode::AMOMINU_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::min((uint32_t)a, (uint32_t)b); });
		} break;

		case Opcode::AMOMAX_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::max(a, b); });
		} break;

		case Opcode::AMOMAXU_W: {
            REQUIRE_ISA(A_ISA_EXT);
			execute_amo(instr, [](int32_t a, int32_t b) { return std::max((uint32_t)a, (uint32_t)b); });
		} break;

			// RV32F Extension

		case Opcode::FLW: {
            REQUIRE_ISA(F_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.I_imm();
			trap_check_addr_alignment<4, true>(addr);
			fp_regs.write(RD, float32_t{(uint32_t)mem->load_word(addr)});
			log_memory_read(addr, last_pc);
		} break;

		case Opcode::FSW: {
            REQUIRE_ISA(F_ISA_EXT);
			uint32_t addr = regs[instr.rs1()] + instr.S_imm();
			trap_check_addr_alignment<4, false>(addr);
            mem->store_word(addr, fp_regs.u32(RS2));
			log_memory_store(addr, last_pc);
		} break;

		case Opcode::FADD_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_add(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSUB_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_sub(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FMUL_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mul(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FDIV_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_div(fp_regs.f32(RS1), fp_regs.f32(RS2)));
			fp_finish_instr();
		} break;

		case Opcode::FSQRT_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_sqrt(fp_regs.f32(RS1)));
			fp_finish_instr();
		} break;

		case Opcode::FMIN_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();

			bool rs1_smaller = f32_lt_quiet(fp_regs.f32(RS1), fp_regs.f32(RS2)) ||
			                   (f32_eq(fp_regs.f32(RS1), fp_regs.f32(RS2)) && f32_isNegative(fp_regs.f32(RS1)));

			if (f32_isNaN(fp_regs.f32(RS1)) && f32_isNaN(fp_regs.f32(RS2))) {
				fp_regs.write(RD, f32_defaultNaN);
			} else {
				if (rs1_smaller)
					fp_regs.write(RD, fp_regs.f32(RS1));
				else
					fp_regs.write(RD, fp_regs.f32(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMAX_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();

			bool rs1_greater = f32_lt_quiet(fp_regs.f32(RS2), fp_regs.f32(RS1)) ||
			                   (f32_eq(fp_regs.f32(RS2), fp_regs.f32(RS1)) && f32_isNegative(fp_regs.f32(RS2)));

			if (f32_isNaN(fp_regs.f32(RS1)) && f32_isNaN(fp_regs.f32(RS2))) {
				fp_regs.write(RD, f32_defaultNaN);
			} else {
				if (rs1_greater)
					fp_regs.write(RD, fp_regs.f32(RS1));
				else
					fp_regs.write(RD, fp_regs.f32(RS2));
			}

			fp_finish_instr();
		} break;

		case Opcode::FMADD_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(fp_regs.f32(RS1), fp_regs.f32(RS2), fp_regs.f32(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FMSUB_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(fp_regs.f32(RS1), fp_regs.f32(RS2), f32_neg(fp_regs.f32(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMADD_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(f32_neg(fp_regs.f32(RS1)), fp_regs.f32(RS2), f32_neg(fp_regs.f32(RS3))));
			fp_finish_instr();
		} break;

		case Opcode::FNMSUB_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, f32_mulAdd(f32_neg(fp_regs.f32(RS1)), fp_regs.f32(RS2), fp_regs.f32(RS3)));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_W_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = f32_to_i32(fp_regs.f32(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_WU_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			regs[RD] = f32_to_ui32(fp_regs.f32(RS1), softfloat_roundingMode, true);
			fp_finish_instr();
		} break;

		case Opcode::FCVT_S_W: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, i32_to_f32(regs[RS1]));
			fp_finish_instr();
		} break;

		case Opcode::FCVT_S_WU: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_setup_rm();
			fp_regs.write(RD, ui32_to_f32(regs[RS1]));
			fp_finish_instr();
		} break;

		case Opcode::FSGNJ_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{(f1.v & ~F32_SIGN_BIT) | (f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJN_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{(f1.v & ~F32_SIGN_BIT) | (~f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FSGNJX_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			auto f1 = fp_regs.f32(RS1);
			auto f2 = fp_regs.f32(RS2);
			fp_regs.write(RD, float32_t{f1.v ^ (f2.v & F32_SIGN_BIT)});
			fp_set_dirty();
		} break;

		case Opcode::FMV_W_X: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			fp_regs.write(RD, float32_t{(uint32_t)regs[RS1]});
			fp_set_dirty();
		} break;

		case Opcode::FMV_X_W: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = fp_regs.u32(RS1);
		} break;

		case Opcode::FEQ_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_eq(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLT_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_lt(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FLE_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_le(fp_regs.f32(RS1), fp_regs.f32(RS2));
			fp_update_exception_flags();
		} break;

		case Opcode::FCLASS_S: {
            REQUIRE_ISA(F_ISA_EXT);
			fp_prepare_instr();
			regs[RD] = f32_classify(fp_regs.f32(RS1));
		} break;

			// RV32D Extension

        case Opcode::FLD: {
            REQUIRE_ISA(D_ISA_EXT);
            uint32_t addr = regs[instr.rs1()] + instr.I_imm();
            trap_check_addr_alignment<8, true>(addr);
            fp_regs.write(RD, float64_t{(uint64_t)mem->load_double(addr)});
        } break;

        case Opcode::FSD: {
            REQUIRE_ISA(D_ISA_EXT);
            uint32_t addr = regs[instr.rs1()] + instr.S_imm();
            trap_check_addr_alignment<8, false>(addr);
            mem->store_double(addr, fp_regs.f64(RS2).v);
        } break;

        case Opcode::FADD_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_add(fp_regs.f64(RS1), fp_regs.f64(RS2)));
            fp_finish_instr();
        } break;

        case Opcode::FSUB_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_sub(fp_regs.f64(RS1), fp_regs.f64(RS2)));
            fp_finish_instr();
        } break;

        case Opcode::FMUL_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_mul(fp_regs.f64(RS1), fp_regs.f64(RS2)));
            fp_finish_instr();
        } break;

        case Opcode::FDIV_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_div(fp_regs.f64(RS1), fp_regs.f64(RS2)));
            fp_finish_instr();
        } break;

        case Opcode::FSQRT_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_sqrt(fp_regs.f64(RS1)));
            fp_finish_instr();
        } break;

        case Opcode::FMIN_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();

            bool rs1_smaller = f64_lt_quiet(fp_regs.f64(RS1), fp_regs.f64(RS2)) ||
                               (f64_eq(fp_regs.f64(RS1), fp_regs.f64(RS2)) && f64_isNegative(fp_regs.f64(RS1)));

            if (f64_isNaN(fp_regs.f64(RS1)) && f64_isNaN(fp_regs.f64(RS2))) {
                fp_regs.write(RD, f64_defaultNaN);
            } else {
                if (rs1_smaller)
                    fp_regs.write(RD, fp_regs.f64(RS1));
                else
                    fp_regs.write(RD, fp_regs.f64(RS2));
            }

            fp_finish_instr();
        } break;

        case Opcode::FMAX_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();

            bool rs1_greater = f64_lt_quiet(fp_regs.f64(RS2), fp_regs.f64(RS1)) ||
                               (f64_eq(fp_regs.f64(RS2), fp_regs.f64(RS1)) && f64_isNegative(fp_regs.f64(RS2)));

            if (f64_isNaN(fp_regs.f64(RS1)) && f64_isNaN(fp_regs.f64(RS2))) {
                fp_regs.write(RD, f64_defaultNaN);
            } else {
                if (rs1_greater)
                    fp_regs.write(RD, fp_regs.f64(RS1));
                else
                    fp_regs.write(RD, fp_regs.f64(RS2));
            }

            fp_finish_instr();
        } break;

        case Opcode::FMADD_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_mulAdd(fp_regs.f64(RS1), fp_regs.f64(RS2), fp_regs.f64(RS3)));
            fp_finish_instr();
        } break;

        case Opcode::FMSUB_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_mulAdd(fp_regs.f64(RS1), fp_regs.f64(RS2), f64_neg(fp_regs.f64(RS3))));
            fp_finish_instr();
        } break;

        case Opcode::FNMADD_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_mulAdd(f64_neg(fp_regs.f64(RS1)), fp_regs.f64(RS2), f64_neg(fp_regs.f64(RS3))));
            fp_finish_instr();
        } break;

        case Opcode::FNMSUB_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_mulAdd(f64_neg(fp_regs.f64(RS1)), fp_regs.f64(RS2), fp_regs.f64(RS3)));
            fp_finish_instr();
        } break;

        case Opcode::FSGNJ_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            auto f1 = fp_regs.f64(RS1);
            auto f2 = fp_regs.f64(RS2);
            fp_regs.write(RD, float64_t{(f1.v & ~F64_SIGN_BIT) | (f2.v & F64_SIGN_BIT)});
            fp_set_dirty();
        } break;

        case Opcode::FSGNJN_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            auto f1 = fp_regs.f64(RS1);
            auto f2 = fp_regs.f64(RS2);
            fp_regs.write(RD, float64_t{(f1.v & ~F64_SIGN_BIT) | (~f2.v & F64_SIGN_BIT)});
            fp_set_dirty();
        } break;

        case Opcode::FSGNJX_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            auto f1 = fp_regs.f64(RS1);
            auto f2 = fp_regs.f64(RS2);
            fp_regs.write(RD, float64_t{f1.v ^ (f2.v & F64_SIGN_BIT)});
            fp_set_dirty();
        } break;

        case Opcode::FCVT_S_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f64_to_f32(fp_regs.f64(RS1)));
            fp_finish_instr();
        } break;

        case Opcode::FCVT_D_S: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, f32_to_f64(fp_regs.f32(RS1)));
            fp_finish_instr();
        } break;

        case Opcode::FEQ_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            regs[RD] = f64_eq(fp_regs.f64(RS1), fp_regs.f64(RS2));
            fp_update_exception_flags();
        } break;

        case Opcode::FLT_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            regs[RD] = f64_lt(fp_regs.f64(RS1), fp_regs.f64(RS2));
            fp_update_exception_flags();
        } break;

        case Opcode::FLE_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            regs[RD] = f64_le(fp_regs.f64(RS1), fp_regs.f64(RS2));
            fp_update_exception_flags();
        } break;

        case Opcode::FCLASS_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            regs[RD] = (int64_t)f64_classify(fp_regs.f64(RS1));
        } break;

        case Opcode::FCVT_W_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            regs[RD] = f64_to_i32(fp_regs.f64(RS1), softfloat_roundingMode, true);
            fp_finish_instr();
        } break;

        case Opcode::FCVT_WU_D: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            regs[RD] = (int32_t)f64_to_ui32(fp_regs.f64(RS1), softfloat_roundingMode, true);
            fp_finish_instr();
        } break;

        case Opcode::FCVT_D_W: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, i32_to_f64((int32_t)regs[RS1]));
            fp_finish_instr();
        } break;

        case Opcode::FCVT_D_WU: {
            REQUIRE_ISA(D_ISA_EXT);
            fp_prepare_instr();
            fp_setup_rm();
            fp_regs.write(RD, ui32_to_f64((int32_t)regs[RS1]));
            fp_finish_instr();
        } break;

        // privileged instructions

        case Opcode::WFI:
            // NOTE: only a hint, can be implemented as NOP
            // std::cout << "[sim:wfi] CSR mstatus.mie " << csrs.mstatus->mie << std::endl;
            release_lr_sc_reservation();

            if (s_mode() && csrs.mstatus.fields.tw)
                raise_trap(EXC_ILLEGAL_INSTR, instr.data());

            if (u_mode() && csrs.misa.has_supervisor_mode_extension())
                raise_trap(EXC_ILLEGAL_INSTR, instr.data());

            if (!ignore_wfi && !has_local_pending_enabled_interrupts())
                sc_core::wait(wfi_event);
            break;

        case Opcode::SFENCE_VMA:
            if (s_mode() && csrs.mstatus.fields.tvm)
                raise_trap(EXC_ILLEGAL_INSTR, instr.data());
            mem->flush_tlb();
            break;

        case Opcode::URET:
            if (!csrs.misa.has_user_mode_extension())
                raise_trap(EXC_ILLEGAL_INSTR, instr.data());
            return_from_trap_handler(UserMode);
            break;

        case Opcode::SRET:
            if (!csrs.misa.has_supervisor_mode_extension() || (s_mode() && csrs.mstatus.fields.tsr))
                raise_trap(EXC_ILLEGAL_INSTR, instr.data());
            return_from_trap_handler(SupervisorMode);
            break;

        case Opcode::MRET:
            return_from_trap_handler(MachineMode);
            break;

            // instructions accepted by decoder but not by this RV32IMACF ISS -> do normal trap
            // RV64I
        case Opcode::LWU:
        case Opcode::LD:
        case Opcode::SD:
        case Opcode::ADDIW:
        case Opcode::SLLIW:
        case Opcode::SRLIW:
        case Opcode::SRAIW:
        case Opcode::ADDW:
        case Opcode::SUBW:
        case Opcode::SLLW:
        case Opcode::SRLW:
        case Opcode::SRAW:
            // RV64M
        case Opcode::MULW:
        case Opcode::DIVW:
        case Opcode::DIVUW:
        case Opcode::REMW:
        case Opcode::REMUW:
            // RV64A
        case Opcode::LR_D:
        case Opcode::SC_D:
        case Opcode::AMOSWAP_D:
        case Opcode::AMOADD_D:
        case Opcode::AMOXOR_D:
        case Opcode::AMOAND_D:
        case Opcode::AMOOR_D:
        case Opcode::AMOMIN_D:
        case Opcode::AMOMAX_D:
        case Opcode::AMOMINU_D:
        case Opcode::AMOMAXU_D:
            // RV64F
        case Opcode::FCVT_L_S:
        case Opcode::FCVT_LU_S:
        case Opcode::FCVT_S_L:
        case Opcode::FCVT_S_LU:
            // RV64D
        case Opcode::FCVT_L_D:
        case Opcode::FCVT_LU_D:
        case Opcode::FMV_X_D:
        case Opcode::FCVT_D_L:
        case Opcode::FCVT_D_LU:
        case Opcode::FMV_D_X:
            RAISE_ILLEGAL_INSTRUCTION();
            break;

        default:
			printf("ERROR: unsupported instruction[" BYTE_TO_BINARY_PATTERN "] at address %x", 
              BYTE_TO_BINARY(instr.opcode()),last_pc);
            throw std::runtime_error("unknown opcode");
	}

//---------------------------------------------------------------------------------
//                             save instruction info of last execution
//---------------------------------------------------------------------------------
		last_executed_steps[ring_buffer_index].last_executed_instruction = op;
		last_executed_steps[ring_buffer_index].last_cycles = cycles_diff;
		last_executed_steps[ring_buffer_index].last_registers = {RS1,RS2,RD};
		last_executed_steps[ring_buffer_index].last_executed_pc = last_pc;
		last_executed_steps[ring_buffer_index].last_powermode = 0; //TODO
		last_executed_steps[ring_buffer_index].last_memory_read = 0;
		last_executed_steps[ring_buffer_index].last_memory_written = 0;
		last_executed_steps[ring_buffer_index].last_step_id = total_num_instr;
		last_executed_steps[ring_buffer_index].last_stack_pointer = regs[RegFile::sp];
		last_executed_steps[ring_buffer_index].last_frame_pointer = regs[RegFile::fp];

		last_executed_steps[ring_buffer_index].last_memory_access_type = std::get<1>(last_memory_access);
		if(std::get<1>(last_memory_access)==AccessType::STORE){//check if memory was accessed in the last execution step
			last_executed_steps[ring_buffer_index].last_memory_written = std::get<0>(last_memory_access); //fetch accessed addresses from persistent variable
			if(last_executed_steps[ring_buffer_index].last_memory_written==0){
				printf("ERROR ZERO write\n\n\n");
			}
			#ifdef debug_dependencies
			printf("WRITE %lx (%s at idx:%d)\n",last_executed_steps[ring_buffer_index].last_memory_written, Opcode::mappingStr[op], ring_buffer_index);
			#endif
		}else{
			if(std::get<1>(last_memory_access)==AccessType::LOAD)
			{
			//    printf("LOAD: %s\n", Opcode::mappingStr[op]);
			   last_executed_steps[ring_buffer_index].last_memory_read = std::get<0>(last_memory_access);
			   #ifdef debug_dependencies
			   printf("LOAD %lx, (%s at idx:%d)\n", last_executed_steps[ring_buffer_index].last_memory_read, Opcode::mappingStr[op], ring_buffer_index);
			   #endif
			}
			
		}
		last_memory_access = {0, AccessType::NONE};//reset last memory access

//-------------------------------------------------------------------------
//                          
//-------------------------------------------------------------------------
	//updating ringbuffer done
	//update index
	ring_buffer_index = (ring_buffer_index+1)%INSTRUCTION_TREE_DEPTH;

}

uint64_t ISS::_compute_and_get_current_cycles() {
	assert(cycle_counter % cycle_time == sc_core::SC_ZERO_TIME);
	assert(cycle_counter.value() % cycle_time.value() == 0);

	uint64_t num_cycles = cycle_counter.value() / cycle_time.value();

	return num_cycles;
}


bool ISS::is_invalid_csr_access(uint32_t csr_addr, bool is_write) {
    if (csr_addr == csr::FFLAGS_ADDR || csr_addr == csr::FRM_ADDR || csr_addr == csr::FCSR_ADDR) {
        REQUIRE_ISA(F_ISA_EXT);
    }
    PrivilegeLevel csr_prv = (0x300 & csr_addr) >> 8;
    bool csr_readonly = ((0xC00 & csr_addr) >> 10) == 3;
    bool s_invalid = (csr_prv == SupervisorMode) && !csrs.misa.has_supervisor_mode_extension();
    bool u_invalid = (csr_prv == UserMode) && !csrs.misa.has_user_mode_extension();
    return (is_write && csr_readonly) || (prv < csr_prv) || s_invalid || u_invalid;
}


void ISS::validate_csr_counter_read_access_rights(uint32_t addr) {
	// match against counter CSR addresses, see RISC-V privileged spec for the address definitions
	if ((addr >= 0xC00 && addr <= 0xC1F) || (addr >= 0xC80 && addr <= 0xC9F)) {
		auto cnt = addr & 0x1F;  // 32 counter in total, naturally aligned with the mcounteren and scounteren CSRs

		if (s_mode() && !csr::is_bitset(csrs.mcounteren, cnt))
			RAISE_ILLEGAL_INSTRUCTION();

		if (u_mode() && (!csr::is_bitset(csrs.mcounteren, cnt) || !csr::is_bitset(csrs.scounteren, cnt)))
			RAISE_ILLEGAL_INSTRUCTION();
	}
}

uint32_t ISS::get_csr_value(uint32_t addr) {
	validate_csr_counter_read_access_rights(addr);

	auto read = [=](auto &x, uint32_t mask) { return x.reg & mask; };

	using namespace csr;

	switch (addr) {
		case TIME_ADDR:
		case MTIME_ADDR: {
			uint64_t mtime = clint->update_and_get_mtime();
			csrs.time.reg = mtime;
			return csrs.time.words.low;
		}

		case TIMEH_ADDR:
		case MTIMEH_ADDR: {
			uint64_t mtime = clint->update_and_get_mtime();
			csrs.time.reg = mtime;
			return csrs.time.words.high;
		}

		case MCYCLE_ADDR:
			csrs.cycle.reg = _compute_and_get_current_cycles();
			return csrs.cycle.words.low;

		case MCYCLEH_ADDR:
			csrs.cycle.reg = _compute_and_get_current_cycles();
			return csrs.cycle.words.high;

		case MINSTRET_ADDR:
			return csrs.instret.words.low;

		case MINSTRETH_ADDR:
			return csrs.instret.words.high;

		SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:  // not implemented
			return 0;

		case MSTATUS_ADDR:
			return read(csrs.mstatus, MSTATUS_MASK);
		case SSTATUS_ADDR:
			return read(csrs.mstatus, SSTATUS_MASK);
		case USTATUS_ADDR:
			return read(csrs.mstatus, USTATUS_MASK);

		case MIP_ADDR:
			return read(csrs.mip, MIP_READ_MASK);
		case SIP_ADDR:
			return read(csrs.mip, SIP_MASK);
		case UIP_ADDR:
			return read(csrs.mip, UIP_MASK);

		case MIE_ADDR:
			return read(csrs.mie, MIE_MASK);
		case SIE_ADDR:
			return read(csrs.mie, SIE_MASK);
		case UIE_ADDR:
			return read(csrs.mie, UIE_MASK);

		case SATP_ADDR:
			if (csrs.mstatus.fields.tvm)
				RAISE_ILLEGAL_INSTRUCTION();
			break;

		case FCSR_ADDR:
			return read(csrs.fcsr, FCSR_MASK);

		case FFLAGS_ADDR:
			return csrs.fcsr.fields.fflags;

		case FRM_ADDR:
			return csrs.fcsr.fields.frm;

        // debug CSRs not supported, thus hardwired
        case TSELECT_ADDR:
            return 1; // if a zero write by SW is preserved, then debug mode is supported (thus hardwire to non-zero)
        case TDATA1_ADDR:
        case TDATA2_ADDR:
        case TDATA3_ADDR:
        case DCSR_ADDR:
        case DPC_ADDR:
        case DSCRATCH0_ADDR:
        case DSCRATCH1_ADDR:
            return 0;
	}

	if (!csrs.is_valid_csr32_addr(addr))
		RAISE_ILLEGAL_INSTRUCTION();

	return csrs.default_read32(addr);
}

void ISS::set_csr_value(uint32_t addr, uint32_t value) {
	auto write = [=](auto &x, uint32_t mask) { x.reg = (x.reg & ~mask) | (value & mask); };

	using namespace csr;

	switch (addr) {
		case MISA_ADDR:                         // currently, read-only, thus cannot be changed at runtime
		SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:  // not implemented
			break;

        case SATP_ADDR: {
            if (csrs.mstatus.fields.tvm)
                RAISE_ILLEGAL_INSTRUCTION();
            write(csrs.satp, SATP_MASK);
            // std::cout << "[iss] satp=" << boost::format("%x") % csrs.satp.reg << std::endl;
        } break;

		case MTVEC_ADDR:
			write(csrs.mtvec, MTVEC_MASK);
			break;
		case STVEC_ADDR:
			write(csrs.stvec, MTVEC_MASK);
			break;
		case UTVEC_ADDR:
			write(csrs.utvec, MTVEC_MASK);
			break;

		case MEPC_ADDR:
			write(csrs.mepc, pc_alignment_mask());
			break;
		case SEPC_ADDR:
			write(csrs.sepc, pc_alignment_mask());
			break;
		case UEPC_ADDR:
			write(csrs.uepc, pc_alignment_mask());
			break;

		case MSTATUS_ADDR:
			write(csrs.mstatus, MSTATUS_MASK);
			break;
		case SSTATUS_ADDR:
			write(csrs.mstatus, SSTATUS_MASK);
			break;
		case USTATUS_ADDR:
			write(csrs.mstatus, USTATUS_MASK);
			break;

		case MIP_ADDR:
			write(csrs.mip, MIP_WRITE_MASK);
			break;
		case SIP_ADDR:
			write(csrs.mip, SIP_MASK);
			break;
		case UIP_ADDR:
			write(csrs.mip, UIP_MASK);
			break;

		case MIE_ADDR:
			write(csrs.mie, MIE_MASK);
			break;
		case SIE_ADDR:
			write(csrs.mie, SIE_MASK);
			break;
		case UIE_ADDR:
			write(csrs.mie, UIE_MASK);
			break;

		case MIDELEG_ADDR:
			write(csrs.mideleg, MIDELEG_MASK);
			break;

		case MEDELEG_ADDR:
			write(csrs.medeleg, MEDELEG_MASK);
			break;

		case SIDELEG_ADDR:
			write(csrs.sideleg, SIDELEG_MASK);
			break;

		case SEDELEG_ADDR:
			write(csrs.sedeleg, SEDELEG_MASK);
			break;

		case MCOUNTEREN_ADDR:
			write(csrs.mcounteren, MCOUNTEREN_MASK);
			break;

		case SCOUNTEREN_ADDR:
			write(csrs.scounteren, MCOUNTEREN_MASK);
			break;

		case MCOUNTINHIBIT_ADDR:
			write(csrs.mcountinhibit, MCOUNTINHIBIT_MASK);
			break;

		case FCSR_ADDR:
			write(csrs.fcsr, FCSR_MASK);
			break;

		case FFLAGS_ADDR:
			csrs.fcsr.fields.fflags = value;
			break;

		case FRM_ADDR:
			csrs.fcsr.fields.frm = value;
			break;

        // debug CSRs not supported, thus hardwired
        case TSELECT_ADDR:
        case TDATA1_ADDR:
        case TDATA2_ADDR:
        case TDATA3_ADDR:
        case DCSR_ADDR:
        case DPC_ADDR:
        case DSCRATCH0_ADDR:
        case DSCRATCH1_ADDR:
            break;

		default:
			if (!csrs.is_valid_csr32_addr(addr))
				RAISE_ILLEGAL_INSTRUCTION();

			csrs.default_write32(addr, value);
	}
}

void ISS::init(instr_memory_if *instr_mem, data_memory_if *data_mem, clint_if *clint, uint32_t entrypoint,
               uint32_t sp) {
	this->instr_mem = instr_mem;
	this->mem = data_mem;
	this->clint = clint;
	regs[RegFile::sp] = sp;
	pc = entrypoint;
}

void ISS::sys_exit() {
	shall_exit = true;
}

unsigned ISS::get_syscall_register_index() {
	if (csrs.misa.has_E_base_isa())
		return RegFile::a5;
	else
		return RegFile::a7;
}


uint64_t ISS::read_register(unsigned idx) {
	return (uint32_t)regs.read(idx);    //NOTE: zero extend
}

void ISS::write_register(unsigned idx, uint64_t value) {
	// Since the value parameter in the function prototype is
	// a uint64_t, signed integer values (e.g. -1) get promoted
	// to values within this range. For example, -1 would be
	// promoted to (2**64)-1. As such, we cannot perform a
	// Boost lexical or numeric cast to uint32_t here as
	// these perform bounds checks. Instead, we perform a C
	// cast without bounds checks.
	regs.write(idx, (uint32_t)value);
}

uint64_t ISS::get_progam_counter(void) {
    return pc;
}

void ISS::block_on_wfi(bool block) {
    ignore_wfi = !block;
}

CoreExecStatus ISS::get_status(void) {
    return status;
}

void ISS::set_status(CoreExecStatus s) {
    status = s;
}

void ISS::enable_debug(void) {
    debug_mode = true;
}

void ISS::insert_breakpoint(uint64_t addr) {
    breakpoints.insert(addr);
}

void ISS::remove_breakpoint(uint64_t addr) {
    breakpoints.erase(addr);
}

uint64_t ISS::get_hart_id() {
    return csrs.mhartid.reg;
}

std::vector<uint64_t> ISS::get_registers(void) {
    std::vector<uint64_t> regvals;

    for (auto v : regs.regs)
        regvals.push_back((uint32_t)v); //NOTE: zero extend

    return regvals;
}


void ISS::fp_finish_instr() {
	fp_set_dirty();
	fp_update_exception_flags();
}

void ISS::fp_prepare_instr() {
	assert(softfloat_exceptionFlags == 0);
	fp_require_not_off();
}

void ISS::fp_set_dirty() {
	csrs.mstatus.fields.sd = 1;
	csrs.mstatus.fields.fs = FS_DIRTY;
}

void ISS::fp_update_exception_flags() {
	if (softfloat_exceptionFlags) {
		fp_set_dirty();
		csrs.fcsr.fields.fflags |= softfloat_exceptionFlags;
		softfloat_exceptionFlags = 0;
	}
}

void ISS::fp_setup_rm() {
	auto rm = instr.frm();
	if (rm == FRM_DYN)
		rm = csrs.fcsr.fields.frm;
	if (rm >= FRM_RMM)
		RAISE_ILLEGAL_INSTRUCTION();
	softfloat_roundingMode = rm;
}

void ISS::fp_require_not_off() {
	if (csrs.mstatus.fields.fs == FS_OFF)
		RAISE_ILLEGAL_INSTRUCTION();
}

void ISS::return_from_trap_handler(PrivilegeLevel return_mode) {
	switch (return_mode) {
		case MachineMode:
			prv = csrs.mstatus.fields.mpp;
			csrs.mstatus.fields.mie = csrs.mstatus.fields.mpie;
			csrs.mstatus.fields.mpie = 1;
			pc = csrs.mepc.reg;
			if (csrs.misa.has_user_mode_extension())
				csrs.mstatus.fields.mpp = UserMode;
			else
				csrs.mstatus.fields.mpp = MachineMode;
			break;

		case SupervisorMode:
			prv = csrs.mstatus.fields.spp;
			csrs.mstatus.fields.sie = csrs.mstatus.fields.spie;
			csrs.mstatus.fields.spie = 1;
			pc = csrs.sepc.reg;
			if (csrs.misa.has_user_mode_extension())
				csrs.mstatus.fields.spp = UserMode;
			else
				csrs.mstatus.fields.spp = SupervisorMode;
			break;

		case UserMode:
			prv = UserMode;
			csrs.mstatus.fields.uie = csrs.mstatus.fields.upie;
			csrs.mstatus.fields.upie = 1;
			pc = csrs.uepc.reg;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(return_mode));
	}

	if (trace)
		printf("[vp::iss] return from trap handler, time %s, pc %8x, prv %1x\n",
		       quantum_keeper.get_current_time().to_string().c_str(), pc, prv);
}

void ISS::trigger_external_interrupt(PrivilegeLevel level) {
	if (trace)
		std::cout << "[vp::iss] trigger external interrupt, " << sc_core::sc_time_stamp() << std::endl;

	switch (level) {
		case UserMode:
			csrs.mip.fields.ueip = true;
			break;
		case SupervisorMode:
			csrs.mip.fields.seip = true;
			break;
		case MachineMode:
			csrs.mip.fields.meip = true;
			break;
	}

	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

void ISS::clear_external_interrupt(PrivilegeLevel level) {
	if (trace)
		std::cout << "[vp::iss] clear external interrupt, " << sc_core::sc_time_stamp() << std::endl;

	switch (level) {
		case UserMode:
			csrs.mip.fields.ueip = false;
			break;
		case SupervisorMode:
			csrs.mip.fields.seip = false;
			break;
		case MachineMode:
			csrs.mip.fields.meip = false;
			break;
	}
}

void ISS::trigger_timer_interrupt(bool status) {
	if (trace)
		std::cout << "[vp::iss] trigger timer interrupt=" << status << ", " << sc_core::sc_time_stamp() << std::endl;
	csrs.mip.fields.mtip = status;
	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

void ISS::trigger_software_interrupt(bool status) {
	if (trace)
		std::cout << "[vp::iss] trigger software interrupt=" << status << ", " << sc_core::sc_time_stamp() << std::endl;
	csrs.mip.fields.msip = status;
	wfi_event.notify(sc_core::SC_ZERO_TIME);
}

PrivilegeLevel ISS::prepare_trap(SimulationTrap &e) {
	// undo any potential pc update (for traps the pc should point to the originating instruction and not it's
	// successor)
	pc = last_pc;
	unsigned exc_bit = (1 << e.reason);

	// 1) machine mode execution takes any traps, independent of delegation setting
	// 2) non-delegated traps are processed in machine mode, independent of current execution mode
	if (prv == MachineMode || !(exc_bit & csrs.medeleg.reg)) {
		csrs.mcause.fields.interrupt = 0;
		csrs.mcause.fields.exception_code = e.reason;
		csrs.mtval.reg = boost::lexical_cast<uint32_t>(e.mtval);
		return MachineMode;
	}

	// see above machine mode comment
	if (prv == SupervisorMode || !(exc_bit & csrs.sedeleg.reg)) {
		csrs.scause.fields.interrupt = 0;
		csrs.scause.fields.exception_code = e.reason;
		csrs.stval.reg = boost::lexical_cast<uint32_t>(e.mtval);
		return SupervisorMode;
	}

	assert(prv == UserMode && (exc_bit & csrs.medeleg.reg) && (exc_bit & csrs.sedeleg.reg));
	csrs.ucause.fields.interrupt = 0;
	csrs.ucause.fields.exception_code = e.reason;
	csrs.utval.reg = boost::lexical_cast<uint32_t>(e.mtval);
	return UserMode;
}

void ISS::prepare_interrupt(const PendingInterrupts &e) {
	if (trace) {
		std::cout << "[vp::iss] prepare interrupt, pending=" << e.pending << ", target-mode=" << e.target_mode
		          << std::endl;
	}

	csr_mip x{e.pending};

	ExceptionCode exc;
	if (x.fields.meip)
		exc = EXC_M_EXTERNAL_INTERRUPT;
	else if (x.fields.msip)
		exc = EXC_M_SOFTWARE_INTERRUPT;
	else if (x.fields.mtip)
		exc = EXC_M_TIMER_INTERRUPT;
	else if (x.fields.seip)
		exc = EXC_S_EXTERNAL_INTERRUPT;
	else if (x.fields.ssip)
		exc = EXC_S_SOFTWARE_INTERRUPT;
	else if (x.fields.stip)
		exc = EXC_S_TIMER_INTERRUPT;
	else if (x.fields.ueip)
		exc = EXC_U_EXTERNAL_INTERRUPT;
	else if (x.fields.usip)
		exc = EXC_U_SOFTWARE_INTERRUPT;
	else if (x.fields.utip)
		exc = EXC_U_TIMER_INTERRUPT;
	else
		throw std::runtime_error("some pending interrupt must be available here");

	switch (e.target_mode) {
		case MachineMode:
			csrs.mcause.fields.exception_code = exc;
			csrs.mcause.fields.interrupt = 1;
			break;

		case SupervisorMode:
			csrs.scause.fields.exception_code = exc;
			csrs.scause.fields.interrupt = 1;
			break;

		case UserMode:
			csrs.ucause.fields.exception_code = exc;
			csrs.ucause.fields.interrupt = 1;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(e.target_mode));
	}
}

PendingInterrupts ISS::compute_pending_interrupts() {
	uint32_t pending = csrs.mie.reg & csrs.mip.reg;

	if (!pending)
		return {NoneMode, 0};

	auto m_pending = pending & ~csrs.mideleg.reg;
	if (m_pending && (prv < MachineMode || (prv == MachineMode && csrs.mstatus.fields.mie))) {
		return {MachineMode, m_pending};
	}

	pending = pending & csrs.mideleg.reg;
	auto s_pending = pending & ~csrs.sideleg.reg;
	if (s_pending && (prv < SupervisorMode || (prv == SupervisorMode && csrs.mstatus.fields.sie))) {
		return {SupervisorMode, s_pending};
	}

	auto u_pending = pending & csrs.sideleg.reg;
	if (u_pending && (prv == UserMode && csrs.mstatus.fields.uie)) {
		return {UserMode, u_pending};
	}

	return {NoneMode, 0};
}

void ISS::switch_to_trap_handler(PrivilegeLevel target_mode) {
	if (trace) {
		printf("[vp::iss] switch to trap handler, time %s, last_pc %8x, pc %8x, irq %u, t-prv %1x\n",
		       quantum_keeper.get_current_time().to_string().c_str(), last_pc, pc, csrs.mcause.fields.interrupt, target_mode);
	}

	// free any potential LR/SC bus lock before processing a trap/interrupt
	release_lr_sc_reservation();

	auto pp = prv;
	prv = target_mode;

	switch (target_mode) {
		case MachineMode:
			csrs.mepc.reg = pc;

			csrs.mstatus.fields.mpie = csrs.mstatus.fields.mie;
			csrs.mstatus.fields.mie = 0;
			csrs.mstatus.fields.mpp = pp;

			pc = csrs.mtvec.get_base_address();

			if(pc == 0) {
				if(error_on_zero_traphandler) {
					throw std::runtime_error("[ISS] Took null trap handler in machine mode");
				} else {
					static bool once = true;
					if (once)
						std::cout << "[ISS] Warn: Taking trap handler in machine mode to 0x0, this is probably an error." << std::endl;
					once = false;
				}
			}

			if (csrs.mcause.fields.interrupt && csrs.mtvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.mcause.fields.exception_code;
			break;

		case SupervisorMode:
			assert(prv == SupervisorMode || prv == UserMode);

			csrs.sepc.reg = pc;

			csrs.mstatus.fields.spie = csrs.mstatus.fields.sie;
			csrs.mstatus.fields.sie = 0;
			csrs.mstatus.fields.spp = pp;

			pc = csrs.stvec.get_base_address();

			if (csrs.scause.fields.interrupt && csrs.stvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.scause.fields.exception_code;
			break;

		case UserMode:
			assert(prv == UserMode);

			csrs.uepc.reg = pc;

			csrs.mstatus.fields.upie = csrs.mstatus.fields.uie;
			csrs.mstatus.fields.uie = 0;

			pc = csrs.utvec.get_base_address();

			if (csrs.ucause.fields.interrupt && csrs.utvec.fields.mode == csr_mtvec::Mode::Vectored)
				pc += 4 * csrs.ucause.fields.exception_code;
			break;

		default:
			throw std::runtime_error("unknown privilege level " + std::to_string(target_mode));
	}
}

void ISS::performance_and_sync_update(Opcode::Mapping executed_op) {
    ++total_num_instr;

	if (!csrs.mcountinhibit.fields.IR)
		++csrs.instret.reg;

	if (lr_sc_counter != 0) {
		--lr_sc_counter;
		assert (lr_sc_counter >= 0);
		if (lr_sc_counter == 0)
            release_lr_sc_reservation();
	}

	auto new_cycles = instr_cycles[executed_op];

	if (!csrs.mcountinhibit.fields.CY)
		cycle_counter += new_cycles;

	quantum_keeper.inc(new_cycles);
	if (quantum_keeper.need_sync()) {
	    if (lr_sc_counter == 0) // match SystemC sync with bus unlocking in a tight LR_W/SC_W loop
		    quantum_keeper.sync();
	}
}

void ISS::run_step() {
	assert(regs.read(0) == 0);

	// speeds up the execution performance (non debug mode) significantly by
	// checking the additional flag first
	if (debug_mode && (breakpoints.find(pc) != breakpoints.end())) {
		status = CoreExecStatus::HitBreakpoint;
		return;
	}

	last_pc = pc;
	try {
		exec_step();

		auto x = compute_pending_interrupts();
		if (x.target_mode != NoneMode) {
			prepare_interrupt(x);
			switch_to_trap_handler(x.target_mode);
		}
	} catch (SimulationTrap &e) {
		if (trace)
			std::cout << "take trap " << e.reason << ", mtval=" << e.mtval << std::endl;
		auto target_mode = prepare_trap(e);
		switch_to_trap_handler(target_mode);
	}

	// NOTE: writes to zero register are supposedly allowed but must be ignored
	// (reset it after every instruction, instead of checking *rd != zero*
	// before every register write)
	regs.regs[regs.zero] = 0;

	// Do not use a check *pc == last_pc* here. The reason is that due to
	// interrupts *pc* can be set to *last_pc* accidentally (when jumping back
	// to *mepc*).
	if (shall_exit)
		status = CoreExecStatus::Terminated;

	performance_and_sync_update(op);
}

void ISS::run() {
	// run a single step until either a breakpoint is hit or the execution
	// terminates
	do {
		run_step();
	} while (status == CoreExecStatus::Runnable);

	// force sync to make sure that no action is missed
	quantum_keeper.sync();
}

struct LoadedLibrary {
    void* handle; // Handle to the loaded library
    std::array<ScoreFunction, SF_BATCH_SIZE> functions;
};

void ISS::flush_ringbuffer(){
	for (size_t offset = 1; offset < INSTRUCTION_TREE_DEPTH; offset++) //TODO check if this has to start at index 0
	{
		ring_buffer_index = (ring_buffer_index+1)%INSTRUCTION_TREE_DEPTH;

		//advance ringbuffer index and insert current instruction into tree (as if it were the oldest entry)
		Opcode::Mapping oldest_op = last_executed_steps[ring_buffer_index].last_executed_instruction;
		if(oldest_op){//ring buffer was not completely filled - just skip this entry 
			//check if a tree for this op already exists
			InstructionNodeR* found_tree = NULL;
			for (InstructionNodeR& root : instruction_trees){
				
				if(root.instruction == oldest_op){
					found_tree = &root;
					break;
				}
			}
			if(found_tree!=NULL){
			}else{
				// printf("first occurance of op %d during flush\n", oldest_op);
				instruction_trees.emplace_back(oldest_op, 0);
				found_tree = &instruction_trees.back();
			}
			//printf("flush: inserting with offset %d\n", offset);

			//insert ringbuffer - this opcode into tree
			found_tree->insert_rb(last_executed_steps, 
								ring_buffer_index, offset);
		}
		// else{
		// 	printf("flush: skipping empty entry at offset %d\n", offset);
		// }
	}
	
	
}

LoadedLibrary load_scoring_functions(const std::string& libraryPath){
	void* library = dlmopen(-1, libraryPath.c_str(), RTLD_NOW); // | RTLD_GLOBAL
	std::cout << "Loading library from: " << libraryPath << std::endl;

	std::array<ScoreFunction, SF_BATCH_SIZE>* score_functions_ptr; 
	if (!library) {
		std::cerr << "Error loading the library: " << dlerror() << std::endl;
		//exit(EXIT_FAILURE);
		return {NULL, nullptr};
	}

	// std::array<ScoreFunction, SF_BATCH_SIZE>* (*getScoreFunctions)() = 
	// reinterpret_cast<std::array<ScoreFunction, SF_BATCH_SIZE>* (*)()>(dlsym(library, "getScoreFunctions"));
	// if (!getScoreFunctions) {
	// 	std::cerr << "Error getting symbol: " << dlerror() << std::endl;
	// 	dlclose(library);
	// 	exit(EXIT_FAILURE);
	// }

	score_functions_ptr = 
		(std::array<ScoreFunction, SF_BATCH_SIZE>*)dlsym(library, "score_functions");

	if (!score_functions_ptr) {
		std::cerr << "Error getting score_functions pointer: " << dlerror() << std::endl;
		dlclose(library);
		//exit(EXIT_FAILURE);
		return {library, nullptr};
	}
	return {library, *score_functions_ptr};
}

void analyze_trees(std::array<ScoreFunction, SF_BATCH_SIZE> score_functions, std::list<InstructionNodeR> instruction_trees){
	std::vector<std::vector<Path>> best_sequences_for_sf; 
	uint32_t score_function_index = 0;
	for (auto &&score_function : score_functions)
	{
		std::vector<Path> tmp_best_sequences; 
		auto _sf = score_functions[score_function_index];
		for(InstructionNodeR& tree : instruction_trees){
			Path p = tree.extend_path(
				{1, 0, 1.0, 0, -1, Opcode::Mapping::UNDEF, _sf});
			tmp_best_sequences.push_back(p);
		}
		best_sequences_for_sf.push_back(tmp_best_sequences);
		std::sort(best_sequences_for_sf[score_function_index].begin(), best_sequences_for_sf[score_function_index].end(), 
		[_sf](Path a, Path b) -> bool{
			return a.get_score(_sf)>b.get_score(_sf);
		});
		score_function_index++;
	}
	printf("finished score function analysis\n");
	
	int sf_index = 0;
	for (auto &&sequences : best_sequences_for_sf)
	{
		printf("\nBest sequences for score function %d:\n", sf_index);
		for (size_t i = 0; i < std::min(3, (int)sequences.size()); i++) //print the best 3 sequences for each score function
		{
			sequences[i].show();
		}
		sf_index++;
	}
}

void ISS::output_dot(std::streambuf *cout_save){
	std::ofstream output;
	if(is_single_file || !output_filename || !output_filename[0]){
			if(output_filename && output_filename[0]){
				std::cout << "writing to file " << output_filename << std::endl;
				output = std::ofstream(output_filename);
				output << "//" << std::time(0) << std::endl;
				std::cout.rdbuf(output.rdbuf());
			}

			std::cout << "digraph g{" << std::endl;
			//shape = record
			std::cout << "node [shape = plaintext, style=\"bold\", height = .5, colorscheme=rdpu9];" << std::endl; //pubu9

			for (InstructionNodeR& tree : instruction_trees){
				//tree.print();
				tree.tree_to_dot(csrs.instret.reg);
			}

			std::cout << "}" << std::endl; 

			if(output_filename){ //reset cout
				std::cout.rdbuf(cout_save);
				std::cout << "restored cout" << std::endl;
			}

			std::cout << std::hex;
			for (const auto& n : memory_access_map) {
				std::cout << n.first << " [" << std::get<0>(n.second) << " | " << std::get<1>(n.second) << "]" << std::endl;
			}
			std::cout << std::dec;

		}else{//one dot file for each opcode output in directory
			std::string single_output_filename = "";
			std::cout << "writing dot files to directory " << output_filename << std::endl;
			//output = std::ofstream(output_filename);
			//output << "//" << std::time(0) << std::endl;
			//std::cout.rdbuf(output.rdbuf());
			uint64_t index = 0;
			for (InstructionNodeR& tree : instruction_trees){
				index++;
				single_output_filename = output_filename + 
										std::string(Opcode::mappingStr[tree.instruction]) + 
										std::string(".dot");
				output = std::ofstream(single_output_filename);
				output << "// " << std::time(0) << std::endl;
				std::cout.rdbuf(output.rdbuf());

				std::cout << "digraph g{" << std::endl;
				std::cout << "node [shape = plaintext, style=\"bold\", height = .5, colorscheme=rdpu9];" << std::endl; //pubu9
				std::cout << "edge [style=\"solid\", arrowsize = .5];" << std::endl; //don't use colorscheme greys9 for now

				//tree.print();
				tree.tree_to_dot(csrs.instret.reg);
				std::cout << "}" << std::endl; 
			}


			//output memory access map
			single_output_filename = output_filename +  std::string("memory_map.md");
			output = std::ofstream(single_output_filename);
			output << "// " << std::time(0) << std::endl;
			std::cout.rdbuf(output.rdbuf());
			
			std::cout << std::hex;
			for (const auto& n : memory_access_map) {
				std::cout << n.first << " [" << std::get<0>(n.second) << " | " << std::get<1>(n.second) << "]" << std::endl;
			}
			std::cout << std::dec;


			if(output_filename){ //reset cout
				std::cout.rdbuf(cout_save);
				std::cout << "restored cout" << std::endl;
			}
		}
}

void ISS::output_csv(std::streambuf *cout_save){
	std::ofstream output;
	if(!output_filename || !output_filename[0]){
		
			std::cout << "[ERROR] No output directory set for csv export" << std::endl;

		}else{
			std::string single_output_filename = "";

			std::string file_path = std::string(input_filename);
			std::string application_name = file_path.substr(file_path.find_last_of("/\\")+1);
			
			std::cout << "writing csv files to directory " << output_filename << std::endl;
			single_output_filename = output_filename + 
										std::string("Execution_Trees_") + 
										application_name + 
										std::string(".csv");
			output = std::ofstream(single_output_filename);
			std::cout.rdbuf(output.rdbuf());

			std::cout  << "id;"
					   << "parentId;" 
					   << "tree;"
					   << "instruction;"
					   << "weight;"
					   << "weightDifference;"
					   << "weightDifferenceMax;"
					   << "weightDifferenceTotal;"
					   << "length;"
					   << "lengthDifference;"
					   << "cycles;"
					   << "dependencyScore;"
					   << "dependencyScoreTotal;"
					   << "dependenciesTrue;"
					   << "dependenciesAnti;"
					   << "dependenciesOut;"
					   << "dependenciesTrueTotal;"
					   << "dependenciesAntiTotal;"
					   << "dependenciesOutTotal;"
					   << "children;"
					   << "childrenDifference;"
					   << "inputs;"
					   << "inputsTotal;"
					   << "outputs;"
					   << "outputsTotal;"
					   << "instructionTypes;"
					   << "branches;"
					   << "occurrenceStart;"
					   << "occurrenceBeginning;"
					   << "occurrenceMid;"
					   << "occurrenceEnd;"
					   << "programCounters;"
					   << "programCountersDifference"
					   << std::endl;


			uint64_t index = 0;
			std::bitset<32> total_inputs;
			std::map<InstructionType, uint32_t> instruction_types;
			uint64_t total_max_weight = 0;
			for (InstructionNodeR& tree : instruction_trees){
				if(tree.weight > total_max_weight){
					total_max_weight = tree.weight;
				}
			}
			//total_inputs.reset();
			for (InstructionNodeR& tree : instruction_trees){
				index++;
				tree.to_csv({
					csrs.instret.reg,
					Opcode::mappingStr[tree.instruction],
					1, //depth
					0,0,0,0,0,0,
					instruction_types, 
					0, 
					tree.weight,
					tree.weight,//last_weight
					total_max_weight,
					tree.get_pc().size(),
				});
			}



			if(output_filename){ //reset cout
				std::cout.rdbuf(cout_save);
				std::cout << "restored cout" << std::endl;
			}
		}
	}

void ISS::output_json(std::streambuf *cout_save, 
		std::vector<std::vector<PathNode>> discovered_sequences_node_list, 
		std::vector<std::vector<std::vector<PathNode>>> discovered_sub_sequences_node_lists, 
		std::vector<std::vector<std::vector<PathNode>>> discovered_variant_sequences_node_lists){
	std::ofstream output;
	nlohmann::json top_level_json;
		int idx = 0;
		printf("Converting Full Paths to JSON\n");
		for (auto &&seq : discovered_sequences_node_list)
		{
			printf(".");
			nlohmann::json path_node_json_array = nlohmann::json::array();
			for (auto &&node : seq)
			{
				nlohmann::json path_node_json = node.to_json();	
				path_node_json_array.push_back(path_node_json);

			}
			std::string key = "Sequence" + std::to_string(idx);
			top_level_json[key] = path_node_json_array;


			int sub_idx = 0;
			for (auto &&sub_seq : discovered_sub_sequences_node_lists.at(idx))
			{
				nlohmann::json sub_sequence_json_array = nlohmann::json::array();
				for (auto &&node : sub_seq)
				{
					nlohmann::json path_node_json = node.to_json();	
					sub_sequence_json_array.push_back(path_node_json);

				}
				std::string sub_sequence_key = key + "-" + std::to_string(sub_idx);
				top_level_json[sub_sequence_key] = sub_sequence_json_array;
				sub_idx++;
			}
			int variant_idx = 0;
			for (auto &&variant_seq : discovered_variant_sequences_node_lists.at(idx))
			{
				nlohmann::json variant_sequence_json_array = nlohmann::json::array();
				for (auto &&node : variant_seq)
				{
					nlohmann::json path_node_json = node.to_json();	
					variant_sequence_json_array.push_back(path_node_json);

				}
				std::string variant_sequence_key = key + "v" + std::to_string(variant_idx);
				top_level_json[variant_sequence_key] = variant_sequence_json_array;
				variant_idx++;
			}
			idx++;
		}
		printf("\n");


		//save or print json
		if(!output_filename || !output_filename[0]){
			
			std::cout << top_level_json.dump(JSON_INDENT) << std::endl;

		}else{
			std::string single_output_filename = "";
			std::cout << "writing json to directory " << output_filename << std::endl;

			std::string file_path = std::string(input_filename);
			std::string application_name = file_path.substr(file_path.find_last_of("/\\")+1);

			single_output_filename = output_filename + 
									std::string("sequences_") + 
									application_name + 
									std::string(".json");
			std::cout << "Sequence file: " << single_output_filename << std::endl;
			output = std::ofstream(single_output_filename);
			std::cout.rdbuf(output.rdbuf());

			std::cout << top_level_json.dump(JSON_INDENT) << std::endl;



			if(output_filename){ //reset cout
				std::cout.rdbuf(cout_save);
				std::cout << "restored cout" << std::endl;
			}
		}
}

void ISS::output_full(std::streambuf *cout_save){
	std::ofstream output;
	nlohmann::ordered_json top_level_json;
	std::cout << "writing json to directory " << output_filename << std::endl;

	std::string single_output_filename = "";
	uint64_t index = 0;
	for (InstructionNodeR& tree : instruction_trees){
		nlohmann::ordered_json tree_json_array = nlohmann::ordered_json::array();
		std::string file_path = std::string(input_filename);
		std::string application_name = file_path.substr(file_path.find_last_of("/\\")+1);
		single_output_filename = output_filename + application_name +
								std::string(Opcode::mappingStr[tree.instruction]) + 
								std::string(".json");

		output = std::ofstream(single_output_filename);
		std::cout.rdbuf(output.rdbuf());
		tree_json_array = tree.to_json();
		std::string json_string = tree_json_array.dump(JSON_INDENT);
		std::cout << json_string << std::endl;
		index++;

		//csrs.instret.reg
		printf(".");

	}
	printf("Done\n");

			if(output_filename){ //reset cout
				std::cout.rdbuf(cout_save);
				std::cout << "restored cout" << std::endl;
			}
		// }
}

void ISS::show() {
	flush_ringbuffer();//insert all instructions currently in the ringbuffer into their trees
	boost::io::ios_flags_saver ifs(std::cout);
	std::cout << "=[ core : " << csrs.mhartid.reg << " ]===========================" << std::endl;
	std::cout << "simulation time: " << sc_core::sc_time_stamp() << std::endl;
	regs.show();
	std::cout << "pc = " << std::hex << pc << std::endl;
	std::cout << "num-instr = " << std::dec << csrs.instret.reg << std::endl;

	std::cout << "==========================\n==========================\n";
	std::cout << "execution statistics: (" << instruction_trees.size() << " Trees)" << std::endl;

	//std::ostream output = std::cout;
	std::streambuf *cout_save = std::cout.rdbuf();

	if(output_as_dot){
		output_dot(cout_save);
	}
	if(output_as_csv){
		output_csv(cout_save);
	}
	if(output_full_export){
		output_full(cout_save);
	}

	//find best instruction squence for each tree
	std::vector<Path> discovered_sequences; 
	std::vector<std::vector<Path>> discovered_sub_sequences;  //TODO currently not used
	int i = 0;
	printf("start analysis\n");
	std::vector<Path> tmp_discovered_sub_sequences; 
	auto sf = [](ScoreParams p) -> float {
		float score = (p.length * p.weight) * p.score_multiplier 
			+ p.weight * p.score_bonus; //length * minimum_weight;
		return score;
	};
	for (InstructionNodeR& tree : instruction_trees){
		Path p = tree.extend_path({1, 0, 1.0, i, -1, Opcode::Mapping::UNDEF, sf});
		discovered_sequences.push_back(p);
		i++;
		printf(".");
	}
	printf("-> analyzed all trees\n");

	//sort discovered paths to print most relevant ones last
	// std::function<float(ScoreParams)> score_func = sf;
	std::sort(discovered_sequences.begin(), discovered_sequences.end(), 
	[sf](Path a, Path b) -> bool{
		return a.get_score(sf)<b.get_score(sf);
	});

	std::vector<std::vector<PathNode>> discovered_sequences_node_list;
	std::vector<std::vector<BranchingPoint>> discovered_variant_starting_points;
	std::vector<std::vector<std::vector<PathNode>>> discovered_sub_sequences_node_lists;
	std::vector<std::vector<std::vector<PathNode>>> discovered_variant_sequences_node_lists;

	printf("\n -----------------\n| Best Sequences |\n -----------------\n");
	for (auto &&p : discovered_sequences)
	{

		//find matching tree for path //maybe just store a reference in path?
		InstructionNodeR* found_tree = NULL;
		for (InstructionNodeR& root : instruction_trees){
			
			if(root.instruction == p.opcodes[0]){
				found_tree = &root;
				break;
			}
		}
		if(found_tree==NULL){ //this should never happen, as there has to exist a tree to find a path in the first place
			printf("[ERROR] Could not find matching tree for discovered path\nOpcode: %s\n", Opcode::mappingStr[p.opcodes[0]]);
		}

		std::vector<PathNode> full_path;
		full_path = found_tree->path_to_path_nodes(p, 0);
		std::vector<BranchingPoint> variant_starting_points = found_tree->find_variant_branch(p, 0);
		//sort variant starting points, so we can extend the top N
		std::sort(variant_starting_points.begin(), variant_starting_points.end(), 
		[](BranchingPoint a, BranchingPoint b) -> bool{
			return a.ratio>b.ratio;
		});

		#ifdef log_variants
		for (auto &&v : variant_starting_points)
		{
			printf("Variant Branching Point: %s at %d, with ratio %.4f\n", Opcode::mappingStr[v.instruction], v.depth, v.ratio);
		}
		#endif

		discovered_sequences_node_list.push_back(full_path);
		discovered_variant_starting_points.push_back(variant_starting_points);

		//force extend to top N variant starting points
		std::vector<Path> tmp_discovered_variants; 
		uint8_t max_variants = !(variant_starting_points.size()<MAX_VARIANTS)?MAX_VARIANTS:variant_starting_points.size();
		for (int32_t i = 0; i < max_variants; i++)
		{
			//convert to Path first (up to new branching point and then force extension)
			Path tmp_variant = found_tree->extend_path({1, 0, 1.0, i, variant_starting_points[i].depth, 
									variant_starting_points[i].instruction, sf});
			tmp_discovered_variants.push_back(tmp_variant);
		}
		
		printf("-------------------------------------------\n");
		//print discovered sequences
		p.show();	

		tmp_discovered_sub_sequences = p.end_of_sequence->force_path_extension(p, sf);
		
		//printf("last Node in sequence should be %s\n", Opcode::mappingStr[p.end_of_sequence->instruction]);
		#ifdef log_variants
		printf("-------------------------------------------\n");
		printf("Sub Sequences (%d)\n[\n",tmp_discovered_sub_sequences.size());
		#endif

		std::vector<std::vector<PathNode>> tmp_node_list;
		for (auto &&subseq : tmp_discovered_sub_sequences)
		{
			#ifdef log_variants
			printf(" - Sub Sequence:\n");
			subseq.show(" - ");
			printf(" - ,\n");
			#endif
			tmp_node_list.push_back(found_tree->path_to_path_nodes(subseq, 0));
		}
		discovered_sub_sequences_node_lists.push_back(tmp_node_list);
		discovered_sub_sequences.push_back(tmp_discovered_sub_sequences);

		//convert variants to path nodes
		std::vector<std::vector<PathNode>> tmp_variant_node_list;
		for (auto &&variant : tmp_discovered_variants)
		{
			#ifdef log_variants
			printf(" - Variant Sequence:\n");
			variant.show(" - ");
			printf(" - ,\n");
			#endif
			tmp_variant_node_list.push_back(found_tree->path_to_path_nodes(variant, 0));
		}
		discovered_variant_sequences_node_lists.push_back(tmp_variant_node_list);
		#ifdef log_variants
		printf("]\n-------------------------------------discovered_sequences_node_list------\n");
		#endif
	}

	if(output_as_json){
		output_json(cout_save, discovered_sequences_node_list, 
			discovered_sub_sequences_node_lists, 
			discovered_variant_sequences_node_lists);
	}

	
	float total_percent = 1.0;
	if(discovered_sequences.empty()){
		printf("[Warning] Ringbuffer was not filled at least once. \nThis means, the whole program fits into one sequence. \nYou probably want to execute a longer program or decrease the tree bound.\n");

	}else{
		//print some additional info
		total_percent = (float)discovered_sequences.back().minimum_weight * (float)discovered_sequences.back().length / (float)csrs.instret.reg;
		std::cout << "inverse dependency score " << discovered_sequences.back().inverse_dependency_score << std::endl;
		std::cout << "partially normalized potential " << discovered_sequences.back().get_normalized_score() << std::endl;
		std::cout << "normalized potential " << discovered_sequences.back().get_normalized_score() * total_percent * 100 << std::endl;

		std::list<Opcode::Mapping> unused_instructions; 
		for (size_t i = Opcode::Mapping::ADD; i < Opcode::Mapping::NUMBER_OF_INSTRUCTIONS; i++)
		{
			Opcode::Mapping c_op = (Opcode::Mapping)i;
			bool found = false;
			for (InstructionNodeR& tree : instruction_trees){
				if(tree.instruction == c_op){
					found = true;
				}
			}
			if(!found){
				unused_instructions.emplace_back(c_op);
			}
		}
		std::cout << "\n[Unused Instructions]" << std::endl;
		if(unused_instructions.size()>0){
			// for (auto &&unused_ins : unused_instructions)
			// {
			// 	std::cout << Opcode::mappingStr[unused_ins] << std::endl;
			// }
			std::cout << unused_instructions.size() << std::endl;
		}else{
			std::cout << "- NONE -" << std::endl;
		}


		//evaluate additional score functions
		LoadedLibrary sf_lib = load_scoring_functions("./vp/build/lib/libfunctions.so");
		std::array<ScoreFunction, SF_BATCH_SIZE> score_functions;
		if(sf_lib.handle){
			if(sf_lib.functions.size() > 0){//TODO 
				score_functions = sf_lib.functions;

				// Access the score_functions array and call the functions
				std::cout << "Testing loaded score functions: " << std::endl;
				for (const auto& func : score_functions) {
					std::cout << "Test Score: " << func({Opcode::Mapping::ADD, Opcode::Mapping::ADD, 100, 8, 0, 3, 2, 1, 1, 0}) << std::endl;
				}

				// {
				//     [](ScoreParams p) -> float {
				// 		float score = ((p.length * p.weight) * p.score_multiplier 
				// 			+ p.weight * p.score_bonus); //length * minimum_weight;
				// 		return score;
				// 	},
				//     [](ScoreParams p) -> float {
				// 		float score = ((p.length * p.weight) * p.score_multiplier 
				// 			+ p.weight * p.score_bonus) / (1 + p.dep_score); 
				// 		return score;
				// 	},
				//     [](ScoreParams p) -> float {
				// 		float score = ((p.length * p.weight) * p.score_multiplier 
				// 			+ p.weight * p.score_bonus) * (p.num_children + 1);
				// 		return score;
				// 	}
				// };
			}else{
				std::cout << "Error loading scoring functions from library\nSkipping additional scoring functions" << std::endl;
			}
		}else{
			std::cout << "Could not find library at default path\nSkipping additional scoring functions" << std::endl;
		}
		

	


		if(interactive_mode){
			printf("start score function analysis\n");
			int run_id = 0;

			while (true) {
				std::cout << "\nEnter \n" 
							<< "\t'a' to run tree analysis\n"
							<< "\t'b' to run analysis performance benchmark\n"
							<< "\t'r' to reload the library\n" 
							<< "\t'p [% threshold]' to prune trees\n" 
							<< "\t'd' to export as dot\n" 
							<< "\t'e' for a full export as json\n" 
							<< "\t'q' to quit\n" 
							<< ":"<< std::endl;
				std::string userInput; 
				std::cin >> userInput;
				char mode = userInput[0];

				if (mode == 'r') {
					// Reload the library and get the updated array of functions
					dlclose(sf_lib.handle);
					std::string library_path = 
							"./vp/build/lib/libfunctions.so"; //+ std::to_string(run_id)
					sf_lib = load_scoring_functions(library_path);
					score_functions = sf_lib.functions;
					//TODO add checks for library and allow to specify custom path
					run_id++;
				} 
				else if(mode == 'a'){ //run analysis
					analyze_trees(score_functions, instruction_trees);
				} 
				else if(mode == 'b'){ //run analysis benchmark
					using std::chrono::high_resolution_clock;
					using std::chrono::duration;
					using std::chrono::milliseconds;

					auto time_analysis1 = high_resolution_clock::now();
					for (size_t i = 0; i < 100; i++)
					{
						analyze_trees(score_functions, instruction_trees);
					}
					auto time_analysis2 = high_resolution_clock::now();
					duration<double, std::milli> ms_double = time_analysis2 - time_analysis1;

					std::cout << "Analysis of 300  SF took " << ms_double.count() << "ms\n" 
						<< ms_double.count()/300.0 << "ms on average per function" << std::endl;
				} 
				else if (mode == 'q') {
					dlclose(sf_lib.handle);
					break;
				} 
				else if (mode == 'p') {
					float prune_threshold = PRUNE_THRESHOLD_WEIGHT;
					if(userInput.length() > 1){
						try {
							prune_threshold = std::stof(userInput.substr(1))/100.0;
							std::cout << "Start pruning trees with threshold " << prune_threshold << std::endl;
						} catch (const std::invalid_argument& e) {
							std::cout << "No valid threshold specified. Set to default (" << PRUNE_THRESHOLD_WEIGHT << ")" << std::endl;
						}
					}
					for (auto &&tree : instruction_trees)
					{
						printf("\tPruning with absolute threshold: %f\n", tree.weight * prune_threshold);
						tree.prune_tree(tree.weight * prune_threshold, 0);	
					}
				} 
				else if (mode == 'd') {
					printf("exporting trees to dot\n");
					std::streambuf *cout_save = std::cout.rdbuf();
					output_dot(cout_save);
				}
				else if (mode == 'e') {
					printf("exporting trees to json\n");
					std::streambuf *cout_save = std::cout.rdbuf();
					output_full(cout_save);
				} 
				else {
					std::cout << "Invalid input." << std::endl;
				}
			}
		}
		
		//close library
		if(sf_lib.handle){
			dlclose(sf_lib.handle);
		}
	}
	std::cout << "total instructions: " << csrs.instret.reg << "(" << total_num_instr << ")" << " [" << total_percent*100 << "]" << std::endl;
	std::cout << "total cycles: " << _compute_and_get_current_cycles() << std::endl;

	if(discovered_sequences.empty()){
		std::cout << "Best sequence: " << "NONE (the whole program fits into one sequence)" 
		<< "\nLength: " << csrs.instret.reg 
		<< "\nWeight: " << 1 
		<< "\n%:      [" << total_percent*100 << "]"
		<< "\nTotal:  " << csrs.instret.reg
		<< "\nNP:     1.0" << std::endl;
	}else{
		std::cout << "Best sequence: " << Opcode::mappingStr[discovered_sequences.back().opcodes[0]] 
		<< "\nLength: " << discovered_sequences.back().length 
		<< "\nWeight: " << discovered_sequences.back().minimum_weight
		<< "\n%:      [" << total_percent*100 << "]"
		<< "\nTotal:  " << csrs.instret.reg
		<< "\nNP:     " << discovered_sequences.back().get_normalized_score() << std::endl;
	}
	// std::cout << "$ " << Opcode::mappingStr[discovered_sequences.back().opcodes[0]] 
	// << " & " << discovered_sequences.back().length 
	// << " & " << discovered_sequences.back().minimum_weight
	// << " & " << csrs.instret.reg << "K[" << total_percent*100 << "]"
	// << " & " << discovered_sequences.back().get_normalized_score() << std::endl;
}
