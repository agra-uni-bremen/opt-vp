#pragma once

#include "core/common/bus_lock_if.h"
#include "core/common/clint_if.h"
#include "core/common/instr.h"
#include "core/common/irq_if.h"
#include "core/common/trap.h"
#include "core/common/debug.h"
#include "csr.h"
#include "fp.h"
#include "mem_if.h"
#include "syscall_if.h"
#include "util/common.h"

#include "trace.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/tlm_quantumkeeper.h>
#include <systemc>

#include <list>
//#include <queue>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


namespace rv32 {

struct RegFile {
	static constexpr unsigned NUM_REGS = 32;

	int32_t regs[NUM_REGS];

	RegFile();

	RegFile(const RegFile &other);

	void write(uint32_t index, int32_t value);

	int32_t read(uint32_t index);

	uint32_t shamt(uint32_t index);

	int32_t &operator[](const uint32_t idx);

	void show();

	enum e : uint16_t {
		x0 = 0,
		x1,
		x2,
		x3,
		x4,
		x5,
		x6,
		x7,
		x8,
		x9,
		x10,
		x11,
		x12,
		x13,
		x14,
		x15,
		x16,
		x17,
		x18,
		x19,
		x20,
		x21,
		x22,
		x23,
		x24,
		x25,
		x26,
		x27,
		x28,
		x29,
		x30,
		x31,

		zero = x0,
		ra = x1,
		sp = x2,
		gp = x3,
		tp = x4,
		t0 = x5,
		t1 = x6,
		t2 = x7,
		s0 = x8,
		fp = x8,
		s1 = x9,
		a0 = x10,
		a1 = x11,
		a2 = x12,
		a3 = x13,
		a4 = x14,
		a5 = x15,
		a6 = x16,
		a7 = x17,
		s2 = x18,
		s3 = x19,
		s4 = x20,
		s5 = x21,
		s6 = x22,
		s7 = x23,
		s8 = x24,
		s9 = x25,
		s10 = x26,
		s11 = x27,
		t3 = x28,
		t4 = x29,
		t5 = x30,
		t6 = x31,
	};
};

// NOTE: on this branch, currently the *simple-timing* model is still directly
// integrated in the ISS. Merge the *timedb* branch to use the timing_if.
struct ISS;

struct timing_if {
	virtual ~timing_if() {}

	virtual void update_timing(Instruction instr, Opcode::Mapping op, ISS &iss) = 0;
};

/* Buffer to be used between the ISS and instruction memory interface to cache compressed instructions.
 * In case the ISS does not support compressed instructions, then this buffer is not necessary and the ISS
 * can use the memory interface directly. */
struct InstructionBuffer {
	instr_memory_if *instr_mem = nullptr;
	uint32_t last_fetch_addr = 0;
	uint32_t buffer = 0;

	uint32_t load_instr(uint64_t addr) {
		if (addr == (last_fetch_addr + 2))
			return (buffer >> 16);

		last_fetch_addr = addr;
		buffer = instr_mem->load_instr(addr);
		return buffer;
	}
};

struct PendingInterrupts {
	PrivilegeLevel target_mode;
	uint32_t pending;
};

struct ISS : public external_interrupt_target, public clint_interrupt_target, public iss_syscall_if, public debug_target_if {
	clint_if *clint = nullptr;
	instr_memory_if *instr_mem = nullptr;
	data_memory_if *mem = nullptr;
	syscall_emulator_if *sys = nullptr;  // optional, if provided, the iss will intercept and handle syscalls directly
	RegFile regs;
	FpRegs fp_regs;
	uint32_t pc = 0;
	uint32_t last_pc = 0;
	bool trace = false;
	bool shall_exit = false;
    bool ignore_wfi = false;
    bool error_on_zero_traphandler = false;
	csr_table csrs;
	PrivilegeLevel prv = MachineMode;
	int64_t lr_sc_counter = 0;
	uint64_t total_num_instr = 0;

	bool is_single_file = false;
	bool output_as_dot = false;
	bool output_as_csv = false;
	bool output_as_json = false;
	bool output_full_export = false;
	bool interactive_mode = false;

	// last decoded and executed instruction and opcode
	Instruction instr;
	Opcode::Mapping op;

	//one entry for each different instruction, representing the root node
	std::list<InstructionNodeR> instruction_trees; //TODO maybe allow derived classes if their info becomes necessary
	//last_executed instructions as a ring buffer
	#ifdef USE_INDIVIDUAL_ARRAYS
	std::array<Opcode::Mapping, INSTRUCTION_TREE_DEPTH> last_executed_instructions;
	std::array<std::tuple<uint16_t,uint16_t,uint16_t>, INSTRUCTION_TREE_DEPTH> last_registers; //-1 if not applicable TODO add rs3 for R4 type fused multiply
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> last_executed_pc;
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> last_memory_read; //fill with 0 if none read
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> last_memory_written;

	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> last_cycles;
	std::array<uint64_t, INSTRUCTION_TREE_DEPTH> last_powermodes;
	#else
	std::array<ExecutionInfo, INSTRUCTION_TREE_DEPTH> last_executed_steps; 
	#endif

	uint64_t prev_cycles = 0;

	std::map<uint64_t, std::tuple<uint64_t,uint64_t>> memory_access_map; //map mem -> write_access | load_access
	std::tuple<uint64_t, AccessType> last_memory_access = {-1, AccessType::NONE}; //address, is_store = 2 is_load = 1 no_memory_access=0

	uint8_t ring_buffer_index =  0;
	CoreExecStatus status = CoreExecStatus::Runnable;
	std::unordered_set<uint32_t> breakpoints;
	bool debug_mode = false;

	const char* output_filename;
	uint64_t *path_hashes;
	const char* input_filename;

	sc_core::sc_event wfi_event;

	std::string systemc_name;
	tlm_utils::tlm_quantumkeeper quantum_keeper;
	sc_core::sc_time cycle_time;
	sc_core::sc_time cycle_counter;  // use a separate cycle counter, since cycle count can be inhibited
	std::array<sc_core::sc_time, Opcode::NUMBER_OF_INSTRUCTIONS> instr_cycles;

	static constexpr int32_t REG_MIN = INT32_MIN;
    static constexpr unsigned xlen = 32;

	ISS(uint32_t hart_id, const char *output_filename = "", const char *input_filename = "", std::vector<uint64_t> input_hashes = {}, bool use_E_base_isa = false);

	void exec_step();

	uint64_t _compute_and_get_current_cycles();

	void init(instr_memory_if *instr_mem, data_memory_if *data_mem, clint_if *clint, uint32_t entrypoint, uint32_t sp);

	void trigger_external_interrupt(PrivilegeLevel level) override;

	void clear_external_interrupt(PrivilegeLevel level) override;

	void trigger_timer_interrupt(bool status) override;

	void trigger_software_interrupt(bool status) override;


	void sys_exit() override;
	unsigned get_syscall_register_index() override;
	uint64_t read_register(unsigned idx) override;
	void write_register(unsigned idx, uint64_t value) override;

    std::vector<uint64_t> get_registers(void) override;

    Architecture get_architecture(void) override {
        return RV32;
    }

    uint64_t get_progam_counter(void) override;
    void enable_debug(void) override;
    CoreExecStatus get_status(void) override;
    void set_status(CoreExecStatus) override;
    void block_on_wfi(bool) override;

    void insert_breakpoint(uint64_t) override;
    void remove_breakpoint(uint64_t) override;

	uint64_t get_hart_id() override;


	void release_lr_sc_reservation() {
		lr_sc_counter = 0;
		mem->atomic_unlock();
	}

	void fp_prepare_instr();
	void fp_finish_instr();
	void fp_set_dirty();
	void fp_update_exception_flags();
	void fp_setup_rm();
	void fp_require_not_off();

	uint32_t get_csr_value(uint32_t addr);
	void set_csr_value(uint32_t addr, uint32_t value);

	bool is_invalid_csr_access(uint32_t csr_addr, bool is_write);
	void validate_csr_counter_read_access_rights(uint32_t addr);

	unsigned pc_alignment_mask() {
		if (csrs.misa.has_C_extension())
			return ~0x1;
		else
			return ~0x3;
	}

	inline void trap_check_pc_alignment() {
		assert(!(pc & 0x1) && "not possible due to immediate formats and jump execution");

		if (unlikely((pc & 0x3) && (!csrs.misa.has_C_extension()))) {
			// NOTE: misaligned instruction address not possible on machines supporting compressed instructions
			raise_trap(EXC_INSTR_ADDR_MISALIGNED, pc);
		}
	}

	template <unsigned Alignment, bool isLoad>
	inline void trap_check_addr_alignment(uint32_t addr) {
		if (unlikely(addr % Alignment)) {
			raise_trap(isLoad ? EXC_LOAD_ADDR_MISALIGNED : EXC_STORE_AMO_ADDR_MISALIGNED, addr);
		}
	}

	inline void log_memory_store(uint64_t address, uint64_t apc){
		//update entry and invalidate last load
		memory_access_map[address] = {apc, 0};
		last_memory_access = {address, AccessType::STORE};
	}
	inline void log_memory_read(uint64_t address, uint64_t apc){
		//update entry and keep last write
		std::get<1>(memory_access_map[address]) = apc;
		last_memory_access = {address, AccessType::LOAD};
	}

	inline void execute_amo(Instruction &instr, std::function<int32_t(int32_t, int32_t)> operation) {
		uint32_t addr = regs[instr.rs1()];
		trap_check_addr_alignment<4, false>(addr);
		uint32_t data;
		try {
			data = mem->atomic_load_word(addr);
		} catch (SimulationTrap &e) {
			if (e.reason == EXC_LOAD_ACCESS_FAULT)
				e.reason = EXC_STORE_AMO_ACCESS_FAULT;
			throw e;
		}
		uint32_t val = operation(data, regs[instr.rs2()]);
		mem->atomic_store_word(addr, val);
		regs[instr.rd()] = data;
	}

	inline bool m_mode() {
		return prv == MachineMode;
	}

	inline bool s_mode() {
		return prv == SupervisorMode;
	}

	inline bool u_mode() {
		return prv == UserMode;
	}

	PrivilegeLevel prepare_trap(SimulationTrap &e);

	void prepare_interrupt(const PendingInterrupts &x);

	PendingInterrupts compute_pending_interrupts();

	bool has_pending_enabled_interrupts() {
		return compute_pending_interrupts().target_mode != NoneMode;
	}

	bool has_local_pending_enabled_interrupts() {
		return csrs.mie.reg & csrs.mip.reg;
	}

	void return_from_trap_handler(PrivilegeLevel return_mode);

	void switch_to_trap_handler(PrivilegeLevel target_mode);

	void performance_and_sync_update(Opcode::Mapping executed_op);

	void run_step() override;

	void run() override;

	void flush_ringbuffer();

	void output_dot(std::streambuf *cout_save);
	void output_csv(std::streambuf *cout_save);
	void output_json(std::streambuf *cout_save, 
		std::vector<std::vector<PathNode>> discovered_sequences_node_list, 
		std::vector<std::vector<std::vector<PathNode>>> discovered_sub_sequences_node_lists, 
		std::vector<std::vector<std::vector<PathNode>>> discovered_variant_sequences_node_lists);
	void output_full(std::streambuf *cout_save);

	void show();
};

/* Do not call the run function of the ISS directly but use one of the Runner
 * wrappers. */
struct DirectCoreRunner : public sc_core::sc_module {
	ISS &core;
	std::string thread_name;

	SC_HAS_PROCESS(DirectCoreRunner);

	DirectCoreRunner(ISS &core) : sc_module(sc_core::sc_module_name(core.systemc_name.c_str())), core(core) {
		thread_name = "run" + std::to_string(core.get_hart_id());
		SC_NAMED_THREAD(run, thread_name.c_str());
	}

	void run() {
		core.run();

		if (core.status == CoreExecStatus::HitBreakpoint) {
			throw std::runtime_error(
			    "Breakpoints are not supported in the direct runner, use the debug "
			    "runner instead.");
		}
		assert(core.status == CoreExecStatus::Terminated);

		sc_core::sc_stop();
	}
};

}  // namespace rv32
