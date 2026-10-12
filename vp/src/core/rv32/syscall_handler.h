#pragma once

#include <assert.h>
#include <fcntl.h>
#include <stdint.h>

#include <tlm_utils/simple_target_socket.h>
#include <systemc>

#include "iss.h"
#include "syscall_if.h"

namespace rv32 {

// see: newlib/libgloss/riscv @
// https://github.com/riscv/riscv-newlib/tree/riscv-newlib-2.5.0/libgloss/riscv
// The names must not be macros: glibc defines SYS_<name> as the host system call numbers.
namespace syscall_nr {
constexpr uint64_t zero = 0;
constexpr uint64_t exit = 93;
constexpr uint64_t exit_group = 94;
constexpr uint64_t getpid = 172;
constexpr uint64_t kill = 129;
constexpr uint64_t read = 63;
constexpr uint64_t write = 64;
constexpr uint64_t open = 1024;
constexpr uint64_t openat = 56;
constexpr uint64_t close = 57;
constexpr uint64_t lseek = 62;
constexpr uint64_t brk = 214;
constexpr uint64_t link = 1025;
constexpr uint64_t unlink = 1026;
constexpr uint64_t mkdir = 1030;
constexpr uint64_t chdir = 49;
constexpr uint64_t getcwd = 17;
constexpr uint64_t stat = 1038;
constexpr uint64_t fstat = 80;
constexpr uint64_t lstat = 1039;
constexpr uint64_t fstatat = 79;
constexpr uint64_t access = 1033;
constexpr uint64_t faccessat = 48;
constexpr uint64_t pread = 67;
constexpr uint64_t pwrite = 68;
constexpr uint64_t uname = 160;
constexpr uint64_t getuid = 174;
constexpr uint64_t geteuid = 175;
constexpr uint64_t getgid = 176;
constexpr uint64_t getegid = 177;
constexpr uint64_t mmap = 222;
constexpr uint64_t munmap = 215;
constexpr uint64_t mremap = 216;
constexpr uint64_t time = 1062;
constexpr uint64_t getmainvars = 2011;
constexpr uint64_t rt_sigaction = 134;
constexpr uint64_t writev = 66;
constexpr uint64_t gettimeofday = 169;
constexpr uint64_t times = 153;
constexpr uint64_t fcntl = 25;
constexpr uint64_t getdents = 61;
constexpr uint64_t dup = 23;

// custom extensions
// the program reached code that must never run, so the execution went wrong
constexpr uint64_t host_error = 1;
constexpr uint64_t host_test_pass = 2;  // RISC-V test execution successfully completed
constexpr uint64_t host_test_fail = 3;  // RISC-V test execution failed
}  // namespace syscall_nr

struct SyscallHandler : public sc_core::sc_module, syscall_emulator_if {
	tlm_utils::simple_target_socket<SyscallHandler> tsock;
	std::unordered_map<uint64_t, iss_syscall_if *> cores;

	void register_core(ISS *core) {
		assert(cores.find(core->get_hart_id()) == cores.end());
		cores[core->get_hart_id()] = core;
	}

	SyscallHandler(sc_core::sc_module_name) {
		tsock.register_b_transport(this, &SyscallHandler::transport);
	}

	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		(void)delay;

		[[maybe_unused]] auto addr = trans.get_address();
		assert(addr % 4 == 0);
		assert(trans.get_data_length() == 4);
		auto hart_id = *((uint32_t *)trans.get_data_ptr());

		assert((cores.find(hart_id) != cores.end()) && "core not registered in syscall handler");

		execute_syscall(cores[hart_id]);
	}

	virtual void execute_syscall(iss_syscall_if *core) override {
		auto syscall = core->read_register(core->get_syscall_register_index());
		auto a3 = core->read_register(RegFile::a3);
		auto a2 = core->read_register(RegFile::a2);
		auto a1 = core->read_register(RegFile::a1);
		auto a0 = core->read_register(RegFile::a0);

		// printf("a7=%u, a0=%u, a1=%u, a2=%u, a3=%u\n", a7, a0, a1, a2, a3);

		auto ans = execute_syscall(syscall, a0, a1, a2, a3);

		core->write_register(RegFile::a0, ans);

		if (shall_exit)
			core->sys_exit();
	}

	uint8_t *mem = 0;     // direct pointer to start of guest memory in host memory
	uint64_t mem_offset;  // start address of the memory as mapped into the
	                      // address space
	uint64_t hp = 0;      // heap pointer
	bool shall_exit = false;
	bool shall_break = false;

	// only for memory consumption evaluation
	uint64_t start_heap = 0;
	uint64_t max_heap = 0;

	uint64_t get_max_heap_memory_consumption() {
		return max_heap - start_heap;
	}

	void init(uint8_t *host_memory_pointer, uint64_t mem_start_address, uint64_t heap_pointer_address) {
		mem = host_memory_pointer;
		mem_offset = mem_start_address;
		hp = heap_pointer_address;

		start_heap = hp;
		max_heap = hp;
	}

	uint8_t *guest_address_to_host_pointer(uintptr_t addr) {
		assert(mem != nullptr);

		return mem + (addr - mem_offset);
	}

	uint8_t *guest_to_host_pointer(void *p) {
		return guest_address_to_host_pointer((uintptr_t)p);
	}

	/*
	 * Syscalls are implemented to work directly on guest memory (represented in
	 * host as byte array). Note: the data structures on the host system might
	 * not be binary compatible with those on the guest system.
	 */
	int execute_syscall(uint64_t n, uint64_t _a0, uint64_t _a1, uint64_t _a2, uint64_t _a3);
};

}  // namespace rv32