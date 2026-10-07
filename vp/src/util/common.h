#pragma once

#include <stdexcept>
#include <sysc/kernel/sc_ver.h>

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

#define UNUSED(x) (void)(x)

inline void ensure(bool cond) {
	if (unlikely(!cond))
		throw std::runtime_error("runtime assertion failed");
}

inline void ensure(bool cond, const std::string &reason) {
	if (unlikely(!cond))
		throw std::runtime_error(reason);
}

inline uint64_t rv64_align_address(uint64_t addr) {
	return addr - addr % 8;
}

inline uint32_t rv32_align_address(uint32_t addr) {
	return addr - addr % 4;
}

/* Allow to provide a custom function name for a SystemC thread to avoid duplicate name warning in case the same
 * SystemC module is instantiated multiple times. */
#if SC_VERSION_MAJOR >= 3
#define SC_NAMED_THREAD(func, name) \
	this->declare_thread_process(SC_MAKE_FUNC_PTR(SC_CURRENT_USER_MODULE_TYPE, func), name)
#else
#define SC_NAMED_THREAD(func, name) declare_thread_process(func##_handle, name, SC_CURRENT_USER_MODULE, func)
#endif
