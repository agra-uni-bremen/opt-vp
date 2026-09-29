cmake_minimum_required(VERSION 3.18)
project(riscv-vp
	VERSION 1.2.2
)

option(USE_SYSTEM_SYSTEMC "use systemc version provided by the system" OFF)

# maximum length of the traced instruction sequences. Nodes keep static arrays of this size,
# so it is fixed at compile time. The effective depth can be lowered at runtime with --trace-depth.
set(INSTRUCTION_TREE_DEPTH "" CACHE STRING "maximum instruction tree depth (default: 20, see vp/src/trace/trace.h)")

# optional trace features, see vp/src/trace/trace.h
option(NO_TRACE_PARAMETER_IMMEDIATES "do not record decoded immediates (smaller traces, faster)" OFF)
option(NO_TRACE_PREDECESSOR_PCS "do not record predecessor pcs per register set" OFF)
option(NO_TRACE_BRANCH_OUTCOMES "do not record per-pc branch taken/not-taken counts" OFF)
option(TRACE_ROOT_PARAMETERS "also record parameters on the root node of each tree" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 17)
# Keep whatever the caller passed in -DCMAKE_CXX_FLAGS and only add the warnings on top.
# Overwriting the variable silently dropped every flag given on the command line, which made
# it impossible to build with a sanitizer or with -g.
#   cmake -S vp -B vp/build-asan -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
#         -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -Wpedantic ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(INSTRUCTION_TREE_DEPTH)
	add_compile_definitions(INSTRUCTION_TREE_DEPTH=${INSTRUCTION_TREE_DEPTH})
endif()
foreach(TRACE_FLAG NO_TRACE_PARAMETER_IMMEDIATES NO_TRACE_PREDECESSOR_PCS NO_TRACE_BRANCH_OUTCOMES TRACE_ROOT_PARAMETERS)
	if(${TRACE_FLAG})
		add_compile_definitions(${TRACE_FLAG})
	endif()
endforeach()

# Allows running tests without invoking `make install` first.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})

include(cmake/AddGitSubmodule.cmake)

find_package( Boost REQUIRED COMPONENTS iostreams program_options log)

if(USE_SYSTEM_SYSTEMC)
	find_library(SystemC libsystemc.a)
endif()

add_subdirectory(src)

enable_testing()
list(APPEND CMAKE_CTEST_ARGUMENTS "--verbose")

add_test(NAME libgdb
	COMMAND ./test.sh
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/libgdb")
add_test(NAME gdb
	COMMAND ./test.sh
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/gdb")
add_test(NAME integration
	COMMAND ./test.sh
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/tests/integration")
add_test(NAME sw
	COMMAND ./test.sh
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/../sw")

# Checks that the trace the VP writes is the trace it wrote before. This is the one
# to run after every change; it takes about two seconds and needs no cross compiler,
# only the example programs under sw/ to have been built once.
add_test(NAME trace
	COMMAND ./test.sh
	WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/../tests/trace")

set_tests_properties(gdb integration sw PROPERTIES ENVIRONMENT
	PATH=$ENV{PATH}:${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set_tests_properties(trace PROPERTIES ENVIRONMENT
	VP_BIN_DIR=${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set_tests_properties(libgdb PROPERTIES ENVIRONMENT
	RISCV_VP_BASE=${CMAKE_CURRENT_SOURCE_DIR}/..)

# Minimal build target: build a subset of platform binaries in one CMake target
# so `cmake --build --target minimal` shows a unified progress indicator.
add_custom_target(minimal
	DEPENDS
		riscv-vp
		linux-vp
		linux32-vp
		tiny32-vp
		tiny64-vp
		tiny32-mc
		tiny64-mc
		microrv32-vp
	COMMENT "Build minimal set of riscv-vp binaries (no GUI/hwitl/virtual-breadboard/test32)"
)

# The binaries a change is normally checked against. tiny64-vp is here because the trace
# reference check runs a case on it, and a stale binary would pass that case silently.
add_custom_target(essential
	DEPENDS
		riscv-vp
		linux32-vp
		tiny32-vp
		tiny64-vp
		microrv32-vp
	COMMENT "Build essential set of riscv-vp binaries (no GUI/hwitl/virtual-breadboard/test32)"
)
