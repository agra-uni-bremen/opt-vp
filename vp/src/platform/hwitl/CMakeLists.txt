file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h*)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)

add_git_submodule(virtual-bus)
# readStruct in virtual-bus/network_io.hpp stores the ssize_t result of read() in an unsigned,
# so an error return of -1 becomes a size near 4 GiB. GCC warns about it. The code belongs to
# the virtual-bus repository, so the warning is silenced here instead of fixed.
target_compile_options(virtual-bus PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-stringop-overflow>)

add_executable(hwitl-vp
	${SOURCES}
)

target_link_libraries(hwitl-vp rv32 virtual-bus platform-common gdb-mc ${Boost_LIBRARIES} systemc pthread)

INSTALL(TARGETS hwitl-vp RUNTIME DESTINATION bin)