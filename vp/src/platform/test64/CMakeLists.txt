# The riscv-tests platform for the rv64 core. It shares its source with test32-vp; the
# TEST_RV64 definition selects the core.
add_executable(test64-vp
        ../test32/test32_main.cpp)

target_compile_definitions(test64-vp PRIVATE TEST_RV64)
target_link_libraries(test64-vp rv64 platform-common gdb-mc ${Boost_LIBRARIES} systemc pthread)

INSTALL(TARGETS test64-vp RUNTIME DESTINATION bin)
