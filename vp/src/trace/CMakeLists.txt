# Everything about execution sequence trees: recording them, analysing them and writing
# them out. The instruction set simulators feed this library and read nothing back except
# through the entry points in report.h.
#
# It depends on core-common for the instruction decoding (Opcode::Mapping, Instruction) and
# on nothing else in the VP. core-common must not depend on it, so a change here rebuilds
# the tracer and the two ISS translation units rather than the whole tree.
add_library(trace
	trace.cpp
	analysis.cpp
	report.cpp
	export_dot.cpp
	export_csv.cpp
	export_json.cpp
	interactive.cpp
	trace.h
	analysis.h
	report.h
	export.h)

target_include_directories(trace PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(trace PUBLIC core-common)
target_link_libraries(trace PRIVATE dl)
