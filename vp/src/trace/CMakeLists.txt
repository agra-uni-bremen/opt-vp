# Everything about execution sequence trees: recording them, analysing them and writing them
# out. A core owns a Tracer (tracer.h), configures it from a TraceConfig (config.h), and feeds
# it one step per executed instruction.
#
# This library depends on core-common for the instruction decoding and on nothing else in the
# VP. Keep core-common independent of it, so a change here rebuilds the tracer and the two ISS
# translation units rather than the whole tree.
add_library(trace
	trace.cpp
	tracer.cpp
	analysis.cpp
	report.cpp
	export_dot.cpp
	export_csv.cpp
	export_json.cpp
	interactive.cpp
	config.h
	trace.h
	tracer.h
	analysis.h
	report.h
	export.h)

target_include_directories(trace PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(trace PUBLIC core-common)
target_link_libraries(trace PRIVATE dl)
