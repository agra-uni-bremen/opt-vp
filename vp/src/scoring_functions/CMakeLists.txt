# A scoring library the VP loads at run time with --scoring-library.
#
# It exports one symbol, `score_functions`, an array of SF_BATCH_SIZE scoring functions. Its
# whole interface to the VP is `vp/src/trace/score.h`, so it links against nothing.

add_library(functions SHARED scoring_functions.cpp)

# score.h and the instruction definitions it needs
target_include_directories(functions PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")

set_target_properties(functions PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
