file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_library(rv32
		iss.cpp
		syscall.cpp
        ${HEADERS})

target_link_libraries(rv32 systemc core-common softfloat dl)


if(COLOR_THEME STREQUAL "LIGHT")
	message("> using color theme LIGHT")
	target_compile_definitions(rv32 PRIVATE COLOR_THEME_LIGHT)
elseif(COLOR_THEME STREQUAL "DARK")
	message("> using color theme DARK")
	target_compile_definitions(rv32 PRIVATE COLOR_THEME_DARK)
endif()


target_include_directories(rv32 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
