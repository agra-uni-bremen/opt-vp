# CMakeLists.txt for functions library

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.18)

# Project name
project(FunctionsLibrary)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files for the library
set(SOURCE_FILES scoring_functions.cpp)

# Include directories
include_directories("${CMAKE_SOURCE_DIR}/../lib/json/single_include")  # Add this line

# Create the shared library
add_library(functions SHARED ${SOURCE_FILES})

# Set the library output name
set_target_properties(functions PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Optionally, set the library installation directory
# For simplicity, you can comment out the following line
# install(TARGETS functions LIBRARY DESTINATION lib)
