add_subdirectory(softfloat)
if(NOT USE_SYSTEM_SYSTEMC)
    include(ExternalProject)
    # SOURCE_DIR builds the submodule in place, and BUILD_ALWAYS runs SystemC's own build on
    # every build of the VP, which recompiles what changed after a submodule update. SystemC
    # gets the VP's build type, so a Release VP links an optimised SystemC kernel.
    ExternalProject_Add(
        systemc_project
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/systemc
        CMAKE_ARGS
          -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
          -DCMAKE_CXX_FLAGS=-w
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_INSTALL_LIBDIR=lib #don't use lib64
          -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
          -DBUILD_SHARED_LIBS=OFF
          -DENABLE_EXAMPLES=OFF
        INSTALL_COMMAND ""
        BUILD_ALWAYS ON
        BINARY_DIR ${CMAKE_BINARY_DIR}/systemc-build
        BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/systemc-build/src/libsystemc.a
    )

    add_library(systemc STATIC IMPORTED GLOBAL)
    set_target_properties(systemc PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/systemc/src
        IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/systemc-build/src/libsystemc.a)
    add_dependencies(systemc systemc_project)
elseif(TARGET SystemC::systemc)
    add_library(systemc INTERFACE)
    target_link_libraries(systemc INTERFACE SystemC::systemc)
endif()
