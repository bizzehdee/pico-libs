add_library(pico_libs_included INTERFACE)
target_compile_definitions(pico_libs_included INTERFACE
        -DPICO_LIBS=1
        )

pico_add_platform_library(pico_libs_included)

# note as we're a .cmake included by the SDK, we're relative to the pico-sdk build
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src ${CMAKE_BINARY_DIR}/pico_libs/src)

if (PICO_LIBS_TESTS_ENABLED OR PICO_LIBS_TOP_LEVEL_PROJECT)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test {CMAKE_BINARY_DIR}/pico_libs/test)
endif ()

