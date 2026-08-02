if (NOT TARGET ccs811_headers)
    add_library(ccs811_headers INTERFACE)
    target_include_directories(ccs811_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ccs811_headers INTERFACE i2c_device)
endif()

if (NOT TARGET ccs811)
    add_library(ccs811 INTERFACE)

    target_sources(ccs811 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ccs811.cpp
    )

    target_link_libraries(ccs811 INTERFACE ccs811_headers)
endif()
