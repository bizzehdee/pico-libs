if (NOT TARGET lis331_headers)
    add_library(lis331_headers INTERFACE)
    target_include_directories(lis331_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(lis331_headers INTERFACE i2c_device)
endif()

if (NOT TARGET lis331)
    add_library(lis331 INTERFACE)

    target_sources(lis331 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/lis331.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lis331hh.cpp
    )

    target_link_libraries(lis331 INTERFACE lis331_headers)
endif()
