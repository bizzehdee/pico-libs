if (NOT TARGET mcp23xxx_headers)
    add_library(mcp23xxx_headers INTERFACE)
    target_include_directories(mcp23xxx_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(mcp23xxx_headers INTERFACE hardware_i2c i2c_device)
endif()

if (NOT TARGET mcp23xxx)
    add_library(mcp23xxx INTERFACE)

    target_sources(mcp23xxx INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mcp23xxx.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mcp23x08.cpp
    ${CMAKE_CURRENT_LIST_DIR}/mcp23x17.cpp
    )

    target_link_libraries(mcp23xxx INTERFACE mcp23xxx_headers)
endif()
