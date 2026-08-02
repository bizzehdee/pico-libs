if (NOT TARGET ws2812_headers)
    add_library(ws2812_headers INTERFACE)
    target_include_directories(ws2812_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ws2812_headers INTERFACE hardware_pio hardware_clocks)
endif()

if (NOT TARGET ws2812)
    add_library(ws2812 INTERFACE)

    target_sources(ws2812 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ws2812.cpp
    )

    target_link_libraries(ws2812 INTERFACE ws2812_headers)
endif()
