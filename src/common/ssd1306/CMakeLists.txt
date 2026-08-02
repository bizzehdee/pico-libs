if (NOT TARGET ssd1306_headers)
    add_library(ssd1306_headers INTERFACE)
    target_include_directories(ssd1306_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ssd1306_headers INTERFACE hardware_spi)
endif()

if (NOT TARGET ssd1306)
    add_library(ssd1306 INTERFACE)

    target_sources(ssd1306 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/ssd1306.cpp
    )

    target_link_libraries(ssd1306 INTERFACE ssd1306_headers hardware_gpio fonts)
endif()
