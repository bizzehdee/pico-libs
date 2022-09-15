if (NOT TARGET ili934x_headers)
    add_library(ili934x_headers INTERFACE)
    target_include_directories(ili934x_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ili934x_headers INTERFACE hardware_spi)
endif()

if (NOT TARGET ili934x)
    add_library(ili934x INTERFACE)

    target_sources(ili934x INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/ili934x.cpp
    )

    target_link_libraries(ili934x INTERFACE ili934x_headers hardware_gpio)
endif()
