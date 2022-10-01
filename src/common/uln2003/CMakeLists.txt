if (NOT TARGET uln2003_headers)
    add_library(uln2003_headers INTERFACE)
    target_include_directories(uln2003_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(uln2003_headers INTERFACE)
endif()

if (NOT TARGET uln2003)
    add_library(uln2003 INTERFACE)

    target_sources(uln2003 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/uln2003.cpp
    )

    target_link_libraries(uln2003 INTERFACE uln2003_headers hardware_gpio)
endif()
