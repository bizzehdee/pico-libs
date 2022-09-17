if (NOT TARGET fonts_headers)
    add_library(fonts_headers INTERFACE)
    target_include_directories(fonts_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(fonts_headers INTERFACE)
endif()

if (NOT TARGET fonts)
    add_library(fonts INTERFACE)

    target_sources(fonts INTERFACE
    
    )

    target_link_libraries(fonts INTERFACE fonts_headers)
endif()
