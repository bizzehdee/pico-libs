if (NOT TARGET drv8830_headers)
    add_library(drv8830_headers INTERFACE)
    target_include_directories(drv8830_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(drv8830_headers INTERFACE i2c_device)
endif()

if (NOT TARGET drv8830)
    add_library(drv8830 INTERFACE)

    target_sources(drv8830 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drv8830.cpp
    )

    target_link_libraries(drv8830 INTERFACE drv8830_headers)
endif()
