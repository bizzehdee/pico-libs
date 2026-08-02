if (NOT TARGET ads1x15_headers)
    add_library(ads1x15_headers INTERFACE)
    target_include_directories(ads1x15_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ads1x15_headers INTERFACE i2c_device)
endif()

if (NOT TARGET ads1x15)
    add_library(ads1x15 INTERFACE)

    target_sources(ads1x15 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ads1x15.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ads1015.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ads1115.cpp
    )

    target_link_libraries(ads1x15 INTERFACE ads1x15_headers)
endif()
