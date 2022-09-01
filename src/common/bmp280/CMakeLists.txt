if (NOT TARGET bmp280_headers)
    add_library(bmp280_headers INTERFACE)
    target_include_directories(bmp280_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(bmp280_headers INTERFACE hardware_i2c)
endif()

if (NOT TARGET bmp280)
    add_library(bmp280 INTERFACE)

    target_sources(bmp280 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/bmp280.cpp
    )

    target_link_libraries(bmp280 INTERFACE bmp280_headers)
endif()
