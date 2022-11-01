if (NOT TARGET pca9685_headers)
    add_library(pca9685_headers INTERFACE)
    target_include_directories(pca9685_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pca9685_headers INTERFACE i2c_device)
endif()

if (NOT TARGET pca9685)
    add_library(pca9685 INTERFACE)

    target_sources(pca9685 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/pca9685.cpp
    )

    target_link_libraries(pca9685 INTERFACE pca9685_headers hardware_gpio)
endif()
