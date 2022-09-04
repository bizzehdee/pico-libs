if (NOT TARGET i2c_device_headers)
    add_library(i2c_device_headers INTERFACE)
    target_include_directories(i2c_device_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(i2c_device_headers INTERFACE hardware_i2c)
endif()

if (NOT TARGET i2c_device)
    add_library(i2c_device INTERFACE)

    target_sources(i2c_device INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/i2c_device.cpp
    ${CMAKE_CURRENT_LIST_DIR}/bus_register.cpp
    ${CMAKE_CURRENT_LIST_DIR}/bus_register_bits.cpp
    )

    target_link_libraries(i2c_device INTERFACE i2c_device_headers)
endif()
