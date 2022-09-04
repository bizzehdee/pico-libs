if (NOT TARGET mpu6050_headers)
    add_library(mpu6050_headers INTERFACE)
    target_include_directories(mpu6050_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(mpu6050_headers INTERFACE hardware_i2c)
endif()

if (NOT TARGET mpu6050)
    add_library(mpu6050 INTERFACE)

    target_sources(mpu6050 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/mpu6050.cpp
    )

    target_link_libraries(mpu6050 INTERFACE mpu6050_headers hardware_timer pico_mem_ops)
endif()
