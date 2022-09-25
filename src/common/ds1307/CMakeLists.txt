if (NOT TARGET ds1307_headers)
    add_library(ds1307_headers INTERFACE)
    target_include_directories(ds1307_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(ds1307_headers INTERFACE hardware_i2c i2c_device hardware_rtc)
endif()

if (NOT TARGET ds1307)
    add_library(ds1307 INTERFACE)

    target_sources(ds1307 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/ds1307.cpp
    )

    target_link_libraries(ds1307 INTERFACE ds1307_headers)
endif()
