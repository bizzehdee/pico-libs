if (NOT TARGET rtclib_headers)
    add_library(rtclib_headers INTERFACE)
    target_include_directories(rtclib_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(rtclib_headers INTERFACE i2c_device hardware_rtc)
endif()

if (NOT TARGET rtclib)
    add_library(rtclib INTERFACE)

    target_sources(rtclib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/rtc_i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ds3231.cpp
    )

    target_link_libraries(rtclib INTERFACE rtclib_headers)
endif()
