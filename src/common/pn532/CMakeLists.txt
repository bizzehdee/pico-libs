if (NOT TARGET pn532_headers)
    add_library(pn532_headers INTERFACE)
    target_include_directories(pn532_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pn532_headers INTERFACE i2c_device)
endif()

if (NOT TARGET pn532)
    add_library(pn532 INTERFACE)

    target_sources(pn532 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pn532.cpp
    )

    target_link_libraries(pn532 INTERFACE pn532_headers)
endif()
