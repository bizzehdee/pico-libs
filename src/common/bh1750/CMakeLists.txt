if (NOT TARGET bh1750_headers)
    add_library(bh1750_headers INTERFACE)
    target_include_directories(bh1750_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(bh1750_headers INTERFACE hardware_i2c i2c_device)
endif()

if (NOT TARGET bh1750)
    add_library(bh1750 INTERFACE)

    target_sources(bh1750 INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/bh1750.cpp
    )

    target_link_libraries(bh1750 INTERFACE bh1750_headers)
endif()
