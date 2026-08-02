if (NOT TARGET mpl115a2_headers)
    add_library(mpl115a2_headers INTERFACE)
    target_include_directories(mpl115a2_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(mpl115a2_headers INTERFACE i2c_device)
endif()

if (NOT TARGET mpl115a2)
    add_library(mpl115a2 INTERFACE)

    target_sources(mpl115a2 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mpl115a2.cpp
    )

    target_link_libraries(mpl115a2 INTERFACE mpl115a2_headers)
endif()
