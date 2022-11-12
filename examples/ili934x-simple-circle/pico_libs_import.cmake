# This is a copy of <PICO_LIBS_PATH>/external/pico_libs_import.cmake

# This can be dropped into an external project to help locate pico_libs
# It should be include()ed prior to project()

if (DEFINED ENV{PICO_LIBS_PATH} AND (NOT PICO_LIBS_PATH))
    set(PICO_LIBS_PATH $ENV{PICO_LIBS_PATH})
    message("Using PICO_LIBS_PATH from environment ('${PICO_LIBS_PATH}')")
endif ()

if (DEFINED ENV{PICO_LIBS_FETCH_FROM_GIT} AND (NOT PICO_LIBS_FETCH_FROM_GIT))
    set(PICO_LIBS_FETCH_FROM_GIT $ENV{PICO_LIBS_FETCH_FROM_GIT})
    message("Using PICO_LIBS_FETCH_FROM_GIT from environment ('${PICO_LIBS_FETCH_FROM_GIT}')")
endif ()

if (DEFINED ENV{PICO_LIBS_FETCH_FROM_GIT_PATH} AND (NOT PICO_LIBS_FETCH_FROM_GIT_PATH))
    set(PICO_LIBS_FETCH_FROM_GIT_PATH $ENV{PICO_LIBS_FETCH_FROM_GIT_PATH})
    message("Using PICO_LIBS_FETCH_FROM_GIT_PATH from environment ('${PICO_LIBS_FETCH_FROM_GIT_PATH}')")
endif ()

if (NOT PICO_LIBS_PATH)
    if (PICO_LIBS_FETCH_FROM_GIT)
        include(FetchContent)
        set(FETCHCONTENT_BASE_DIR_SAVE ${FETCHCONTENT_BASE_DIR})
        if (PICO_LIBS_FETCH_FROM_GIT_PATH)
            get_filename_component(FETCHCONTENT_BASE_DIR "${PICO_LIBS_FETCH_FROM_GIT_PATH}" REALPATH BASE_DIR "${CMAKE_SOURCE_DIR}")
        endif ()
        FetchContent_Declare(
                pico_libs
                GIT_REPOSITORY https://github.com/bizzehdee/pico_libs
                GIT_TAG master
        )
        if (NOT pico_libs)
            message("Downloading Raspberry Pi Pico Extras")
            FetchContent_Populate(pico_libs)
            set(PICO_LIBS_PATH ${pico_libs_SOURCE_DIR})
        endif ()
        set(FETCHCONTENT_BASE_DIR ${FETCHCONTENT_BASE_DIR_SAVE})
    else ()
        if (PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/../pico_libs")
            set(PICO_LIBS_PATH ${PICO_SDK_PATH}/../pico_libs)
            message("Defaulting PICO_LIBS_PATH as sibling of PICO_SDK_PATH: ${PICO_LIBS_PATH}")
        else()
            message(FATAL_ERROR
                    "PICO LIBS location was not specified. Please set PICO_LIBS_PATH or set PICO_LIBS_FETCH_FROM_GIT to on to fetch from git."
                    )
        endif()
    endif ()
endif ()

set(PICO_LIBS_PATH "${PICO_LIBS_PATH}" CACHE PATH "Path to the PICO LIBS")
set(PICO_LIBS_FETCH_FROM_GIT "${PICO_LIBS_FETCH_FROM_GIT}" CACHE BOOL "Set to ON to fetch copy of PICO LIBS from git if not otherwise locatable")
set(PICO_LIBS_FETCH_FROM_GIT_PATH "${PICO_LIBS_FETCH_FROM_GIT_PATH}" CACHE FILEPATH "location to download LIBS")

get_filename_component(PICO_LIBS_PATH "${PICO_LIBS_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_LIBS_PATH})
    message(FATAL_ERROR "Directory '${PICO_LIBS_PATH}' not found")
endif ()

set(PICO_LIBS_PATH ${PICO_LIBS_PATH} CACHE PATH "Path to the PICO LIBS" FORCE)

add_subdirectory(${PICO_LIBS_PATH} pico_libs)
