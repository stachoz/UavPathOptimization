cmake_minimum_required(VERSION 3.15)
project(UavPathOptimization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ====================================================================
# Find the CPLEX installation
# ====================================================================
# This script will automatically find CPLEX if the CPLEX_ROOT cache variable
# or the CPLEX_STUDIO_DIR environment variable is set.
#
# If CPLEX is not found, you can specify the path when you configure CMake:
#
# From the command line:
# cmake -DCPLEX_ROOT="C:/Program Files/IBM/ILOG/CPLEX_Studio2211" .
#
# Or in the CLion CMake settings:
# -DCPLEX_ROOT="C:/Program Files/IBM/ILOG/CPLEX_Studio2211"
# ====================================================================

set(CPLEX_ROOT "" CACHE PATH "Path to the CPLEX Studio installation directory.")

if(NOT CPLEX_ROOT AND DEFINED ENV{CPLEX_STUDIO_DIR})
    file(TO_CMAKE_PATH "$ENV{CPLEX_STUDIO_DIR}" CPLEX_ROOT)
    message(STATUS "Found CPLEX in environment variable CPLEX_STUDIO_DIR: ${CPLEX_ROOT}")
endif()

if(NOT CPLEX_ROOT OR NOT EXISTS "${CPLEX_ROOT}/cplex/include/ilcplex/ilocplex.h")
    message(FATAL_ERROR "CPLEX installation not found or invalid. "
            "Please set the CPLEX_ROOT CMake variable to the root of your CPLEX Studio installation. "
            "For example: -DCPLEX_ROOT=\"C:/Program Files/IBM/ILOG/CPLEX_Studio2211\"")
else()
    message(STATUS "Found valid CPLEX installation at: ${CPLEX_ROOT}")
endif()

add_library(cplex_interface INTERFACE)

target_include_directories(cplex_interface INTERFACE
        "${CPLEX_ROOT}/cplex/include"
        "${CPLEX_ROOT}/concert/include"
)

target_compile_definitions(cplex_interface INTERFACE IL_STD)

if(MSVC)
    # --- Windows (MSVC) ---
    message(STATUS "Configuring for Windows (MSVC)")
    if ("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
        set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x64_windows_msvc14/stat_mdd")
        set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x64_windows_msvc14/stat_mdd")
        target_compile_options(cplex_interface INTERFACE /MDd)
    else()
        set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x64_windows_msvc14/stat_mda")
        set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x64_windows_msvc14/stat_mda")
        target_compile_options(cplex_interface INTERFACE /MD)
    endif()

    target_link_libraries(cplex_interface INTERFACE
            "${CPLEX_LIB_DIR}/cplex2211.lib"
            "${CPLEX_LIB_DIR}/ilocplex.lib"
            "${CONCERT_LIB_DIR}/concert.lib"
    )
    # Suppress common warnings from CPLEX headers in MSVC
    target_compile_options(cplex_interface INTERFACE /wd4068 /wd4244 /wd4267)

elseif(UNIX)
    # --- Linux / macOS ---
    message(STATUS "Configuring for UNIX")
    find_package(Threads REQUIRED)

    # Adjust this path based on the actual Linux library location
    set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic")
    set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic")

    target_link_libraries(cplex_interface INTERFACE
            "${CPLEX_LIB_DIR}/libcplex.a"
            "${CPLEX_LIB_DIR}/libilocplex.a"
            "${CONCERT_LIB_DIR}/libconcert.a"
            Threads::Threads # Link against pthreads
    )
    target_link_libraries(cplex_interface INTERFACE -ldl -lm)
endif()


add_executable(${PROJECT_NAME} main.cpp
)

target_sources(${PROJECT_NAME} PUBLIC
        file.h
)

target_link_libraries(${PROJECT_NAME} PRIVATE cplex_interface)

target_compile_definitions(${PROJECT_NAME} PUBLIC
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)
