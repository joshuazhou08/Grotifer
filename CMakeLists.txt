cmake_minimum_required(VERSION 3.14)
project(GrotiferCMake LANGUAGES CXX C)

# ============================================================
# C++ Standard
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# SHARED LIBRARY SOURCES (all production code)
# ============================================================
file(GLOB_RECURSE LIB_SRC
    "${CMAKE_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/hardware/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/tasks/*.cpp"
)

# Remove test files from the library
file(GLOB_RECURSE TEST_SRC "${CMAKE_SOURCE_DIR}/src/tests/*.cpp")
list(REMOVE_ITEM LIB_SRC ${TEST_SRC})

# Build static library
add_library(grotifer_core STATIC ${LIB_SRC})

target_include_directories(grotifer_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# ============================================================
# External Libraries
# ============================================================
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

list(APPEND CMAKE_LIBRARY_PATH
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
)

find_library(LIB_EPOSCMD NAMES EposCmd REQUIRED)
find_library(LIB_LABJACK NAMES labjackusb REQUIRED)
find_library(LIB_FTD2XX  NAMES ftd2xx REQUIRED)
find_library(LIB_MODBUS  NAMES modbus REQUIRED)

target_link_libraries(grotifer_core PUBLIC
    ${LIB_EPOSCMD}
    ${LIB_LABJACK}
    ${LIB_FTD2XX}
    ${LIB_MODBUS}
    Threads::Threads
    Eigen3::Eigen
    labjack_u6
    stdc++fs
)

# ============================================================
# MAIN EXECUTABLE
# ============================================================
file(GLOB_RECURSE MAIN_SRC "${CMAKE_SOURCE_DIR}/src/main.cpp")

add_executable(mainExe ${MAIN_SRC})
target_link_libraries(mainExe PRIVATE grotifer_core)

# ============================================================
# STEPPER TEST EXECUTABLE (only test + shared library)
# ============================================================
file(GLOB_RECURSE STEPPER_TEST_SRC
    "${CMAKE_SOURCE_DIR}/src/tests/*.cpp"
)

add_executable(stepperTest ${STEPPER_TEST_SRC})
target_link_libraries(stepperTest PRIVATE grotifer_core)

# ============================================================
# External LabJack Library
# ============================================================
add_subdirectory(external/labjack)

# ============================================================
# Architecture Printout
# ============================================================
if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HW_ARCH "Arm")
else()
    set(HW_ARCH "x86")
endif()
message(STATUS "Building for architecture: ${HW_ARCH}")

# ============================================================
# RPATH
# ============================================================
set_target_properties(mainExe PROPERTIES
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH "/usr/local/lib;/usr/lib"
)

set_target_properties(stepperTest PROPERTIES
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH "/usr/local/lib;/usr/lib"
)
