cmake_minimum_required(VERSION 3.14)
project(GrotiferCMake LANGUAGES CXX C)

# === C++ Standard ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# === Base Source Layout ===
file(GLOB_RECURSE SRC_MAIN     "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE SRC_TASKS    "${CMAKE_SOURCE_DIR}/src/tasks/*.cpp")
file(GLOB_RECURSE SRC_CORE  "${CMAKE_SOURCE_DIR}/src/core/*.cpp")
file(GLOB_RECURSE SRC_HARDWARE "${CMAKE_SOURCE_DIR}/src/hardware/*.cpp")

set(SOURCES
    ${SRC_MAIN}
    ${SRC_TASKS}
    ${SRC_CORE}
    ${SRC_HARDWARE}
)

# === Executable Target ===
add_executable(mainExe ${SOURCES})

# === Include Directories ===
# Your headers live in include/, vendor headers in MasterLibrary/GrotiferHardware
set(MASTER_LIB_DIR "${CMAKE_SOURCE_DIR}/MasterLibrary")

target_include_directories(mainExe
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        
)

# === External Dependencies ===
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# === System-installed Hardware Libraries ===
# These .so files should be installed on the system (e.g. /usr/local/lib)
list(APPEND CMAKE_LIBRARY_PATH
    /usr/lib/x86_64-linux-gnu
    /usr/lib/arm-linux-gnueabihf
    /usr/lib/aarch64-linux-gnu
)

find_library(LIB_EPOSCMD NAMES EposCmd REQUIRED)
find_library(LIB_LABJACK NAMES labjackusb REQUIRED)
find_library(LIB_FTD2XX  NAMES ftd2xx REQUIRED)
find_library(LIB_MODBUS  NAMES modbus REQUIRED)
# === Sanity Checks ===
if (NOT LIB_EPOSCMD)
    message(FATAL_ERROR "libEposCmd.so not found. Please install the Maxon EPOS SDK.")
endif()

if (NOT LIB_LABJACK)
    message(FATAL_ERROR "liblabjackusb.so not found. Please install the LabJack USB SDK.")
endif()

if (NOT LIB_FTD2XX)
    message(FATAL_ERROR "libftd2xx.so not found. Please install the FTDI D2XX SDK.")
endif()

if (NOT LIB_MODBUS)
    message(FATAL_ERROR "libmodbus.so not found. Try: sudo apt install libmodbus-dev")
endif()

# === Link All Required Libraries ===
target_link_libraries(mainExe PRIVATE
    ${LIB_EPOSCMD}
    ${LIB_LABJACK}
    ${LIB_FTD2XX}
    ${LIB_MODBUS}
    Threads::Threads
    Eigen3::Eigen
    stdc++fs
)

# === External Libraries ===
add_subdirectory(external/labjack)
target_link_libraries(mainExe PRIVATE labjack_u6)

# === Architecture Printout (Optional) ===
if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HW_ARCH "Arm")
else()
    set(HW_ARCH "x86")
endif()
message(STATUS "Building for architecture: ${HW_ARCH}")

# === RPATH Settings (so you don't need LD_LIBRARY_PATH manually) ===
set_target_properties(mainExe PROPERTIES
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH "/usr/local/lib;/usr/lib"
)
