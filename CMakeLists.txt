cmake_minimum_required(VERSION 3.14)
project(GrotiferCMake LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#
# 1) Tell CMake where your “MasterLibrary/” lives,
#    i.e. where GrotiferMaster.hpp and GrotiferHardware/… are
#
set(MASTER_LIB_DIR "${CMAKE_SOURCE_DIR}/MasterLibrary")

# === Source files ===
file(GLOB_RECURSE SRC_MAIN "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE SRC_TASKS "${CMAKE_SOURCE_DIR}/src/tasks/*.cpp")
file(GLOB_RECURSE SRC_SOLVERS "${CMAKE_SOURCE_DIR}/src/solvers/*.cpp")
file(GLOB_RECURSE SRC_HARDWARE "${CMAKE_SOURCE_DIR}/src/hardware/*.cpp")

set(SOURCES
    ${SRC_MAIN}
    ${SRC_TASKS}
    ${SRC_SOLVERS}
    ${SRC_HARDWARE}
)

# === Executable ===
add_executable(mainExe ${SOURCES})

# === Include directories ===
target_include_directories(mainExe
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

# === External libraries ===
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(mainExe PRIVATE Eigen3::Eigen)


# 5) If you have prebuilt libraries (either .so or .lib/.dll),
#    decide which directory to search. Example using “x86” vs “Arm” subfolders:
if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR
    CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HW_ARCH "Arm")
else()
    set(HW_ARCH "x86")
endif()

# Print the selected architecture
message(STATUS "Selected hardware architecture: ${HW_ARCH}")

set(HARDWARE_SOLIB_DIR
    "${MASTER_LIB_DIR}/GrotiferHardware/HardwareSoLib/${HW_ARCH}"
)

# Sanity check:
if (NOT EXISTS "${HARDWARE_SOLIB_DIR}")
    message(FATAL_ERROR
        "Cannot find HardwareSoLib folder: ${HARDWARE_SOLIB_DIR}"
    )
endif()
file(GLOB SOLIBS "${HARDWARE_SOLIB_DIR}/*")
message(STATUS "Available libraries in ${HARDWARE_SOLIB_DIR}: ${SOLIBS}")

# 6) Tell the linker to search HARDWARE_SOLIB_DIR at link time
link_directories(
    "${HARDWARE_SOLIB_DIR}"
)

# 7) Explicitly link the libraries you need.
#    If you have:
#      libEposCmd.so   (Linux)  → link with “EposCmd”
#      libftd2xx.so    → link with “ftd2xx”
#      liblabjackusb.so→ link with “labjackusb”
#      libmodbus.so    → link with “modbus”
#
#    Or, if you’re on Windows with .lib/.dll:
#      EposCmd.lib     → link with “EposCmd”
#      ftd2xx.lib      → link with “ftd2xx”
#      labjackusb.lib  → link with “labjackusb”
#      modbus.lib      → link with “modbus”
#
target_link_libraries(mainExe PUBLIC
    "${HARDWARE_SOLIB_DIR}/libEposCmd.so"
    "${HARDWARE_SOLIB_DIR}/libftd2xx.so"
    "${HARDWARE_SOLIB_DIR}/liblabjackusb.so"
    "${HARDWARE_SOLIB_DIR}/libmodbus.so"
)

# 8) (Optional) Embed a runtime path so you don’t have to set LD_LIBRARY_PATH
#    at runtime (mostly relevant on Linux/macOS). On Windows this does nothing.
set_target_properties(mainExe PROPERTIES
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH "${HARDWARE_SOLIB_DIR}"
)