add_library(labjack_u6 STATIC
    u6.c
)

target_include_directories(labjack_u6 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against the system-installed liblabjackusb.so
find_library(LIB_LABJACK NAMES labjackusb PATHS /usr/lib /usr/local/lib REQUIRED)
target_link_libraries(labjack_u6 PUBLIC ${LIB_LABJACK})