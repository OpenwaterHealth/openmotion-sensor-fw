cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Generate version.h at configure time (initial)
find_package(Git QUIET)

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FW_GIT_DESCRIBE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_DESCRIBE_RESULT
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FW_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_SHA_RESULT
    )
    if(NOT GIT_DESCRIBE_RESULT EQUAL 0)
        set(FW_GIT_DESCRIBE "unknown")
    endif()
    if(NOT GIT_SHA_RESULT EQUAL 0)
        set(FW_GIT_SHA "unknown")
    endif()
else()
    set(FW_GIT_DESCRIBE "unknown")
    set(FW_GIT_SHA "unknown")
endif()

string(TIMESTAMP FW_BUILD_TIME_UTC "%Y-%m-%d %H:%M:%S UTC" UTC)

message(STATUS "FW version   : ${FW_GIT_DESCRIBE}")
message(STATUS "FW git sha   : ${FW_GIT_SHA}")
message(STATUS "FW build time: ${FW_BUILD_TIME_UTC}")

configure_file(
    ${CMAKE_SOURCE_DIR}/version.h.in
    ${CMAKE_BINARY_DIR}/generated/version.h
    @ONLY
)

# Set the project name
set(CMAKE_PROJECT_NAME motion-sensor-fw)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    USB/Class/COMMS/Src/usbd_comms.c
    USB/Class/CompositeBuilder/Src/usbd_composite_builder.c
    USB/Class/HISTO/Src/usbd_histo.c
    USB/Class/IMU/Src/usbd_imu.c
    Core/Src/0X02C1B.c
    Core/Src/camera_manager.c
    Core/Src/crosslink.c
    Core/Src/histo_fake.c
    Core/Src/i2c_protocol.c
    Core/Src/i2c_master.c
    Core/Src/i2c_health.c
    Core/Src/ICM20948.c
    Core/Src/if_commands.c
    Core/Src/lwrb.c
    Core/Src/jsmn.c
    Core/Src/logging.c
    Core/Src/uart_comms.c
    Core/Src/flash_eeprom.c
    Core/Src/motion_config.c
    Core/Src/system_monitor.c
    Core/Src/if_factory_prog.c
    Core/Src/ram_scrub.c
    Core/Src/utils.c
    USB/Core/Src/usbd_ioreq.c
    USB/Core/Src/usbd_desc.c
    USB/Core/Src/usbd_ctlreq.c
    USB/Core/Src/usbd_core.c
    USB/Core/Src/usbd_conf.c
    USB/Core/Src/usb_device.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_BINARY_DIR}/generated
    USB/Core/Inc
    USB/Class/CompositeBuilder/Inc
    USB/Class/COMMS/Inc
    USB/Class/HISTO/Inc
    USB/Class/IMU/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USE_USBD_COMPOSITE
    USE_USB_HS
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# FPGA bitstream binary (always fetch latest from GitHub releases)
set(FPGA_BITSTREAM_BIN ${CMAKE_SOURCE_DIR}/fpga/openmotion-camera-fpga.bin)
set(FPGA_BITSTREAM_URL "https://github.com/OpenwaterHealth/openmotion-camera-fpga/releases/latest/download/openmotion-camera-fpga.bin")
set(FPGA_BITSTREAM_ADDR 0x081A0000)

message(STATUS "Downloading latest FPGA bitstream from ${FPGA_BITSTREAM_URL} ...")
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/fpga)
file(DOWNLOAD
    ${FPGA_BITSTREAM_URL}
    ${FPGA_BITSTREAM_BIN}
    SHOW_PROGRESS
    STATUS FPGA_DOWNLOAD_STATUS
)
list(GET FPGA_DOWNLOAD_STATUS 0 FPGA_DOWNLOAD_ERROR)
if(NOT FPGA_DOWNLOAD_ERROR EQUAL 0)
    list(GET FPGA_DOWNLOAD_STATUS 1 FPGA_DOWNLOAD_MSG)
    message(FATAL_ERROR "Failed to download FPGA bitstream: ${FPGA_DOWNLOAD_MSG}")
endif()
message(STATUS "FPGA bitstream downloaded to ${FPGA_BITSTREAM_BIN}")

# Locate a Python interpreter for build-time scripts (e.g. merge_hex.py).
# Prefer python3, fall back to python (Windows installers often ship only "python").
find_program(PYTHON_EXECUTABLE NAMES python3 python)
if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR "Python interpreter not found (looked for 'python3' and 'python'). "
                        "Install Python 3 and ensure it is on PATH.")
endif()
message(STATUS "Using Python interpreter: ${PYTHON_EXECUTABLE}")

# Generate hex and bin files after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    # Generate firmware hex and bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}-raw.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}-raw.bin

    # Convert FPGA bitstream to hex at the target flash address
    COMMAND ${CMAKE_OBJCOPY} -I binary -O ihex
        --change-addresses ${FPGA_BITSTREAM_ADDR}
        ${FPGA_BITSTREAM_BIN}
        openmotion-camera-fpga.hex

    # Merge firmware + FPGA bitstream into a single hex file
    COMMAND ${CMAKE_COMMAND} -E echo "Merging firmware + FPGA bitstream hex files..."
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/merge_hex.py
        ${CMAKE_PROJECT_NAME}.hex
        ${CMAKE_PROJECT_NAME}-raw.hex
        openmotion-camera-fpga.hex

    # Generate combined bin from merged hex (for DFU flashing)
    COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary
        ${CMAKE_PROJECT_NAME}.hex
        ${CMAKE_PROJECT_NAME}.bin

    COMMENT "Generating .hex/.bin and merging FPGA bitstream at ${FPGA_BITSTREAM_ADDR}"
)

# Build + DFU-flash targets, one per sensor side. Invokes scripts/deploy.py.
# `--device` is required by the script, so we expose two separate targets.
find_program(PYTHON_FOR_DEPLOY NAMES python3 python)
if(PYTHON_FOR_DEPLOY)
    foreach(SIDE left right)
        add_custom_target(flash-${SIDE}
            COMMAND ${PYTHON_FOR_DEPLOY} ${CMAKE_SOURCE_DIR}/scripts/deploy.py
                    --device ${SIDE} --config $<CONFIG> --no-build
            DEPENDS ${CMAKE_PROJECT_NAME}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            USES_TERMINAL
            COMMENT "Flashing ${CMAKE_PROJECT_NAME} (${SIDE} sensor) via DFU"
        )
    endforeach()
else()
    message(WARNING "Python not found — `flash-left`/`flash-right` targets will not be available")
endif()
