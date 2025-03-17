# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.5)

# Build as release but with debug symbols
set(CMAKE_BUILD_TYPE RelWithDebugInfo)
# set(CMAKE_BUILD_TYPE Debug)
set(PICO_PLATFORM rp2350-arm-s)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(dreamdrivecast C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Copy program to ram during boot
# set(PICO_COPY_TO_RAM 1)

add_compile_options(
        # -Wall
        # -save-temps
        # -Wno-unused-function # we have some for the docs that aren't called
        # -Wno-maybe-uninitialized
        -O0
)

# Initialize the SDK
pico_sdk_init()

# Include FreeRTOS integration
# add_subdirectory(lib/freertos-kernel/portable/ThirdParty/GCC/RP2040 FreeRTOS-Kernel)

# Include 3rd party SDCard implementation
add_subdirectory(lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src)

# Add applications
add_executable(dreamdrivecast
    rp2350.cpp
    sega_packet_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/utils/sd_utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils/hw_config.c
    ${CMAKE_CURRENT_LIST_DIR}/utils/gdrom_utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lib/nullDC/ImgReader/common.cpp
    ${CMAKE_CURRENT_LIST_DIR}/lib/nullDC/ImgReader/gdi.cpp
)

pico_generate_pio_header(dreamdrivecast
    ${CMAKE_CURRENT_LIST_DIR}/ide_handling.pio
)

target_include_directories(dreamdrivecast PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/library
    ${CMAKE_CURRENT_LIST_DIR}/utils
    # ${CMAKE_CURRENT_LIST_DIR}/lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src/ff15/source
    # ${CMAKE_CURRENT_LIST_DIR}/lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src/include
    ${CMAKE_CURRENT_LIST_DIR}/lib/nullDC/ImgReader
)

# pull in common dependencies
target_link_libraries(dreamdrivecast
    pico_multicore
    pico_stdlib
    hardware_pio

    # 3rd party libs
    # FreeRTOS-Kernel-Static # FreeRTOS kernel, static allocations
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)


# stdio config
pico_enable_stdio_usb(dreamdrivecast 0)
pico_enable_stdio_uart(dreamdrivecast 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(dreamdrivecast)

