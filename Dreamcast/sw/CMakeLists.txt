cmake_minimum_required(VERSION 3.12)

# Build as release but with debug symbols
set(CMAKE_BUILD_TYPE RelWithDebInfo)

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
        -O3
)

# Initialize the SDK
pico_sdk_init()

# Include FreeRTOS integration
# add_subdirectory(lib/freertos-kernel/portable/ThirdParty/GCC/RP2040 FreeRTOS-Kernel)

# Include 3rd party SDCard implementation
# add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/src)

# Add shared code
add_subdirectory(shared)
add_subdirectory(mcu1/lib/sdcard/no-OS-FatFS-SD-SPI-RPi-Pico/src)

# Add applications
add_subdirectory(mcu1)
add_subdirectory(mcu2)
