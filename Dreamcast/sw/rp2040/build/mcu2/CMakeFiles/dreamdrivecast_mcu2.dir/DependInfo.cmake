
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_divider/divider.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_divider/divider.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_divider/divider.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_divider/divider.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_aeabi.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_aeabi.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_aeabi.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_aeabi.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S.obj"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/crt0.S" "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/build/mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/crt0.S.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "CFG_TUSB_MCU=OPT_MCU_RP2040"
  "CFG_TUSB_OS=OPT_OS_PICO"
  "LIB_PICO_BIT_OPS=1"
  "LIB_PICO_BIT_OPS_PICO=1"
  "LIB_PICO_DIVIDER=1"
  "LIB_PICO_DIVIDER_HARDWARE=1"
  "LIB_PICO_DOUBLE=1"
  "LIB_PICO_DOUBLE_PICO=1"
  "LIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1"
  "LIB_PICO_FLOAT=1"
  "LIB_PICO_FLOAT_PICO=1"
  "LIB_PICO_INT64_OPS=1"
  "LIB_PICO_INT64_OPS_PICO=1"
  "LIB_PICO_MALLOC=1"
  "LIB_PICO_MEM_OPS=1"
  "LIB_PICO_MEM_OPS_PICO=1"
  "LIB_PICO_MULTICORE=1"
  "LIB_PICO_PLATFORM=1"
  "LIB_PICO_PRINTF=1"
  "LIB_PICO_PRINTF_PICO=1"
  "LIB_PICO_RUNTIME=1"
  "LIB_PICO_STANDARD_LINK=1"
  "LIB_PICO_STDIO=1"
  "LIB_PICO_STDIO_USB=1"
  "LIB_PICO_STDLIB=1"
  "LIB_PICO_SYNC=1"
  "LIB_PICO_SYNC_CRITICAL_SECTION=1"
  "LIB_PICO_SYNC_MUTEX=1"
  "LIB_PICO_SYNC_SEM=1"
  "LIB_PICO_TIME=1"
  "LIB_PICO_UNIQUE_ID=1"
  "LIB_PICO_UTIL=1"
  "PICO_BOARD=\"pico\""
  "PICO_BUILD=1"
  "PICO_CMAKE_BUILD_TYPE=\"RelWithDebInfo\""
  "PICO_COPY_TO_RAM=1"
  "PICO_CXX_ENABLE_EXCEPTIONS=0"
  "PICO_NO_FLASH=0"
  "PICO_NO_HARDWARE=0"
  "PICO_ON_DEVICE=1"
  "PICO_RP2040_USB_DEVICE_UFRAME_FIX=1"
  "PICO_TARGET_NAME=\"dreamdrivecast_mcu2\""
  "PICO_USE_BLOCKED_RAM=0"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "mcu2"
  "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/mcu2"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_multicore/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_sync/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_sync/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_base/include"
  "generated/pico_base"
  "/Users/kaili/Code/pico-sdk/src/boards/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_platform/include"
  "/Users/kaili/Code/pico-sdk/src/rp2040/hardware_regs/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_base/include"
  "/Users/kaili/Code/pico-sdk/src/rp2040/hardware_structs/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_claim/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_time/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_timer/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_util/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_stdlib/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_gpio/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_uart/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_resets/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_clocks/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pll/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_vreg/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_watchdog/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_xosc/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_divider/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_runtime/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_printf/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_bit_ops/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_divider/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_malloc/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bootrom/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_binary_info/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_unique_id/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_flash/include"
  "/Users/kaili/Code/pico-sdk/src/common/pico_usb_reset_interface/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_int64_ops/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_mem_ops/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/boot_stage2/include"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/common"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/hw"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pio/include"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/audio/audio_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/audio/audio_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/audio/audio_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_rt_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_rt_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/dfu/dfu_rt_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/hid/hid_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/hid/hid_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/hid/hid_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/midi/midi_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/midi/midi_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/midi/midi_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/msc/msc_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/msc/msc_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/msc/msc_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ecm_rndis_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ecm_rndis_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ecm_rndis_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/usbtmc/usbtmc_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/usbtmc/usbtmc_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/usbtmc/usbtmc_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/video/video_device.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/video/video_device.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/class/video/video_device.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/common/tusb_fifo.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/common/tusb_fifo.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/common/tusb_fifo.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd_control.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd_control.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/device/usbd_control.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.obj.d"
  "/Users/kaili/Code/pico-sdk/lib/tinyusb/src/tusb.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/tusb.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/lib/tinyusb/src/tusb.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_sync/critical_section.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/critical_section.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/critical_section.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_sync/lock_core.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/lock_core.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/lock_core.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_sync/mutex.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/mutex.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/mutex.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_sync/sem.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/sem.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_sync/sem.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_time/time.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_time/time.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_time/time.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_time/timeout_helper.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_time/timeout_helper.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_time/timeout_helper.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_util/datetime.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/datetime.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/datetime.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_util/pheap.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/pheap.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/pheap.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/common/pico_util/queue.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/queue.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/common/pico_util/queue.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_claim/claim.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_claim/claim.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_claim/claim.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_clocks/clocks.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_clocks/clocks.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_clocks/clocks.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_flash/flash.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_flash/flash.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_flash/flash.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_gpio/gpio.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_gpio/gpio.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_gpio/gpio.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/irq.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/irq.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_irq/irq.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pio/pio.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pio/pio.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pio/pio.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pll/pll.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pll/pll.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_pll/pll.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_sync/sync.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_sync/sync.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_sync/sync.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_timer/timer.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_timer/timer.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_timer/timer.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_uart/uart.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_uart/uart.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_uart/uart.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_vreg/vreg.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_vreg/vreg.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_vreg/vreg.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_xosc/xosc.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_xosc/xosc.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/hardware_xosc/xosc.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_init_rom.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_init_rom.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_init_rom.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_math.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_math.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_double/double_math.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_init_rom.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_init_rom.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_init_rom.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_math.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_math.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_float/float_math.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_multicore/multicore.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_multicore/multicore.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_multicore/multicore.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_platform/platform.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_platform/platform.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_platform/platform.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_printf/printf.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_printf/printf.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_printf/printf.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_runtime/runtime.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_runtime/runtime.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_runtime/runtime.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio/stdio.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio/stdio.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio/stdio.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/reset_interface.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/reset_interface.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/reset_interface.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_unique_id/unique_id.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_unique_id/unique_id.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_unique_id/unique_id.c.obj.d"
  "/Users/kaili/Code/Dreamdrive/Dreamcast/sw/rp2040/mcu2/mcu2.c" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/mcu2.c.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/mcu2.c.obj.d"
  "/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp.obj" "gcc" "mcu2/CMakeFiles/dreamdrivecast_mcu2.dir/Users/kaili/Code/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp.obj.d"
  )

# Targets to which this target links which contain Fortran sources.
set(CMAKE_Fortran_TARGET_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
