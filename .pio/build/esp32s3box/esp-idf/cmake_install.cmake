# Install script for directory: C:/Users/lavco/.platformio/packages/framework-espidf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/ESP32-S3_ESP-IDF_COMPONENTS")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/newlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/__pio_env/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_gptimer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/app_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_event/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/nvs_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_pcnt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_mcpwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_ana_cmpr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_sdspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_sdio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_dac/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_tsens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_sdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_parlio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_usb_serial_jtag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/driver/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_phy/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_vfs_console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/vfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/lwip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_netif_stack/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_netif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/wpa_supplicant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_coex/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/bt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/unity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/cmock/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/http_parser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp-tls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_isp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_ppa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_touch_sens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_eth/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_gdbstub/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_hid/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/tcp_transport/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_http_client/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_http_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_https_ota/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_https_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_psram/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/protobuf-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/protocomm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_local_ctrl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/espcoredump/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/wear_levelling/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/idf_test/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/ieee802154/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/json/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/mqtt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/nvs_sec_provider/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/perfmon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/rt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/spiffs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/touch_element/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/ulp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/wifi_provisioning/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/src/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_kalman_motion/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_pressure_tendency/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_scalar_trend/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/sensirion_gas_index_algorithm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_type_utils/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_s12sd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_driver_i2c_ext/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ahtxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ak8975/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_as7341/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_bh1750/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_bmp280/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_bmp390/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ccs811/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ens160/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_hdc1080/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_hmc5883l/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ltr390uv/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_mlx90614/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_mmc56x3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_mpu6050/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_sgp4x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_sht4x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ssd1306/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_tlv493d/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_veml7700/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/onewire_bus/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_ds18b20/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_max31865/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_time_into_interval/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_datalogger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/lavco/OneDrive/Documents/PlatformIO/Projects/ESP32-S3_ESP-IDF_COMPONENTS/.pio/build/esp32s3box/esp-idf/esp_nvs_ext/cmake_install.cmake")
endif()

