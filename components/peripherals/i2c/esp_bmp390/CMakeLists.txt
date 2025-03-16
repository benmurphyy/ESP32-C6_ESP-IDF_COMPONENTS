idf_component_register(
    SRCS bmp390.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c esp_type_utils log esp_common esp_timer
)
