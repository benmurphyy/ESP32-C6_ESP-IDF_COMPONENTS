idf_component_register(
    SRCS max30105.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common esp_event driver log
)
