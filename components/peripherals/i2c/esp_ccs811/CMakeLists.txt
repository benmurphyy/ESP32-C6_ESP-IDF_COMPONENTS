idf_component_register(
    SRCS ccs811.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c esp_type_utils esp_driver_gpio esp_event esp_timer
)
