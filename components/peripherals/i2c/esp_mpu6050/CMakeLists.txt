idf_component_register(
    SRCS mpu6050.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c_ext esp_timer esp_driver_gpio
)
