idf_component_register(
    SRCS datatable.c
    INCLUDE_DIRS include
    REQUIRES json esp_common esp_timer esp_time_into_interval
)
