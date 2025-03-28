# ESP-IDF UUID Generator

[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://visualstudio.microsoft.com)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_uuid.svg)](https://registry.platformio.org/libraries/k0i05/esp_uuid)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_uuid/badge.svg)](https://components.espressif.com/components/k0i05/esp_uuid)

This ESP32 espressif IoT development framework (esp-idf) RFC-4122 UUID generator component.  Information on features and functionality are documented and can be found in the `uuid.h` header file.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/utilities/esp_uuid>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `uuid.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_uuid
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── uuid_version.h
    │   └── uuid.h
    └── uuid.c
```

## Basic Example

Once the component is referenced as an include, the functions should be visible and available for usage.  The below example demonstrates variant-4 and random UUID generation with processing time-span results printed to the serial console.

```c
#include <uuid.h>

void utils_uuid_task( void *pvParameters ) {
    TickType_t           last_wake_time   = xTaskGetTickCount ();

    uuid_init();
    
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## UUID - START #########################");
        
        uint64_t start_time = esp_timer_get_time();
        uuid_set_mode(UUID_MODE_VARIANT4);
        const char* uuid_var = uuid_generate();
        uint64_t stop_time = esp_timer_get_time();
        uint32_t time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Variant4 UUID (%lu-us): %s", time_diff, uuid_var);

        start_time = esp_timer_get_time();
        uuid_set_mode(UUID_MODE_RANDOM);
        const char* uuid_ran = uuid_generate();
        stop_time = esp_timer_get_time();
        time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Random UUID   (%lu-us): %s", time_diff, uuid_ran);

        uint32_t seed1 = random();
        uint32_t seed2 = random();

        start_time = esp_timer_get_time();
        uuid_seed(seed1, seed2);
        stop_time = esp_timer_get_time();
        time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Seed Time: %lu-us", time_diff);

        start_time = esp_timer_get_time();
        uuid_set_mode(UUID_MODE_VARIANT4);
        const char* uuid_seed = uuid_generate();
        stop_time = esp_timer_get_time();
        time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Variant4 UUID (%lu-us): %s", time_diff, uuid_seed);

        ESP_LOGI(APP_TAG, "######################## UUID - END ###########################");

        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, UTILS_TASK_SAMPLING_RATE );
    }
    
    // free resources
    vTaskDelete( NULL );
}
```

The results of the UUID generation printed to the serial console are shown below.

```text
I (31627) ESP-IDF COMPONENTS [APP]: ######################## UUID - START #########################
I (31627) ESP-IDF COMPONENTS [APP]: Variant4 UUID (5-us): 053031da-e5cf-4de1-8429-285b0dba59d5
I (31637) ESP-IDF COMPONENTS [APP]: Random UUID   (5-us): d29b226d-04b5-e3ae-cd63-e6ec0d5611ab
I (31647) ESP-IDF COMPONENTS [APP]: Seed Time: 1-us
I (31647) ESP-IDF COMPONENTS [APP]: Variant4 UUID (5-us): 3d9d78de-071d-4901-bde4-ded767d8deb8
I (31657) ESP-IDF COMPONENTS [APP]: ######################## UUID - END ###########################
```

## References

Information referenced for this component are outlined as follows:

- RFC-4122 specification: <https://datatracker.ietf.org/doc/html/rfc4122>
- C++ UUID generator (original source): <https://github.com/RobTillaart/UUID/blob/master/README.md>

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
