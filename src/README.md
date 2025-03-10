# ESP32-S3 ESP-IDF Components Repository

[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://visualstudio.microsoft.com)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)

## Overall Purpose

This code is designed to be a central hub for testing and demonstrating various peripheral components (sensors, displays, etc.) that can interface with the ESP32-S3 microcontroller. It's part of a larger project (based on the directory structure and README.md) aimed at creating a reusable library of ESP-IDF components for common devices.

## Key Sections and Concepts

### 1. Header Comments and License

- The top portion of the code contains standard header comments, including the MIT license. This is important for open-source projects.
- The ASCII art is just a visual flair.

### 2. File-Level Comments

- The comment block after the ASCII art provides a high-level description of the file's purpose:
- It's a repository for ESP-IDF components.
- It includes basic examples for each component.
- It's designed for the ESP32-S3 development board.
- It mentions how to configure I2C and component-specific settings (in app_config.h and `[component-name]_task.h` files).
- It also gives some important command to run in powershell for `menuconfig` and prune tasks.

### 3. Component Includes

- ```<stdio.h>, <stdlib.h>, <ctype.h>, <unistd.h>, <string.h>```: Standard C library headers for input/output, memory allocation, character handling, POSIX operating system API, and string manipulation.
- ```<esp_log.h>```: ESP-IDF logging library for debugging.
- ```<freertos/FreeRTOS.h>, <freertos/task.h>```: FreeRTOS headers for task management.
- Component-specific headers:
- ```<i2c_master_ext.h>```: Header for a custom I2C master driver.
- ```<nvs_ext.h>```: Header for a custom Non-Volatile Storage (NVS) driver.
- ```<ahtxx_task.h>, <ak8975_task.h>, ... <veml7700_task.h>```: Headers for the task functions that interact with specific I2C components (sensors, etc.).
- ```<ds18b20_task.h>```: Header for the task function that interacts with the DS18B20 one-wire sensor.

### 4. Component Enumerations

- i2c_components_t, owb_components_t, spi_components_t: These enum types define the available I2C, One-Wire (OWB), and SPI components that can be tested/used in the application. They provide a clear and organized way to refer to the different devices.

### 5. Bus Configuration and Handles

- i2c0_bus_cfg, i2c0_bus_hdl: These variables store the configuration and handle for the I2C master bus 0.
- owb0_rmt_cfg, owb0_bus_cfg, owb0_bus_hdl: Variables for the one-wire master bus 0 configuration and handle.
- spi1_bus_cfg, spi1_dev_hdl: Variables for SPI master bus 1.
- i2c0_component_tasked, owb0_component_tasked, spi1_component_tasked: Boolean flags to track if a component is already running on a particular bus. This is crucial because only one component should communicate on a given bus at a time.

### 6. Task Creation Functions (owb0_task_create, i2c0_task_create)

- These are custom functions for creating FreeRTOS tasks specifically for the OWB and I2C components.
- They take a TaskFunction_t (pointer to the task function) and a task name as input.
- Important Feature: They ensure that only one task per bus is created by checking the *_component_tasked flags.
- They use xTaskCreatePinnedToCore to create tasks and assign them to a specific core (APP_CPU_NUM = core 1 in this case).

### 7. Component Example Start Functions (owb0_component_example_start, i2c0_component_example_start)

- These are the main entry points for launching a specific component's example task.
- They take a *_components_t enum value as input to indicate which component to run.
- They use a switch statement to map the component to its corresponding task creation function (e.g., owb0_ds18b20_task or i2c0_ahtxx_task).
- They rely on the i2c0_task_create and owb0_task_create to manage the creation of the tasks.

### 8. i2c0_device_scan Function

- Scan the I2C bus for connected device.

### 9. app_main Function

- This is the main entry point of the ESP32 application, similar to main() in standard C.
- Initialization:
- Prints startup messages, free memory, and IDF version.
- Sets ESP-IDF log levels.
- Initializes NVS (Non-Volatile Storage).
- Initializes the one-wire master bus 0 (onewire_new_bus_rmt).
- Initializes the I2C master bus 0 (i2c_new_master_bus).
- Component Execution (commented code):
- The code to start a component example is commented out, meaning it's not being executed.
- It shows how you would choose which example to run using i2c0_component_example_start or owb0_component_example_start and the corresponding enum values.
- Only the example of the BMP280 is uncommented.
- vTaskDelay: A small delay before starting the component example, probably to allow the system to settle.

## How to Use (Based on Comments and Structure)

1. Choose a Component: Decide which I2C or OWB component you want to test (e.g., AHTXX, DS18B20, BMP280, etc.).
2. Uncomment the Appropriate Line: In the app_main function, uncomment the i2c0_component_example_start or owb0_component_example_start line that corresponds to the component you want to use.
3. Build and Flash: Build the code using PlatformIO (or your preferred build system) and flash it to the ESP32-S3.
4. Monitor Output: Use a serial monitor to view the log output and observe the results of the component's example.

## In Essence

This code provides a well-structured framework for managing and running example code for a variety of peripheral components on the ESP32-S3. It's designed for:

- Organization: Clear component definitions using enums.
- Modularity: Separating component-specific logic into their own task functions.
- Safety: Preventing multiple components from trying to use the same bus simultaneously.
- Reusability: Creating a set of components that can be easily used in other projects.
- Testing: Making it straightforward to test different components by just uncommenting a line.
- Custom component: allows to add new custom components.

If you have any more questions about this code or any specific parts, just let me know!

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
