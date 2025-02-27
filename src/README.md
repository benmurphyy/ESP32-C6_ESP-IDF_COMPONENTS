# Overall Purpose

This code is designed to be a central hub for testing and demonstrating various peripheral components (sensors, displays, etc.) that can interface with the ESP32-S3 microcontroller. It's part of a larger project (based on the directory structure and README.md) aimed at creating a reusable library of ESP-IDF components for common devices.

## Key Sections and Concepts

1. ### Header Comments and License:

- The top portion of the code contains standard header comments, including the MIT license. This is important for open-source projects.
- The ASCII art is just a visual flair.

2. ### File-Level Comments:

- The comment block after the ASCII art provides a high-level description of the file's purpose:
    - It's a repository for ESP-IDF components.
    - It includes basic examples for each component.
    - It's designed for the ESP32-S3 development board.
    - It mentions how to configure I2C and component-specific settings (in app_config.h and `[component-name]_task.h` files).
    - It also gives some important command to run in powershell for `menuconfig` and prune tasks.

3. ### Component Includes:

- ```<stdio.h>, <stdlib.h>, <ctype.h>, <unistd.h>, <string.h>```: Standard C library headers for input/output, memory allocation, character handling, POSIX operating system API, and string manipulation.
- ```<esp_log.h>```: ESP-IDF logging library for debugging.
- ```<freertos/FreeRTOS.h>, <freertos/task.h>```: FreeRTOS headers for task management.
- Component-specific headers:
    - ```<i2c_master_ext.h>```: Header for a custom I2C master driver.
    - ```<nvs_ext.h>```: Header for a custom Non-Volatile Storage (NVS) driver.
    - ```<ahtxx_task.h>, <ak8975_task.h>, ... <veml7700_task.h>```: Headers for the task functions that interact with specific I2C components (sensors, etc.).
    - ```<ds18b20_task.h>```: Header for the task function that interacts with the DS18B20 one-wire sensor.

4. ### Component Enumerations:

- i2c_components_t, owb_components_t, spi_components_t: These enum types define the available I2C, One-Wire (OWB), and SPI components that can be tested/used in the application. They provide a clear and organized way to refer to the different devices.

5. ### Bus Configuration and Handles:

- i2c0_bus_cfg, i2c0_bus_hdl: These variables store the configuration and handle for the I2C master bus 0.
- owb0_rmt_cfg, owb0_bus_cfg, owb0_bus_hdl: Variables for the one-wire master bus 0 configuration and handle.
- spi1_bus_cfg, spi1_dev_hdl: Variables for SPI master bus 1.
- i2c0_component_tasked, owb0_component_tasked, spi1_component_tasked: Boolean flags to track if a component is already running on a particular bus. This is crucial because only one component should communicate on a given bus at a time.

6. ### Task Creation Functions (owb0_task_create, i2c0_task_create):

- These are custom functions for creating FreeRTOS tasks specifically for the OWB and I2C components.
- They take a TaskFunction_t (pointer to the task function) and a task name as input.
- Important Feature: They ensure that only one task per bus is created by checking the *_component_tasked flags.
- They use xTaskCreatePinnedToCore to create tasks and assign them to a specific core (APP_CPU_NUM = core 1 in this case).

7. ### Component Example Start Functions (owb0_component_example_start, i2c0_component_example_start):

- These are the main entry points for launching a specific component's example task.
- They take a *_components_t enum value as input to indicate which component to run.
- They use a switch statement to map the component to its corresponding task creation function (e.g., owb0_ds18b20_task or i2c0_ahtxx_task).
- They rely on the i2c0_task_create and owb0_task_create to manage the creation of the tasks.

8. ### i2c0_device_scan Function

- Scan the I2C bus for connected device.

9. ### app_main Function:

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

### How to Use (Based on Comments and Structure)

1. Choose a Component: Decide which I2C or OWB component you want to test (e.g., AHTXX, DS18B20, BMP280, etc.).
2. Uncomment the Appropriate Line: In the app_main function, uncomment the i2c0_component_example_start or owb0_component_example_start line that corresponds to the component you want to use.
3. Build and Flash: Build the code using PlatformIO (or your preferred build system) and flash it to the ESP32-S3.
4. Monitor Output: Use a serial monitor to view the log output and observe the results of the component's example.

### In Essence:

This code provides a well-structured framework for managing and running example code for a variety of peripheral components on the ESP32-S3. It's designed for:

- Organization: Clear component definitions using enums.
- Modularity: Separating component-specific logic into their own task functions.
- Safety: Preventing multiple components from trying to use the same bus simultaneously.
- Reusability: Creating a set of components that can be easily used in other projects.
- Testing: Making it straightforward to test different components by just uncommenting a line.
- Custom component: allows to add new custom components.

If you have any more questions about this code or any specific parts, just let me know!



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)