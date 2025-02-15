# Roithner LaserTechnik GUVA-S12SD Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Roithner LaserTechnik GUVA-S12SD ultraviolet analog sensor.  Information on features and functionality are documented and can be found in the `s12sd.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/b34d78b383ce366b4954c946b7181bafb1c9ebb3/components/peripherals/adc/esp_s12sd

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `s12sd.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_s12sd
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── s12sd_version.h
    │   └── s12sd.h
    └── s12sd.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <s12sd.h>

void adc0_s12sd_task( void *pvParameters ) {
    /* work in progress */
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)