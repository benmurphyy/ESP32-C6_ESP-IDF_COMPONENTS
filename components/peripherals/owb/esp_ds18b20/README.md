# Maxim-Integrated DS18B20 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Maxim-Integrated DS18B20 1-wire temperature sensor.  Information on features and functionality are documented and can be found in the `ds18b20.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: 

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `ds18b20.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_ds18b20
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ds18b20_version.h
    │   └── ds18b20.h
    └── ds18b20.c
```
## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <ds18b20.h>


```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)