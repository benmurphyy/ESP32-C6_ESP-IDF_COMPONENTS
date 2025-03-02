# Rohm BH1750 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Rohm BH1750 sensor.  Information on features and functionality are documented and can be found in the `bh1750.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/b078d5bec745a7953d74438723c52c1fdf7b2cea/components/peripherals/i2c/esp_bh1750

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `bh1750.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_bh1750
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── bh1750_version.h
    │   └── bh1750.h
    └── bh1750.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings for the AHT10, AHT20, AHT21, AHT25, AND AHT30 sensor types and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <bh1750.h>

void i2c0_bh1750_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bh1750_config_t dev_cfg         = I2C_BH1750_CONFIG_DEFAULT;
    bh1750_handle_t dev_hdl;
    //
    // init device
    bh1750_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bh1750 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BH1750 - START #########################");
        //
        // handle sensor
        float ambient_light;
        esp_err_t result = bh1750_get_ambient_light(dev_hdl, &ambient_light);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bh1750 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f lux", ambient_light);

        }
        //
        ESP_LOGI(APP_TAG, "######################## BH1750 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    bh1750_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)