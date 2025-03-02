<a href="https://components.espressif.com/components/k0i05/esp_sht4x">
<img src="https://components.espressif.com/components/k0i05/esp_sht4x/badge.svg" />
</a>

# Sensirion SHT4X Series of Sensors
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Sensirion SHT4X series of sensors (SHT40, SHT41, SHT43, and SHT45).  Information on features and functionality are documented and can be found in the `sht4x.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_sht4x

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `sht4x.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_sht4x
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── sht4x_version.h
    │   └── sht4x.h
    └── sht4x.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <sht4x.h>

void i2c0_sht4x_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    sht4x_config_t dev_cfg          = I2C_SHT4X_CONFIG_DEFAULT;
    sht4x_handle_t dev_hdl;
    //
    // init device
    sht4x_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sht4x handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SHT4X - START #########################");
        //
        // handle sensor
        float temperature, humidity;
        esp_err_t result = sht4x_get_measurement(dev_hdl, &temperature, &humidity);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "sht4x device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
            ESP_LOGI(APP_TAG, "relative humidity:   %.2f %s", humidity, '%');
        }
        //
        ESP_LOGI(APP_TAG, "######################## SHT4X - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    sht4x_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)