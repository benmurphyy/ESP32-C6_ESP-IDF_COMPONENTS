<a href="https://components.espressif.com/components/k0i05/esp_ahtxx">
<img src="https://components.espressif.com/components/k0i05/esp_ahtxx/badge.svg" />
</a>

# Asair AHTXX Series of Sensors
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Asair AHTXX series of sensors (AHT10, AHT20, AHT21, AHT25, AND AHT30).  Information on features and functionality are documented and can be found in the `ahtxx.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_ahtxx

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `ahtxx.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_ahtxx
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ahtxx_version.h
    │   └── ahtxx.h
    └── ahtxx.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings for the AHT10, AHT20, AHT21, AHT25, AND AHT30 sensor types and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <ahtxx.h>

void i2c0_ahtxx_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ahtxx_config_t dev_cfg          = I2C_AHT20_CONFIG_DEFAULT;
    ahtxx_handle_t dev_hdl;
    //
    // init device
    ahtxx_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ahtxx handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AHTXX - START #########################");
        //
        // handle sensor
        float temperature, humidity;
        esp_err_t result = ahtxx_get_measurement(dev_hdl, &temperature, &humidity);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ahtxx device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
            ESP_LOGI(APP_TAG, "relative humidity:   %.2f %s", humidity, '%');
        }
        //
        ESP_LOGI(APP_TAG, "######################## AHTXX - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ahtxx_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)