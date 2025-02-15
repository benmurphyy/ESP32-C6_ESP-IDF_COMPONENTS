# Texas Instruments HDC1080 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Texas Instruments HDC1080 sensor.  Information on features and functionality are documented and can be found in the `hdc1080.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/2f662825bf50c863ff09dd461361724ba9e470f2/components/peripherals/i2c/esp_hdc1080

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `hdc1080.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_hdc1080
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── hdc1080.h
    └── hdc1080.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <hdc1080.h>

void i2c0_hdc1080_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_hdc1080_config_t dev_cfg        = I2C_HDC1080_CONFIG_DEFAULT;
    i2c_hdc1080_handle_t dev_hdl;

    // init device
    i2c_hdc1080_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "hdc1080 handle init failed");
        assert(dev_hdl);
    }

    ESP_LOGI(APP_TAG, "Device ID:       0x%04x", dev_hdl->device_id);
    ESP_LOGI(APP_TAG, "Manufacturer ID: 0x%04x", dev_hdl->manufacturer_id);

    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## HDC1080 - START #########################");
        //
        // handle sensor
        
        float temperature; float humidity; float dewpoint;
        esp_err_t result = i2c_hdc1080_get_measurements(dev_hdl, &temperature, &humidity, &dewpoint);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "hdc1080 get measurement failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "Temperature: %f °C", temperature);
            ESP_LOGI(APP_TAG, "Dewpoint:    %f °C", dewpoint);
            ESP_LOGI(APP_TAG, "Humidity:    %f %c", humidity, '%');
        }
        
        //
        ESP_LOGI(APP_TAG, "######################## HDC1080 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_hdc1080_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)