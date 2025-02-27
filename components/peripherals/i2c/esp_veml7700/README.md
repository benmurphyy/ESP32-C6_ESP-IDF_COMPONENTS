# Vishay VEML7700 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Vishay VEML7700 sensor.  Information on features and functionality are documented and can be found in the `veml7700.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/d74f2254d808b55d1db944f0c931d3411bb1e697/components/peripherals/i2c/esp_veml7700

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `veml7700.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

The auto-calibrate algorithms aren't consistent and are still being worked on.

```
components
└── esp_veml7700
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── veml7700_version.h
    │   └── veml7700.h
    └── veml7700.c
```
## Typical Luminance Values
Luminance ranges with use-case examples:

* `10-5 lx`: Light from Sirius, the brightest star in the night sky
* `10-4 lx`: Total starlight, overcast sky
* `0.002 lx`: Moonless clear night sky with airglow
* `0.01 lx`: Quarter moon, 0.27 lx; full moon on a clear night
* `1 lx`: Full moon overhead at tropical latitudes
* `3.4 lx`: Dark limit of civil twilight under a clear sky
* `50 lx`: Family living room
* `80 lx`: Hallway / bathroom
* `100 lx`: Very dark overcast day
* `320 lx to 500 lx`: Office lighting
* `400 lx`: Sunrise or sunset on a clear day
* `1,000 lx`: Overcast day; typical TV studio lighting
* `10,000 lx to 25,000 lx`: Full daylight (not direct sun)
* `32,000 lx to 130,000 lx`: Direct sunlight

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <veml7700.h>

void i2c0_veml7700_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_veml7700_config_t dev_cfg       = I2C_VEML7700_CONFIG_DEFAULT;
    i2c_veml7700_handle_t dev_hdl;
    //
    // init device
    i2c_veml7700_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "veml7700 handle init failed");
        assert(dev_hdl);
    }
    //
    // optimize sensor
    //i2c_veml7700_optimize_configuration(dev_hdl);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## VEML7700 - START #########################");
        //
        // handle sensor
        float ambient_light;
        //uint16_t als_counts;
        esp_err_t result = i2c_veml7700_get_ambient_light(dev_hdl, &ambient_light);
        //esp_err_t result = i2c_veml7700_get_ambient_light_counts(dev_hdl, &als_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "veml7700 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f lux", ambient_light);
            //ESP_LOGI(APP_TAG, "ambient light:     %u counts", als_counts);
        }
        //
        ESP_LOGI(APP_TAG, "######################## VEML7700 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_veml7700_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)