# Lite-On LTR390UV Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Lite-On LTR390UV I2C sensor.  Information on features and functionality are documented and can be found in the `ltr390uv.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_ltr390uv

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `ltr390uv.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_ltr390uv
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ltr390uv_version.h
    │   └── ltr390uv.h
    └── ltr390uv.c
```
## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <ltr390uv.h>

void i2c0_ltr390uv_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ltr390uv_config_t dev_cfg          = I2C_LTR390UV_CONFIG_DEFAULT;
    ltr390uv_handle_t dev_hdl;
    //
    // init device
    ltr390uv_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ltr390uv handle init failed");
        assert(dev_hdl);
    }
    //
    ltr390uv_control_register_t c_reg;
    ltr390uv_interrupt_config_register_t ic_reg;
    ltr390uv_measure_register_t m_reg;
    ltr390uv_gain_register_t    g_reg;
    //
    /* attempt i2c read transaction */
    ltr390uv_get_measure_register(dev_hdl, &m_reg);
    ltr390uv_get_gain_register(dev_hdl, &g_reg);
    ltr390uv_get_interrupt_config_register(dev_hdl, &ic_reg);
    ltr390uv_get_control_register(dev_hdl, &c_reg);
    //
    ESP_LOGI(APP_TAG, "Control Register (0x%02x): %s", c_reg.reg, uint8_to_binary(c_reg.reg));
    ESP_LOGI(APP_TAG, "Measure Register (0x%02x): %s", m_reg.reg, uint8_to_binary(m_reg.reg));
    ESP_LOGI(APP_TAG, "Gain Register    (0x%02x): %s", g_reg.reg, uint8_to_binary(g_reg.reg));
    ESP_LOGI(APP_TAG, "IRQ Cfg Register (0x%02x): %s", ic_reg.reg, uint8_to_binary(ic_reg.reg));
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## LTR390UV - START #########################");
        //
        // handle sensor
        
        float ambient_light; 
        esp_err_t result = ltr390uv_get_ambient_light(dev_hdl, &ambient_light);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f Lux", ambient_light);
        }

        uint32_t sensor_counts;
        result = ltr390uv_get_als(dev_hdl, &sensor_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "als sensor counts: %lu", sensor_counts);
        }

        float uvi;
        result = ltr390uv_get_ultraviolet_index(dev_hdl, &uvi);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ultraviolet index: %f", uvi);
        }

        result = ltr390uv_get_uvs(dev_hdl, &sensor_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "uvs sensor counts: %lu", sensor_counts);
        }
        //
        ESP_LOGI(APP_TAG, "######################## LTR390UV - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ltr390uv_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)