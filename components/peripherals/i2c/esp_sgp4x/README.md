<a href="https://components.espressif.com/components/k0i05/esp_sgp4x">
<img src="https://components.espressif.com/components/k0i05/esp_sgp4x/badge.svg" />
</a>

# Sensirion SGP4X Series of Sensors
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Sensirion SGP4X series of sensors (SGP40 and SGP41).  Information on features and functionality are documented and can be found in the `sgp4x.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_sgp4x

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `sgp4x.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

The SGP40 is not fully implemented, support for the SGP40 will be revisited once a sensor is available to test with.
```
components
└── esp_sgp4x
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── sgp4x_version.h
    │   └── sgp4x.h
    └── sgp4x.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <sgp4x.h>
#include <sensirion_gas_index_algorithm.h>

void i2c0_sgp4x_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    sgp4x_config_t dev_cfg          = I2C_SGP41_CONFIG_DEFAULT;
    sgp4x_handle_t dev_hdl;
    bool               dev_self_tested  = false;
    bool               dev_conditioned  = false;
    //
    // initialize gas index parameters
    GasIndexAlgorithmParams voc_params;
    GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithmParams nox_params;
    GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    //
    // init device
    sgp4x_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sgp4x handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SGP4X - START #########################");
        //
        // handle sensor
        if(dev_self_tested == false) {
            sgp4x_self_test_result_t self_test_result;
            esp_err_t result = sgp4x_execute_self_test(dev_hdl, &self_test_result);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGI(APP_TAG, "VOC Pixel:   %d", self_test_result.pixels.voc_pixel_failed);
                ESP_LOGI(APP_TAG, "NOX Pixel:   %d", self_test_result.pixels.nox_pixel_failed);
            }
            dev_self_tested = true;
        }
        // 
        if(dev_conditioned == false) {
            for(int i = 0; i < 10; i++) {
                uint16_t sraw_voc; 
                esp_err_t result = sgp4x_execute_conditioning(dev_hdl, &sraw_voc);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
                } else {
                    ESP_LOGI(APP_TAG, "SRAW VOC: %u", sraw_voc);
                }
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second * 10 iterations = 10-seconds
            }
            dev_conditioned = true;
        } else {
            uint16_t sraw_voc; uint16_t sraw_nox;
            int32_t voc_index; int32_t nox_index;
            esp_err_t result = sgp4x_measure_signals(dev_hdl, &sraw_voc, &sraw_nox);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
            } else {
                GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index);
                GasIndexAlgorithm_process(&nox_params, sraw_nox, &nox_index);

                ESP_LOGI(APP_TAG, "SRAW VOC: %u | VOC Index: %li", sraw_voc, voc_index);
                ESP_LOGI(APP_TAG, "SRAW NOX: %u | NOX Index: %li", sraw_nox, nox_index);
            }
        }
        
        //
        ESP_LOGI(APP_TAG, "######################## SGP4X - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    sgp4x_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)