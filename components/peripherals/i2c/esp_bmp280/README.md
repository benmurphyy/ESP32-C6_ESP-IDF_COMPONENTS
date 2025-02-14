# Bosch BMP280 Sensor
This esp-idf driver was developed for the Bosch BMP280 sensor.  Information on features and functionality are documented and can be found in the `bmp280.h` header file.

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.
```
#include <bmp280.h>

void i2c0_bmp280_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_bmp280_config_t dev_cfg         = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t dev_hdl;
    //
    // init device
    i2c_bmp280_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bmp280 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BMP280 - START #########################");
        //
        // handle sensor
        float temperature, pressure;
        esp_err_t result = i2c_bmp280_get_measurements(dev_hdl, &temperature, &pressure);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bmp280 device read failed (%s)", esp_err_to_name(result));
        } else {
            pressure = pressure / 100;
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", pressure);
        }
        //
        ESP_LOGI(APP_TAG, "######################## BMP280 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_bmp280_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)