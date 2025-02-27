# Bosch BMP390 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Bosch BMP390 sensor.  Information on features and functionality are documented and can be found in the `bmp290.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/e5b7051e796db2b5e93ac426611336bd6fc4c144/components/peripherals/i2c/esp_bmp390

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `bmp390.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_bmp390
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── bmp390_version.h
    │   └── bmp390.h
    └── bmp390.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <bmp390.h>

void i2c0_bmp390_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_bmp390_config_t dev_cfg         = I2C_BMP390_CONFIG_DEFAULT;
    i2c_bmp390_handle_t dev_hdl;
    //
    // init device
    i2c_bmp390_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bmp390 handle init failed");
        assert(dev_hdl);
    }

    /* configuration registers */
    i2c_bmp390_power_control_register_t     power_ctrl_reg;
    i2c_bmp390_configuration_register_t     config_reg;
    i2c_bmp390_oversampling_register_t      oversampling_reg;
    i2c_bmp390_output_data_rate_register_t  output_data_rate_reg;
    i2c_bmp390_interrupt_control_register_t interrupt_ctrl_reg;

    /* attempt to read configuration register */
    i2c_bmp390_get_configuration_register(dev_hdl, &config_reg);

    /* attempt to read oversampling register */
    i2c_bmp390_get_oversampling_register(dev_hdl, &oversampling_reg);

    /* attempt to read to power control register */
    i2c_bmp390_get_power_control_register(dev_hdl, &power_ctrl_reg);

    /* attempt to read to output data rate register */
    i2c_bmp390_get_output_data_rate_register(dev_hdl, &output_data_rate_reg);

    /* attempt to read to interrupt control register */
    i2c_bmp390_get_interrupt_control_register(dev_hdl, &interrupt_ctrl_reg);


    ESP_LOGI(APP_TAG, "Configuration (0x%02x): %s", config_reg.reg,           uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Oversampling  (0x%02x): %s", oversampling_reg.reg,     uint8_to_binary(oversampling_reg.reg));
    ESP_LOGI(APP_TAG, "Data Rate     (0x%02x): %s", output_data_rate_reg.reg, uint8_to_binary(output_data_rate_reg.reg));
    ESP_LOGI(APP_TAG, "Power Control (0x%02x): %s", power_ctrl_reg.reg,       uint8_to_binary(power_ctrl_reg.reg));
    ESP_LOGI(APP_TAG, "Int Control   (0x%02x): %s", interrupt_ctrl_reg.reg,   uint8_to_binary(interrupt_ctrl_reg.reg));

    if(interrupt_ctrl_reg.bits.irq_data_ready_enabled) ESP_LOGE(APP_TAG, "bmp390 irq data ready is enabled");

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BMP390 - START #########################");
        //
        // handle sensor
        i2c_bmp390_set_power_mode(dev_hdl, I2C_BMP390_POWER_MODE_FORCED);

        float temperature, pressure;
        esp_err_t result = i2c_bmp390_get_measurements(dev_hdl, &temperature, &pressure);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bmp390 device read failed (%s)", esp_err_to_name(result));
        } else {
            pressure = pressure / 100;
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", pressure);
        }
        //
        ESP_LOGI(APP_TAG, "######################## BMP390 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_bmp390_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)