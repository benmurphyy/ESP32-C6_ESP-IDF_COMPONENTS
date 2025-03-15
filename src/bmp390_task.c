/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmp390_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <bmp390_task.h>

static inline void print_registers(bmp390_handle_t handle) {
    /* configuration registers */
    bmp390_power_control_register_t     power_ctrl_reg;
    bmp390_configuration_register_t     config_reg;
    bmp390_oversampling_register_t      oversampling_reg;
    bmp390_output_data_rate_register_t  output_data_rate_reg;
    bmp390_interrupt_control_register_t interrupt_ctrl_reg;

    /* attempt to read configuration register */
    bmp390_get_configuration_register(handle, &config_reg);

    /* attempt to read oversampling register */
    bmp390_get_oversampling_register(handle, &oversampling_reg);

    /* attempt to read to power control register */
    bmp390_get_power_control_register(handle, &power_ctrl_reg);

    /* attempt to read to output data rate register */
    bmp390_get_output_data_rate_register(handle, &output_data_rate_reg);

    /* attempt to read to interrupt control register */
    bmp390_get_interrupt_control_register(handle, &interrupt_ctrl_reg);


    ESP_LOGI(APP_TAG, "Configuration (0x%02x): %s", config_reg.reg,           uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Oversampling  (0x%02x): %s", oversampling_reg.reg,     uint8_to_binary(oversampling_reg.reg));
    ESP_LOGI(APP_TAG, "Data Rate     (0x%02x): %s", output_data_rate_reg.reg, uint8_to_binary(output_data_rate_reg.reg));
    ESP_LOGI(APP_TAG, "Power Control (0x%02x): %s", power_ctrl_reg.reg,       uint8_to_binary(power_ctrl_reg.reg));
    ESP_LOGI(APP_TAG, "Int Control   (0x%02x): %s", interrupt_ctrl_reg.reg,   uint8_to_binary(interrupt_ctrl_reg.reg));

    if(interrupt_ctrl_reg.bits.irq_data_ready_enabled) ESP_LOGE(APP_TAG, "bmp390 irq data ready is enabled");
}

void i2c0_bmp390_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bmp390_config_t dev_cfg         = I2C_BMP390_CONFIG_DEFAULT;
    bmp390_handle_t dev_hdl;
    //
    // init device
    bmp390_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bmp390 handle init failed");
        assert(dev_hdl);
    }

    print_registers(dev_hdl);

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BMP390 - START #########################");
        //
        // handle sensor

        float temperature, pressure;
        esp_err_t result = bmp390_get_measurements(dev_hdl, &temperature, &pressure);
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
    bmp390_delete( dev_hdl );
    vTaskDelete( NULL );
}