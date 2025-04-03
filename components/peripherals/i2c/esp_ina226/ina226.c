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
 * @file ina226.c
 *
 * ESP-IDF driver for INA225 current, voltage, and power monitoring sensor
 * 
 * https://github.com/cybergear-robotics/ina226/blob/master/ina226.c
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ina226.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define INA226_REG_CONFIG      (0x00)
#define INA226_REG_SHUNT_V     (0x01)
#define INA226_REG_BUS_V       (0x02)
#define INA226_REG_POWER       (0x03)
#define INA226_REG_CURRENT     (0x04)
#define INA226_REG_CALIBRATION (0x05)
#define INA226_REG_MANU_ID     (0xfe)
#define INA226_REG_DIE_ID      (0xff)

#define INA226_DEF_CONFIG 0x399f
#define INA226_ASUKIAAA_DEFAULT_CONFIG 0x4127

#define INA226_POWERUP_DELAY_MS         UINT16_C(25)    //!< ina226 I2C start-up delay before device accepts transactions
#define INA226_APPSTART_DELAY_MS        UINT16_C(25)            
#define INA226_RESET_DELAY_MS           UINT16_C(250)   //!< ina226 I2C software reset delay before device accepts transactions
#define INA226_DATA_READY_DELAY_MS      UINT16_C(10)
#define INA226_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define INA226_CMD_DELAY_MS             UINT16_C(10)
#define INA226_TX_RX_DELAY_MS           UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "ina226";

/*
* functions and subroutines
*/


/**
 * @brief INA226 I2C read halfword from register address transaction.
 * 
 * @param handle INA226 device handle.
 * @param reg_addr INA226 register address to read from.
 * @param word INA226 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina226_i2c_read_word_from(ina226_handle_t handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina226_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief INA226 I2C write halfword to register address transaction.
 * 
 * @param handle INA226 device handle.
 * @param reg_addr INA226 register address to write to.
 * @param word INA226 write transaction input halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina226_i2c_write_word_to(ina226_handle_t handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina226_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

esp_err_t ina226_get_configuration_register(ina226_handle_t handle, ina226_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_set_configuration_register(ina226_handle_t handle, const ina226_config_register_t reg) {
    ina226_config_register_t config = { .reg = reg.reg };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    config.bits.reserved = 0;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_write_word_to(handle, INA226_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_get_calibration_register(ina226_handle_t handle, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CONFIG, reg), TAG, "read calibration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_set_calibration_register(ina226_handle_t handle, const uint16_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_write_word_to(handle, INA226_REG_CALIBRATION, reg), TAG, "write calibration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ina226_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ina226_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ccs811 device handle initialization failed", ina226_config->i2c_address);

    /* validate memory availability for handle */
    ina226_handle_t out_handle;
    out_handle = (ina226_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ini226 device");

    /* copy configuration */
    out_handle->dev_config = *ina226_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to soft-reset */
    ESP_GOTO_ON_ERROR(ina226_reset(out_handle), err_handle, TAG, "unable to soft-reset, init failed");

    /* set device handle */
    *ina226_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    float minimum_lsb = max_current / 32767.0f;
    float current_lsb = (uint16_t)(minimum_lsb * 100000000);
    current_lsb /= 100000000.0f;
    current_lsb /= 0.0001f;
    current_lsb = ceil(current_lsb);
    current_lsb *= 0.0001f;
    handle->current_lsb = current_lsb;
    handle->power_lsb = current_lsb * 25.0f;
    uint16_t calibration_value = (uint16_t)((0.00512f) / (current_lsb * shunt_resistance));

    ESP_RETURN_ON_ERROR(ina226_set_calibration_register(handle, calibration_value), TAG, "unable to write calibration register, calibration failed");

    return ESP_OK;
}

esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_SHUNT_V, &sig), TAG, "read shunt voltage failed" );

    *voltage = (float)sig * 2.5e-6f; /* fixed to 2.5 uV */

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_BUS_V, &sig), TAG, "read bus voltage failed" );

    *voltage = (float)sig * 0.00125f;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_current(ina226_handle_t handle, float *const current) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CURRENT, &sig), TAG, "read bus voltage failed" );

    *current = (float)sig * handle->current_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_power(ina226_handle_t handle, float *const power) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_POWER, &sig), TAG, "read bus voltage failed" );

    *power = (float)sig * handle->power_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_reset(ina226_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ina226_config_register_t config;

    config.bits.reset_enabled = true;

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR(ina226_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_RESET_DELAY_MS));

    /* attempt to configure device */
    ESP_RETURN_ON_ERROR(ina226_get_configuration_register(handle, &config), TAG, "unable to read configuration register, reset failed");

    config.bits.mode                = handle->dev_config.mode;
    config.bits.avg_mode            = handle->dev_config.averaging_mode;
    config.bits.bus_volt_conv_time  = handle->dev_config.bus_voltage_conv_time;
    config.bits.shun_volt_conv_time = handle->dev_config.shunt_voltage_conv_time;

    ESP_RETURN_ON_ERROR(ina226_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    /* attempt to write calibration factor */
    ESP_RETURN_ON_ERROR(ina226_calibrate(handle, handle->dev_config.max_current, handle->dev_config.shunt_resistance), TAG, "unable to calibrate device, reset failed");

    return ESP_OK;
}

