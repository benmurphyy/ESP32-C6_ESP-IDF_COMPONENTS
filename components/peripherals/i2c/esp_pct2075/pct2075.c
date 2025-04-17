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
// https://github.com/RobTillaart/INA228/blob/master/INA228.cpp
/**
 * @file pct2075.c
 *
 * ESP-IDF driver for PCT2075 temperature sensor
 * 
 * 
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/pct2075.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PCT2075_REG_CONFIG              UINT8_C(0x01)   // POR State: 0x00
#define PCT2075_REG_TEMP                UINT8_C(0x00)   // POR State: 0x0000
#define PCT2075_REG_OVER_TEMP_SHTDWN    UINT8_C(0x03)   // POR State: 0x5000 / 0x6E00 / 0xFB00
#define PCT2075_REG_TEMP_HYST           UINT8_C(0x02)   // POR State: 0x4B00 / 0x6900 / 0F600
#define PCT2075_REG_TEMP_IDLE           UINT8_C(0x04)   // POR State: 0x00


#define PCT2075_POWERUP_DELAY_MS         UINT16_C(25)
#define PCT2075_APPSTART_DELAY_MS        UINT16_C(25)
#define PCT2075_CMD_DELAY_MS             UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "ina226";

/**
 * @brief PCT2075 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle PCT2075 device handle.
 * @param reg_addr PCT2075 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_read_from(pct2075_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C read halfword (two's compliment) from register address transaction.
 * 
 * @param handle PCT2075 device handle.
 * @param reg_addr PCT2075 register address to read from.
 * @param word PCT2075 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_read_word_from(pct2075_handle_t handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_read_word_from failed" );

    /* set output parameter */
    *word = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    ESP_LOGI(TAG, "pct2075_i2c_read_word_from(0x%04x):rx[0]=0x%02x|rx[1]=0x%02x", *word, rx[0], rx[1]);

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C write halfword (two's compliment) to register address transaction.
 * 
 * @param handle PCT2075 device handle.
 * @param reg_addr PCT2075 register address to write to.
 * @param word PCT2075 write transaction input halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_write_word_to(pct2075_handle_t handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)((word >> 8) & 0xff), (uint8_t)(word & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Converts an 11-bit value, using two's compliment, to a signed 16-bit integer value.
 * 
 * @param buffer 
 * @return int16_t 
 */
static inline int16_t pct2075_11bit_to_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    uint16_t sig = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    // shift right by 5, i.e. 11-bits are interest, and sign value
    int16_t sigd = (int16_t)sig >> 5;
    return sigd;
}

/**
 * @brief Converts a 9-bit value, using two's compliment, to a signed 16-bit integer value.
 * 
 * @param buffer 
 * @return int16_t 
 */
static inline int16_t pct2075_9bit_to_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    uint16_t sig = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    // shift right by 7, i.e. 9-bits are interest, and sign value
    int16_t sigd = (int16_t)sig >> 7;
    return sigd;
}

esp_err_t pct2075_get_config_register(pct2075_handle_t handle, pct2075_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_word_from(handle, PCT2075_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t pct2075_set_config_register(pct2075_handle_t handle, const pct2075_config_register_t reg) {
    pct2075_config_register_t config = { .reg = reg.reg };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    config.bits.reserved = 0;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write_word_to(handle, PCT2075_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t pct2075_init(const i2c_master_bus_handle_t master_handle, const pct2075_config_t *pct2075_config, pct2075_handle_t *const pct2075_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && pct2075_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, pct2075_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ptc2075 device handle initialization failed", pct2075_config->i2c_address);

    /* validate memory availability for handle */
    pct2075_handle_t out_handle;
    out_handle = (pct2075_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c pct2075 device");

    /* copy configuration */
    out_handle->dev_config = *pct2075_config;

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

    /* set device handle */
    *pct2075_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t pct2075_get_temperature(pct2075_handle_t handle, float *const temperature){
    bit16_uint8_buffer_t rx;
    //uint16_t sig;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(handle, PCT2075_REG_TEMP, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read temperature register failed" );
    //ESP_RETURN_ON_ERROR( pct2075_i2c_read_word_from(handle, PCT2075_REG_TEMP, &sig), TAG, "read temperature register failed" );

    *temperature = (float)pct2075_11bit_to_int16(rx) * 0.125f;
    //*temperature = (float)((sig >> 5) * 0.125f);


    return ESP_OK;
}

esp_err_t pct2075_remove(pct2075_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t pct2075_delete(pct2075_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( pct2075_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle->i2c_handle) {
        free(handle->i2c_handle);
        free(handle);
    }

    return ESP_OK;
}