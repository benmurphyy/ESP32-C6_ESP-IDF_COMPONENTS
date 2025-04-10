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
 * @file at24cxxx.c
 *
 * ESP-IDF driver for AT24CXXX EEPROM types
 * 
 * https://github.com/nopnop2002/esp-idf-24c/tree/master
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/**
 * dependency includes
 */

#include "include/at24cxxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
 
/**
 * constant definitions
 */

#define AT24CXXX_POWERUP_DELAY_MS      UINT16_C(120)
#define AT24CXXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< at24cxxx delay after initialization before application start-up */
#define AT24CXXX_READ_DELAY_MS         UINT16_C(5) 
#define AT24CXXX_WRITE_DELAY_MS        UINT16_C(10)

/**
 * macro definitions
 */

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


 
/**
 * static constant declarations
 */

static const char* TAG = "at24cxxx";

/**
 * @brief AT24CXXX memory layout by EEPROM type.
 */
static const at24cxxx_memory_mapping_t at24cxxx_memory_maps[5] = { 
    { AT24CXXX_AT24C32,  32,  32768,  4096,  32,  128, (128 * 32) - 1  },   // AT24C32
    { AT24CXXX_AT24C64,  64,  64536,  8192,  32,  256, (128 * 64) - 1  },   // AT24C64
    { AT24CXXX_AT24C128, 128, 131072, 16384, 64,  256, (128 * 128) - 1 },   // AT24C128
    { AT24CXXX_AT24C256, 256, 262144, 32768, 64,  512, (128 * 256) - 1 },   // AT24C256
    { AT24CXXX_AT24C512, 512, 524288, 65536, 128, 512, (128 * 512) - 1 },   // AT24C512
};

/**
 * static function and subroutine declarations
 */

/**
 * function and subroutine declarations
 */


/**
 * @brief AT24CXXX I2C read halfword from register address transaction.
 * 
 * @param handle AT24CXXX device handle.
 * @param reg_addr AT24CXXX word register address to read from.
 * @param byte AT24CXXX read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t at24cxxx_i2c_read_byte_from(at24cxxx_handle_t handle, const uint16_t reg_addr, uint8_t *const byte) {
    const bit16_uint8_buffer_t tx = { (uint8_t)((reg_addr >> 8) & 0xff), (uint8_t)(reg_addr & 0xff) }; // msb, lsb (register)
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_read_word_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief AT24CXXX I2C write byte to word register address transaction.
 * 
 * @param handle AT24CXXX device handle.
 * @param reg_addr AT24CXXX register address to write to as a word.
 * @param byte AT24CXXX write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t at24cxxx_i2c_write_byte_to(at24cxxx_handle_t handle, const uint16_t reg_addr, const uint8_t byte) {
    const bit24_uint8_buffer_t tx = { (uint8_t)((reg_addr >> 8) & 0xff), (uint8_t)(reg_addr & 0xff), byte }; // msb, lsb (register), data

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

esp_err_t at24cxxx_init(const i2c_master_bus_handle_t master_handle, const at24cxxx_config_t *at24cxxx_config, at24cxxx_handle_t *const at24cxxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && (at24cxxx_config || at24cxxx_handle) );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, at24cxxx_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, at24cxxx device handle initialization failed", at24cxxx_config->i2c_address);

    /* validate memory availability for handle */
    at24cxxx_handle_t out_handle;
    out_handle = (at24cxxx_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c at24cxxx device, init failed");

    /* copy configuration */
    out_handle->dev_config = *at24cxxx_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* configure EEPROM settings */
    out_handle->memory_map = at24cxxx_memory_maps[out_handle->dev_config.eeprom_type];

    /* set device handle */
    *at24cxxx_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t at24cxxx_read_random_byte(at24cxxx_handle_t handle, const uint16_t data_addr, uint8_t *const data) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_FALSE((data_addr <= handle->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range", data_addr);

    /* attempt read i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_read_byte_from(handle, data_addr, data), TAG, "i2c read from word address 0x%04x failed", data_addr );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_READ_DELAY_MS));

    return ESP_OK;
}

esp_err_t at24cxxx_write_byte(at24cxxx_handle_t handle, const uint16_t data_addr, const uint8_t data) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_FALSE((data_addr <= handle->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range", data_addr);

    /* attempt write i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_write_byte_to(handle, data_addr, data), TAG, "i2c write to word address 0x%04x failed", data_addr );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_WRITE_DELAY_MS));

    return ESP_OK;
}

esp_err_t at24cxxx_erase(at24cxxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    return ESP_OK;
}

esp_err_t at24cxxx_remove(at24cxxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t at24cxxx_delete(at24cxxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( at24cxxx_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle->i2c_handle) {
        free(handle->i2c_handle);
        free(handle);
    }

    return ESP_OK;
}

const char* at24cxxx_get_fw_version(void) {
    return AT24CXXX_FW_VERSION_STR;
}

int32_t at24cxxx_get_fw_version_number(void) {
    return AT24CXXX_FW_VERSION_INT32;
}