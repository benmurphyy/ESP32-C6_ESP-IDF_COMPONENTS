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
 * @file i2c_master_ext.c
 *
 * ESP-IDF driver extension for i2c peripheral drivers
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "i2c_master_ext.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define UINT8_TO_BINARY_BUFFER_SIZE     (9)     // 8 bits + 1 for null terminator
#define UINT16_TO_BINARY_BUFFER_SIZE    (17)    // 16 bits + 1 for null ter
#define UINT32_TO_BINARY_BUFFER_SIZE    (33)    // 32 bits + 1 for null ter

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


static char uint8_to_binary_buffer[UINT8_TO_BINARY_BUFFER_SIZE]; 
static char uint16_to_binary_buffer[UINT16_TO_BINARY_BUFFER_SIZE]; 
static char uint32_to_binary_buffer[UINT32_TO_BINARY_BUFFER_SIZE]; 


/*
* static constant declarations
*/
static const char *TAG = "i2c_master_ext";

/*
* functions and subroutines
*/

uint32_t get_chip_id(void) {
    uint32_t chipid = 0L;
    for (int i = 0; i < 17; i = i + 8) {
        chipid |= ((get_efuse_mac() >> (40 - i)) & 0xff) << i;
    }
    return chipid;
}

uint64_t get_efuse_mac(void) {
    uint64_t chipmacid = 0LL;
    esp_efuse_mac_get_default((uint8_t *)(&chipmacid));
    return chipmacid;
}

const char *uint8_to_binary(uint8_t n) {
    uint8_to_binary_buffer[8] = '\0';

    for (int i = 7; i >= 0; --i) {
        uint8_to_binary_buffer[i] = '0' + (n & 1); // '0' or '1'
        n >>= 1; // shift to the next bit
    }

    return uint8_to_binary_buffer;
}

const char *uint16_to_binary(uint16_t n) {
    uint16_to_binary_buffer[16] = '\0';

    for (int i = 15; i >= 0; --i) {
        uint16_to_binary_buffer[i] = '0' + (n & 1); // '0' or '1'
        n >>= 1; // shift to the next bit
    }

    return uint16_to_binary_buffer;
}

const char *uint32_to_binary(uint32_t n) {
    uint32_to_binary_buffer[32] = '\0';

    for (int i = 31; i >= 0; --i) {
        uint32_to_binary_buffer[i] = '0' + (n & 1); // '0' or '1'
        n >>= 1; // shift to the next bit
    }

    return uint32_to_binary_buffer;
}


uint16_t bytes_to_uint16(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    } else {
        return ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
    }
}

uint32_t bytes_to_uint32(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    } else {
        return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];
    }
}

uint64_t bytes_to_uint64(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (uint64_t)bytes[0] | ((uint64_t)bytes[1] << 8) | ((uint64_t)bytes[2] << 16) | ((uint64_t)bytes[3] << 24) | ((uint64_t)bytes[4] << 32) | ((uint64_t)bytes[5] << 40) | ((uint64_t)bytes[6] << 48) | ((uint64_t)bytes[7] << 56);
    } else {
        return ((uint64_t)bytes[0] << 56) | ((uint64_t)bytes[1] << 48) | ((uint64_t)bytes[2] << 40) | ((uint64_t)bytes[3] << 32) | ((uint64_t)bytes[4] << 24) | ((uint64_t)bytes[5] << 16) | ((uint64_t)bytes[6] << 8) | (uint64_t)bytes[7];
    }
}

int16_t bytes_to_int16(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (int16_t)bytes[0] | ((int16_t)bytes[1] << 8);
    } else {
        return ((int16_t)bytes[0] << 8) | (int16_t)bytes[1];
    }
}

int32_t bytes_to_int32(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (int32_t)bytes[0] | ((int32_t)bytes[1] << 8) | ((int32_t)bytes[2] << 16) | ((int32_t)bytes[3] << 24);
    } else {
        return ((int32_t)bytes[0] << 24) | ((int32_t)bytes[1] << 16) | ((int32_t)bytes[2] << 8) | (int32_t)bytes[3];
    }
}

int64_t bytes_to_int64(const uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        return (int64_t)bytes[0] | ((int64_t)bytes[1] << 8) | ((int64_t)bytes[2] << 16) | ((int64_t)bytes[3] << 24) | ((int64_t)bytes[4] << 32) | ((int64_t)bytes[5] << 40) | ((int64_t)bytes[6] << 48) | ((int64_t)bytes[7] << 56);
    } else {
        return ((int64_t)bytes[0] << 56) | ((int64_t)bytes[1] << 48) | ((int64_t)bytes[2] << 40) | ((int64_t)bytes[3] << 32) | ((int64_t)bytes[4] << 24) | ((int64_t)bytes[5] << 16) | ((int64_t)bytes[6] << 8) | (int64_t)bytes[7];
    }
}

void uint16_to_bytes(const uint16_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);        // lsb
        bytes[1] = (uint8_t)((value >> 8) & 0xff); // msb
    } else {
        bytes[0] = (uint8_t)((value >> 8) & 0xff); // msb
        bytes[1] = (uint8_t)(value & 0xff);        // lsb
    }
}

void uint32_to_bytes(const uint32_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);    
        bytes[1] = (uint8_t)((value >> 8) & 0xff);
        bytes[2] = (uint8_t)((value >> 16) & 0xff);
        bytes[3] = (uint8_t)((value >> 24) & 0xff);
    } else {
        bytes[0] = (uint8_t)((value >> 24) & 0xff);      
        bytes[1] = (uint8_t)((value >> 16) & 0xff);
        bytes[2] = (uint8_t)((value >> 8) & 0xff); 
        bytes[3] = (uint8_t)(value & 0xff);
    }
}

void uint64_to_bytes(const uint64_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);   
        bytes[1] = (uint8_t)((value >> 8) & 0xff);
        bytes[2] = (uint8_t)((value >> 16) & 0xff);
        bytes[3] = (uint8_t)((value >> 24) & 0xff);
        bytes[4] = (uint8_t)((value >> 32) & 0xff); 
        bytes[5] = (uint8_t)((value >> 40) & 0xff); 
        bytes[6] = (uint8_t)((value >> 48) & 0xff); 
        bytes[7] = (uint8_t)((value >> 56) & 0xff); 
    } else {
        bytes[0] = (uint8_t)((value >> 56) & 0xff);
        bytes[1] = (uint8_t)((value >> 48) & 0xff);
        bytes[2] = (uint8_t)((value >> 40) & 0xff);
        bytes[3] = (uint8_t)((value >> 32) & 0xff);
        bytes[4] = (uint8_t)((value >> 24) & 0xff);      
        bytes[5] = (uint8_t)((value >> 16) & 0xff);
        bytes[6] = (uint8_t)((value >> 8) & 0xff); 
        bytes[7] = (uint8_t)(value & 0xff);
    }
}

void int16_to_bytes(const int16_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);        // lsb
        bytes[1] = (uint8_t)((value >> 8) & 0xff); // msb
    } else {
        bytes[0] = (uint8_t)((value >> 8) & 0xff); // msb
        bytes[1] = (uint8_t)(value & 0xff);        // lsb
    }
}

void int32_to_bytes(const int32_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);    
        bytes[1] = (uint8_t)((value >> 8) & 0xff);
        bytes[2] = (uint8_t)((value >> 16) & 0xff);
        bytes[3] = (uint8_t)((value >> 24) & 0xff);
    } else {
        bytes[0] = (uint8_t)((value >> 24) & 0xff);      
        bytes[1] = (uint8_t)((value >> 16) & 0xff);
        bytes[2] = (uint8_t)((value >> 8) & 0xff); 
        bytes[3] = (uint8_t)(value & 0xff);
    }
}

void int64_to_bytes(const int64_t value, uint8_t* bytes, bool little_endian) {
    if(little_endian == true) {
        bytes[0] = (uint8_t)(value & 0xff);   
        bytes[1] = (uint8_t)((value >> 8) & 0xff);
        bytes[2] = (uint8_t)((value >> 16) & 0xff);
        bytes[3] = (uint8_t)((value >> 24) & 0xff);
        bytes[4] = (uint8_t)((value >> 32) & 0xff); 
        bytes[5] = (uint8_t)((value >> 40) & 0xff); 
        bytes[6] = (uint8_t)((value >> 48) & 0xff); 
        bytes[7] = (uint8_t)((value >> 56) & 0xff); 
    } else {
        bytes[0] = (uint8_t)((value >> 56) & 0xff);
        bytes[1] = (uint8_t)((value >> 48) & 0xff);
        bytes[2] = (uint8_t)((value >> 40) & 0xff);
        bytes[3] = (uint8_t)((value >> 32) & 0xff);
        bytes[4] = (uint8_t)((value >> 24) & 0xff);      
        bytes[5] = (uint8_t)((value >> 16) & 0xff);
        bytes[6] = (uint8_t)((value >> 8) & 0xff); 
        bytes[7] = (uint8_t)(value & 0xff);
    }
}

void copy_bytes(const uint8_t* source, uint8_t* destination, uint16_t size) {
    for(uint16_t i = 0; i < size; i ++) {
        destination[i] = source[i];
    }
}






esp_err_t i2c_master_bus_detect_devices(i2c_master_bus_handle_t handle) {
    const uint16_t probe_timeout_ms = 50; // timeout in milliseconds
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);

        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            
            address = i + j;

            esp_err_t ret = i2c_master_probe(handle, address, probe_timeout_ms);

            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return ESP_OK; 
}

esp_err_t i2c_master_bus_read_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint8_t *const data) {
    const i2c_uint8_t tx = { reg_addr };
    i2c_uint8_t rx = { 0 };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_uint8 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint8 - rx[0] %02x", rx[0]);

    *data = rx[0];

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint16_t *const data) {
    const i2c_uint8_t tx  = { reg_addr };
    i2c_uint16_t rx = { 0 };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_uint16 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint16 - rx[0] %02x | rx[1] %02x", rx[0], rx[1]);

    *data = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_byte16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint16_t *const data) {
    const i2c_uint8_t tx = { reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_byte16 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint16 - data[0] %02x | data[1] %02x", *data[0], *data[1]);

    return ESP_OK;   
}

esp_err_t i2c_master_bus_read_byte24(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint24_t *const data) {
    const i2c_uint8_t tx = { reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_byte24 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint24 - data[0] %02x | data[1] %02x | data[2] %02x", *data[0], *data[1], *data[2]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read16_byte24(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint24_t *const data) {
    const i2c_bytes_to_uint16_t tx = { .value = reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx.bytes, I2C_UINT16_SIZE, *data, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read16_byte24 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read16_uint24 - data[0] %02x | data[1] %02x | data[2] %02x", *data[0], *data[1], *data[2]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_uint32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint32_t *const data) {
    const i2c_uint8_t tx  = { reg_addr };
    i2c_uint32_t rx = { 0 };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT32_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_uint32 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint32 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x", rx[0], rx[1], rx[2], rx[3]);

    *data = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 24);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_byte32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint32_t *const data) {
    const i2c_uint8_t tx = { reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT32_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_byte32 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint32 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x", *data[0], *data[1], *data[2], *data[3]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_byte48(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint48_t *const data) {
    const i2c_uint8_t tx = { reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_byte48 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint48 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x", *data[0], *data[1], *data[2], *data[3], *data[4], *data[5]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read16_byte48(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint48_t *const data) {
    const i2c_bytes_to_uint16_t tx = { .value = reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx.bytes, I2C_UINT16_SIZE, *data, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read16_byte48 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read16_uint48 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x", *data[0], *data[1], *data[2], *data[3], *data[4], *data[5]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read_byte64(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint64_t *const data) {
    const i2c_uint8_t tx = { reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx, I2C_UINT8_SIZE, *data, I2C_UINT64_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read_byte48 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read_uint64 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x | rx[6] %02x | rx[7] %02x", *data[0], *data[1], *data[2], *data[3], *data[4], *data[5], *data[6], *data[7]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_read16_byte64(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint64_t *const data) {
    const i2c_bytes_to_uint16_t tx = { .value = reg_addr };

    ESP_ARG_CHECK( handle && data ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle, tx.bytes, I2C_UINT16_SIZE, *data, I2C_UINT64_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_read16_byte48 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_read16_uint64 - rx[0] %02x | rx[1] %02x | rx[2] %02x | rx[3] %02x | rx[4] %02x | rx[5] %02x | rx[6] %02x | rx[7] %02x", *data[0], *data[1], *data[2], *data[3], *data[4], *data[5], *data[6], *data[7]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_write_cmd(i2c_master_dev_handle_t handle, const uint8_t command) {
    const i2c_uint8_t tx = { command };

    ESP_ARG_CHECK( handle ); // ignore `command` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle, tx, I2C_UINT8_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_write_cmd failed" );

    ESP_LOGD(TAG, "i2c_master_bus_write_cmd - tx[0] %02x", tx[0]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_write16_cmd(i2c_master_dev_handle_t handle, const uint16_t command) {
    const i2c_bytes_to_uint16_t tx = { .value = command };

    ESP_ARG_CHECK( handle ); // ignore `command` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle, tx.bytes, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_write16_cmd failed" );

    ESP_LOGD(TAG, "i2c_master_bus_write16_cmd - tx[0] %02x | tx[1] %02x ", tx.bytes[0], tx.bytes[1]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_write_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint8_t data) {
    const i2c_uint16_t tx = { reg_addr, data };

    ESP_ARG_CHECK( handle ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle, tx, I2C_UINT16_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_write_uint8 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_write_uint8 - tx[0] %02x | tx[1] %02x", tx[0], tx[1]);

    return ESP_OK;
}

esp_err_t i2c_master_bus_write_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint16_t data) {
    const i2c_uint24_t tx = { reg_addr, (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xff) }; // register, lsb, msb

    ESP_ARG_CHECK( handle ); // ignore `reg_addr` given a range of 0x00 to 0xff is acceptable

    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle, tx, I2C_UINT24_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_bus_write_uint16 failed" );

    ESP_LOGD(TAG, "i2c_master_bus_write_uint8 - tx[0] %02x | tx[1] %02x | tx[2] %02x", tx[0], tx[1], tx[2]);

    return ESP_OK;
}
