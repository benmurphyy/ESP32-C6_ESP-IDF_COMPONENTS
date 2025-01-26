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
 * @file i2c_master_ext.h
 * @defgroup i2c_master i2c_master_ext
 * @{
 *
 * ESP-IDF driver extension for i2c peripheral drivers
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __I2C_MASTER_EXT_H__
#define __I2C_MASTER_EXT_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <esp_mac.h>



#ifdef __cplusplus
extern "C" {
#endif

#define I2C_XFR_TIMEOUT_MS  (500)          //!< I2C transaction timeout in milliseconds

#define I2C_UINT72_SIZE	    (9)
#define I2C_UINT64_SIZE	    (8)
#define I2C_UINT56_SIZE     (7)
#define I2C_UINT48_SIZE     (6)
#define I2C_UINT40_SIZE     (5)
#define I2C_UINT32_SIZE	    (4)
#define I2C_UINT24_SIZE	    (3)
#define I2C_UINT16_SIZE	    (2)
#define I2C_UINT8_SIZE	    (1)

typedef uint8_t             i2c_uint72_t[I2C_UINT72_SIZE];
typedef uint8_t             i2c_uint64_t[I2C_UINT64_SIZE];
typedef uint8_t             i2c_uint56_t[I2C_UINT56_SIZE];
typedef uint8_t             i2c_uint48_t[I2C_UINT48_SIZE];
typedef uint8_t             i2c_uint40_t[I2C_UINT40_SIZE];
typedef uint8_t             i2c_uint32_t[I2C_UINT32_SIZE];
typedef uint8_t             i2c_uint24_t[I2C_UINT24_SIZE];
typedef uint8_t             i2c_uint16_t[I2C_UINT16_SIZE];
typedef uint8_t             i2c_uint8_t[I2C_UINT8_SIZE];

/* 4-byte conversion to float IEEE754 */
typedef union {
    uint8_t bytes[4];
    float   value;
} i2c_bytes_to_float_t;

/* 4-byte conversion to uint32_t */
typedef union {
    uint8_t  bytes[4];
    uint32_t value;
} i2c_bytes_to_uint32_t;

/* 4-byte conversion to int32_t */
typedef union {
    uint8_t  bytes[4];
    int32_t  value;
} i2c_bytes_to_int32_t;

/* 2-byte conversion to uint16_t */
typedef union {
    uint8_t  bytes[2];
    uint16_t value;
} i2c_bytes_to_uint16_t;

/* 2-byte conversion to int16_t */
typedef union {
    uint8_t  bytes[2];
    int16_t  value;
} i2c_bytes_to_int16_t;

/**
 * @brief Generates a unique chip identifier from e-fuse mac address.
 * 
 * @return uint32_t Chip identifier.
 */
uint32_t get_chip_id(void);

/**
 * @brief Gets the e-fuse mac address.
 * 
 * @return uint64_t Mac address.
 */
uint64_t get_efuse_mac(void);

/**
 * @brief Converts `uint8_t` type to binary as a string.
 * 
 * @param value `uint8_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint8_to_binary(const uint8_t value);

/**
 * @brief Converts `int8_t` type to binary as a string.
 * 
 * @param value `int8_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *int8_to_binary(const int8_t value);

/**
 * @brief Converts `uint16_t` type to binary as a string.
 * 
 * @param value `uint16_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint16_to_binary(const uint16_t value);

/**
 * @brief Converts `int16_t` type to binary as a string.
 * 
 * @param value `int16_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *int16_to_binary(const int16_t value);

/**
 * @brief Converts `uint32_t` type to binary as a string.
 * 
 * @param value `uint32_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint32_to_binary(const uint32_t value);

/**
 * @brief Converts `int32_t` type to binary as a string.
 * 
 * @param value `int32_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *int32_to_binary(const int32_t value);

/**
 * @brief Converts `uint64_t` type to binary as a string.
 * 
 * @param value `uint64_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *uint64_to_binary(const uint64_t value);

/**
 * @brief Converts `int64_t` type to binary as a string.
 * 
 * @param value `int64_t` to transform to binary string.
 * @return char* binary string representation.
 */
const char *int64_to_binary(const int64_t value);

/**
 * @brief Converts byte array to `uint16_t` data-type.
 * 
 * @param bytes Byte array to convert to `uint16_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return uint16_t Converted byte array as `uint16_t` data-type.
 */
uint16_t bytes_to_uint16(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts byte array to `uint32_t` data-type.
 * 
 * @param bytes Byte array to convert to `uint32_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return uint32_t Converted byte array as `uint32_t` data-type.
 */
uint32_t bytes_to_uint32(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts byte array to `uint64_t` data-type.
 * 
 * @param bytes Byte array to convert to `uint64_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return uint64_t Converted byte array as `uint64_t` data-type.
 */
uint64_t bytes_to_uint64(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts byte array to `int16_t` data-type.
 * 
 * @param bytes Byte array to convert to `int16_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int16_t Converted byte array as `int16_t` data-type.
 */
int16_t bytes_to_int16(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts byte array to `int32_t` data-type.
 * 
 * @param bytes Byte array to convert to `int32_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int32_t Converted byte array as `int32_t` data-type.
 */
int32_t bytes_to_int32(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts byte array to `int64_t` data-type.
 * 
 * @param bytes Byte array to convert to `int64_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int64_t Converted byte array as `int64_t` data-type.
 */
int64_t bytes_to_int64(const uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `uint16_t` data-type to a byte array.
 * 
 * @param value `uint16_t` data-type to convert to byte array.
 * @param bytes Converted `uint16_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint16_to_bytes(const uint16_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `uint32_t` data-type to a byte array.
 * 
 * @param value `uint32_t` data-type to convert to byte array.
 * @param bytes Converted `uint32_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint32_to_bytes(const uint32_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `uint64_t` data-type to a byte array.
 * 
 * @param value `uint64_t` data-type to convert to byte array.
 * @param bytes Converted `uint64_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint64_to_bytes(const uint64_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `int16_t` data-type to a byte array.
 * 
 * @param value `int16_t` data-type to convert to byte array.
 * @param bytes Converted `int16_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int16_to_bytes(const int16_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `int32_t` data-type to a byte array.
 * 
 * @param value `int32_t` data-type to convert to byte array.
 * @param bytes Converted `int32_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int32_to_bytes(const int32_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Converts `int64_t` data-type to a byte array.
 * 
 * @param value `int64_t` data-type to convert to byte array.
 * @param bytes Converted `int64_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int64_to_bytes(const int64_t value, uint8_t* bytes, bool little_endian);

/**
 * @brief Copies bytes from source byte array to destination byte array.
 * 
 * @param source Byte array source to copy from.
 * @param destination Byte array destination to copy to.
 * @param size Size of destination byte array.
 */
void copy_bytes(const uint8_t* source, uint8_t* destination, size_t size);

/**
 * @brief I2C master bus device detection that prints the address (1-byte) of detected devices.
 * 
 * @param handle master bus handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_detect_devices(i2c_master_bus_handle_t handle);

/**
 * @brief I2C device `uint8_t` data (1-byte) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint8_t` (1-byte) data read from device
 * @return esp_err_t ESP_OK on success, ESP_FAIL not successful.
 */
esp_err_t i2c_master_bus_read_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint8_t *const data);

/**
 * @brief I2C device `uint16_t` data (2-byte little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint16_t` (2-byte little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint16_t *const data);

/**
 * @brief I2C device `i2c_uint16_t` data (2-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint16_t` (2-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint16_t *const data);

/**
 * @brief I2C device `i2c_uint24_t` data (3-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint24_t` (3-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte24(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint24_t *const data);

/**
 * @brief I2C device `i2c_uint24_t` data (3-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint24_t` (3-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte24(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint24_t *const data);


/**
 * @brief I2C device `uint32_t` data (4-byte little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `uint32_t` (4-byte little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_uint32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, uint32_t *const data);

/**
 * @brief I2C device `i2c_uint32_t` data (4-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint32_t` (4-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte32(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint32_t *const data);

/**
 * @brief I2C device `i2c_uint48_t` data (6-byte array little endian) read from command (1-byte).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint48_t` (6-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte48(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint48_t *const data);

/**
 * @brief I2C device `i2c_uint48_t` data (6-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint48_t` (6-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte48(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint48_t *const data);

/**
 * @brief I2C device `i2c_uint64_t` data (8-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[out] data `i2c_uint64_t` (8-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read_byte64(i2c_master_dev_handle_t handle, const uint8_t reg_addr, i2c_uint64_t *const data);

/**
 * @brief I2C device `i2c_uint64_t` data (8-byte array little endian) read from command (2-byte little endian).  This is a write-read I2C transaction.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (2-byte little endian)
 * @param[out] data `i2c_uint64_t` (8-byte array little endian) data read from device
 * @return esp_err_t ESP_OK: I2C master transmit-receive success - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid. - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_bus_read16_byte64(i2c_master_dev_handle_t handle, const uint16_t reg_addr, i2c_uint64_t *const data);


/**
 * @brief I2C device write command (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] command device command address (1-byte)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_cmd(i2c_master_dev_handle_t handle, const uint8_t command);

/**
 * @brief I2C device write command (2-byte little endian).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] command device command address (2-byte little endian)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write16_cmd(i2c_master_dev_handle_t handle, const uint16_t command);


/**
 * @brief I2C device write register (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[in] data data to write (1-byte)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_uint8(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint8_t data);

/**
 * @brief I2C device write register (1-byte).  This is a write I2C transaction only.
 *
 * @param[in] handle device handle
 * @param[in] reg_addr device register address (1-byte)
 * @param[in] data data to write (2-byte little endian)
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_master_bus_write_uint16(i2c_master_dev_handle_t handle, const uint8_t reg_addr, const uint16_t data);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __I2C_MASTER_EXT_H__
