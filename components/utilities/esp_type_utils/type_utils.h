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
 * @file type_utils.h
 * @defgroup type_utils
 * @{
 *
 * ESP-IDF type utilities
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TYPE_UTILS_H__
#define __TYPE_UTILS_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_mac.h>


#ifdef __cplusplus
extern "C" {
#endif


/* 4-byte conversion to float IEEE754 (little endian) */
typedef union {
    uint8_t bytes[4];
    float   value;
} bytes_to_float_t;

/* 8-byte conversion to double (little endian) */
typedef union {
    uint8_t bytes[8];
    double  value;
} bytes_to_double_t;

/* 8-byte conversion to uint64_t (little endian) */
typedef union {
    uint8_t  bytes[8];
    uint64_t value;
} bytes_to_uint64_t;

/* 8-byte conversion to int64_t (little endian) */
typedef union {
    uint8_t  bytes[8];
    int64_t  value;
} bytes_to_int64_t;

/* 4-byte conversion to uint32_t (little endian) */
typedef union {
    uint8_t  bytes[4];
    uint32_t value;
} bytes_to_uint32_t;

/* 4-byte conversion to int32_t (little endian) */
typedef union {
    uint8_t  bytes[4];
    int32_t  value;
} bytes_to_int32_t;

/* 2-byte conversion to uint16_t (little endian) */
typedef union {
    uint8_t  bytes[2];
    uint16_t value;
} bytes_to_uint16_t;

/* 2-byte conversion to int16_t (little endian) */
typedef union {
    uint8_t  bytes[2];
    int16_t  value;
} bytes_to_int16_t;

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
uint16_t bytes_to_uint16(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts byte array to `uint32_t` data-type.
 * 
 * @param bytes Byte array to convert to `uint32_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return uint32_t Converted byte array as `uint32_t` data-type.
 */
uint32_t bytes_to_uint32(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts byte array to `uint64_t` data-type.
 * 
 * @param bytes Byte array to convert to `uint64_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return uint64_t Converted byte array as `uint64_t` data-type.
 */
uint64_t bytes_to_uint64(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts byte array to `int16_t` data-type.
 * 
 * @param bytes Byte array to convert to `int16_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int16_t Converted byte array as `int16_t` data-type.
 */
int16_t bytes_to_int16(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts byte array to `int32_t` data-type.
 * 
 * @param bytes Byte array to convert to `int32_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int32_t Converted byte array as `int32_t` data-type.
 */
int32_t bytes_to_int32(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts byte array to `int64_t` data-type.
 * 
 * @param bytes Byte array to convert to `int64_t` data-type.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 * @return int64_t Converted byte array as `int64_t` data-type.
 */
int64_t bytes_to_int64(const uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `uint16_t` data-type to a byte array.
 * 
 * @param value `uint16_t` data-type to convert to byte array.
 * @param bytes Converted `uint16_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint16_to_bytes(const uint16_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `uint32_t` data-type to a byte array.
 * 
 * @param value `uint32_t` data-type to convert to byte array.
 * @param bytes Converted `uint32_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint32_to_bytes(const uint32_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `uint64_t` data-type to a byte array.
 * 
 * @param value `uint64_t` data-type to convert to byte array.
 * @param bytes Converted `uint64_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void uint64_to_bytes(const uint64_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `int16_t` data-type to a byte array.
 * 
 * @param value `int16_t` data-type to convert to byte array.
 * @param bytes Converted `int16_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int16_to_bytes(const int16_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `int32_t` data-type to a byte array.
 * 
 * @param value `int32_t` data-type to convert to byte array.
 * @param bytes Converted `int32_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int32_to_bytes(const int32_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `int64_t` data-type to a byte array.
 * 
 * @param value `int64_t` data-type to convert to byte array.
 * @param bytes Converted `int64_t` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void int64_to_bytes(const int64_t value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `float` data-type to a byte array.
 * 
 * @param value `float` data-type to convert to byte array.
 * @param bytes Converted `float` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void float_to_bytes(const float value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Converts `double` data-type to a byte array.
 * 
 * @param value `double` data-type to convert to byte array.
 * @param bytes Converted `double` data-type as byte array.
 * @param little_endian Little endian byte order when true, otherwise, big endian byte order when false.
 */
void double_to_bytes(const double value, uint8_t* bytes, const bool little_endian);

/**
 * @brief Copies bytes from source byte array to destination byte array.
 * 
 * @param source Byte array source to copy from.
 * @param destination Byte array destination to copy to.
 * @param size Size of destination byte array.
 */
void copy_bytes(const uint8_t* source, uint8_t* destination, const size_t size);




#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TYPE_UTILS_H__
