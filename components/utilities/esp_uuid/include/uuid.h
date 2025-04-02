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
 * @file uuid.h
 * @defgroup utilities
 * @{
 *
 * Motivated by: https://github.com/RobTillaart/UUID/blob/master/README.md
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __UUID_H__
#define __UUID_H__


#include <stdint.h>
#include <stdbool.h>
#include "uuid_version.h"

#ifdef __cplusplus
extern "C" {
#endif




/**
 * @brief UUID modes enumerator definition.
 */
typedef enum uuid_modes_e {
    UUID_MODE_VARIANT4 = 0,  /*!< Variant-4 UUID */
    UUID_MODE_RANDOM   = 1,  /*!< Random UUID */
} uuid_modes_t;

/**
 * @brief Initialize UUID generator with default seed values (1 and 2).
 */
void uuid_init(void);

/**
 * @brief Seed the UUID generator with a variable number of arguments. 
 * The total number of arguments must be between 1 and 2.  Otherwise, 
 * the generator will be seeded with default values (1 and 2).
 * 
 * @param size Number of arguments to seed the generator.
 * @param ... Variable number of arguments (1 to 2, 2 seedlings is ideal).
 */
void uuid_seed(uint8_t size, ... );

/**
 * @brief Generate a UUID (i.e. d29b226d-04b5-e3ae-cd63-e6ec0d5611ab).
 * 
 * @return const char* Pointer to the UUID string.
 */
const char* uuid_generate(void);

/**
 * @brief Set the UUID mode to either variant-4 or random.
 * 
 * @param mode The UUID mode to set.
 */
void uuid_set_mode(const uuid_modes_t mode);

/**
 * @brief Get the current UUID mode.
 * 
 * @return uuid_modes_t The current UUID mode.
 */
uuid_modes_t uuid_get_mode(void);

/**
 * @brief Converts `uuid` firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* `uuid` firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* uuid_get_fw_version(void);

/**
 * @brief Converts `uuid` firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t `uuid` firmware version number.
 */
int32_t uuid_get_fw_version_number(void);




#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __UUID_H__