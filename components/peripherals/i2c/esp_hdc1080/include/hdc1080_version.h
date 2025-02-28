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
 * @file hdc1080_version.h
 * @defgroup drivers hdc1080
 * @{
 *
 * ESP-IDF driver for hdc1080 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __HDC1080_VERSION_H__
#define __HDC1080_VERSION_H__


#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

/** Major version number (X.x.x) */
#define HDC1080_FW_VERSION_MAJOR 1
/** Minor version number (x.X.x) */
#define HDC1080_FW_VERSION_MINOR 1
/** Patch version number (x.x.X) */
#define HDC1080_FW_VERSION_PATCH 0


/**
 * public macro definitions
 */

/** 
 * Macro to print x parameter as a string i.e. enclose x in double quotes. 
 */
#define HDC1080_STR_QUOTES( x ) #x

/** 
 * Macro to create a string of x parameter with all macros fully expanded. 
 */                 
#define HDC1080_STR( x ) HDC1080_STR_QUOTES( x )

/** 
 * Macro to generate current firmware version numbers (major, minor, patch) into a string that is formatted as X.X.X (e.g. 4.0.0). 
 */
#define HDC1080_FW_VERSION_STR                        \
        HDC1080_STR( HDC1080_FW_VERSION_MAJOR ) "." \
        HDC1080_STR( HDC1080_FW_VERSION_MINOR ) "." \
        HDC1080_STR( HDC1080_FW_VERSION_PATCH )

/** 
 * Macro to convert firmware version parameters (major, minor, patch numbers) into an integer (`int32_t`) 
 * value that can be used for comparison purposes.
 * 
 * As an example, [COMPONENT]_FW_VERSION_INT32 >= [COMPONENT]_FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define HDC1080_FW_VERSION_PARAMS_INT32( major, minor, patch )        \
        ((major << 16) | (minor << 8) | (patch))

/**
 * Macro to generate current firmware version numbers (major, minor, patch) as an integer (`int32_t`) value that can 
 * be used for comparison purposes.
 * 
 * As an example, [COMPONENT]_FW_VERSION_INT32 >= [COMPONENT]_FW_VERSION_PARAMS_INT32(4, 0, 0).
 */
#define HDC1080_FW_VERSION_INT32                                      \
        HDC1080_FW_VERSION_PARAMS_INT32(HDC1080_FW_VERSION_MAJOR,   \
                                          HDC1080_FW_VERSION_MINOR,   \
                                          HDC1080_FW_VERSION_PATCH)



/**
 * @brief Converts HDC1080 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* HDC1080 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* hdc1080_get_fw_version(void);

/**
 * @brief Converts HDC1080 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t HDC1080 firmware version number.
 */
int32_t hdc1080_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __HDC1080_VERSION_H__
