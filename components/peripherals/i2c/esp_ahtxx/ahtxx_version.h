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
 * @file ahtxx_version.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for ahtxx sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AHTXX_VERSION_H__
#define __AHTXX_VERSION_H__


#ifdef __cplusplus
extern "C"
{
#endif

/*
 * public macro definitions
*/

#define I2C_AHTXX_STR_QUOTES( x ) #x                    /* Print macro argument as a string i.e. enclose x in double quotes */
#define I2C_AHTXX_STR( x ) I2C_AHTXX_STR_QUOTES( x )    /* Create a string of x with all macros fully expanded */


/*
 * public constant definitions
*/

#define I2C_AHTXX_FW_VERSION_MAJOR 0
#define I2C_AHTXX_FW_VERSION_MINOR 3
#define I2C_AHTXX_FW_VERSION_PATCH 3

/* Firmware version string in format x.x.x */
#define I2C_AHTXX_FW_VERSION_STR                        \
        I2C_AHTXX_STR( I2C_AHTXX_FW_VERSION_MAJOR ) "." \
        I2C_AHTXX_STR( I2C_AHTXX_FW_VERSION_MINOR ) "." \
        I2C_AHTXX_STR( I2C_AHTXX_FW_VERSION_PATCH )



#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AHTXX_VERSION_H__
