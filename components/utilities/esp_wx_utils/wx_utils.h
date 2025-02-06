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
 * @file wx_utils.h
 * @defgroup wx_utils
 * @{
 *
 * ESP-IDF weather utilities
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __WX_UTILS_H__
#define __WX_UTILS_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>


#ifdef __cplusplus
extern "C" {
#endif

/*
 * weather utilities definitions
*/

#define WX_TEMPERATURE_MAX       (float)(125.0)  //!< maximum temperature range
#define WX_TEMPERATURE_MIN       (float)(-40.0)  //!< minimum temperature range
#define WX_HUMIDITY_MAX          (float)(100.0)  //!< maximum humidity range
#define WX_HUMIDITY_MIN          (float)(0.0)    //!< minimum humidity range

/*
 * weather utilities enumerator and structure declarations
*/

typedef struct {
    float maximum;
    float minimum;
} wx_scalar_range_t;



/*
* weather utilities static global declarations
*/

/**
 * @brief Weather utilities temperature maximum and minimum ranges in degrees Celsius (default maximum 125 and minimum -40).
 */
static wx_scalar_range_t wx_temperature_range = { .maximum = WX_TEMPERATURE_MAX, .minimum = WX_TEMPERATURE_MIN };

/**
 * @brief Weather utilities humidity maximum and minimum ranges in percent (default maximum 100 and minimum 0).
 */
static wx_scalar_range_t wx_humidity_range = { .maximum = WX_HUMIDITY_MAX, .minimum = WX_HUMIDITY_MIN };


/*
* function and subroutine declarations
*/

/**
 * @brief Sets the global weather utilities maximum and minimum temperature range (`wx_temperature_ranges`) in degrees Celsius.
 * 
 * @param maximum Maximum temperature range in degrees Celsius.
 * @param minimum Minimum temperature range in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_set_temperature_range(const float maximum, const float minimum);

/**
 * @brief Sets the global weather utilities humidity maximum and minimum range (`wx_humidity_ranges`) in percent.
 * 
 * @param maximum Maximum humidity range in percent.
 * @param minimum Minimum humidity range in percent.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_set_humidity_range(const float maximum, const float minimum);

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity.
 *
 * @param[in] temperature Air temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @param[out] dewpoint Calculated dewpoint temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint);




#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __WX_UTILS_H__
