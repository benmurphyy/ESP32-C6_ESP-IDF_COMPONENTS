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
 * @file wx_utils.c
 *
 * ESP-IDF weather utilities (utils)
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "wx_utils.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * macro definitions
*/

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static declarations
*/


/*
* static constant declarations
*/

static const char *TAG = "wx_utils";


/*
* functions and subroutines
*/

esp_err_t wx_set_temperature_range(const float maximum, const float minimum) {
    /* validate max and min arguments */
    if(minimum > maximum || maximum < minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "minimum temperature must be smaller than the maximum temperature, set temperature range failed");
    }

    /* set max and min range */
    wx_temperature_range = (wx_scalar_range_t){ .maximum = maximum, .minimum = minimum };

    return ESP_OK;
}

esp_err_t wx_set_humidity_range(const float maximum, const float minimum) {
    /* validate max and min arguments */
    if(minimum > maximum || maximum < minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "minimum humidity must be smaller than the maximum humidity, set humidity range failed");
    }

    /* set max and min ranges */
    wx_humidity_range = (wx_scalar_range_t){ .maximum = maximum, .minimum = minimum };

    return ESP_OK;
}

esp_err_t wx_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK(dewpoint);

    // validate range of temperature parameter
    if(temperature > wx_temperature_range.maximum || temperature < wx_temperature_range.minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dewpoint failed");
    }

    // validate range of humidity parameter
    if(humidity > wx_humidity_range.maximum || humidity < wx_humidity_range.minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dewpoint failed");
    }

    // calculate dew-point temperature
    double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}