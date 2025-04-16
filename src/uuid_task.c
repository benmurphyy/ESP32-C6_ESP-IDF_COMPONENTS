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
 * @file uuid_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <math.h>
#include <uuid_task.h>
#include <esp_timer.h>



void utils_uuid_task( void *pvParameters ) {
    TickType_t           last_wake_time   = xTaskGetTickCount ();

    uuid_init();
    
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## UUID - START #########################");
        
        uint64_t start_time;
        uint64_t stop_time;
        uint32_t time_diff;

        for(uint8_t i = 0; i < 5; i++) {
            start_time = esp_timer_get_time();
            uuid_set_mode(UUID_MODE_VARIANT4);
            const char* uuid_var = uuid_generate();
            stop_time = esp_timer_get_time();
            time_diff = stop_time - start_time;
            ESP_LOGI(APP_TAG, "Run(%u) Variant4 UUID (%lu-us): %s", i, time_diff, uuid_var);
        }
        
        for(uint8_t i = 0; i < 5; i++) {
            start_time = esp_timer_get_time();
            uuid_set_mode(UUID_MODE_RANDOM);
            const char* uuid_ran = uuid_generate();
            stop_time = esp_timer_get_time();
            time_diff = stop_time - start_time;
            ESP_LOGI(APP_TAG, "Run(%u) Random UUID   (%lu-us): %s", i, time_diff, uuid_ran);
        }

        uint32_t seed1 = random();
        uint32_t seed2 = random();

        start_time = esp_timer_get_time();
        uuid_seed(seed1, seed2);
        stop_time = esp_timer_get_time();
        time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Seed Time: %lu-us", time_diff);

        start_time = esp_timer_get_time();
        uuid_set_mode(UUID_MODE_VARIANT4);
        const char* uuid_seed = uuid_generate();
        stop_time = esp_timer_get_time();
        time_diff = stop_time - start_time;
        ESP_LOGI(APP_TAG, "Variant4 UUID (%lu-us): %s", time_diff, uuid_seed);

        ESP_LOGI(APP_TAG, "######################## UUID - END ###########################");

        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, UTILS_TASK_SAMPLING_RATE );
    }
    
    // free resources
    vTaskDelete( NULL );
}