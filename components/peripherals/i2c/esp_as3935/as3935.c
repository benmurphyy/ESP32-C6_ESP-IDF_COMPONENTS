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
 * @file as3935.c
 *
 * ESP-IDF driver for AS3935 lightning detection sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "as3935.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/gpio.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * @brief AS3935 definitions
*/
/* AS3935 localized constants */
#define AS3935_IRQ_FLAG_DEFAULT         (0)
#define AS3935_MUTEX_WAIT_TIME          (50)  // ticks
#define AS3935_EVENT_LOOP_POOL_DELAY_MS (50)  // milliseconds
#define AS3935_EVENT_LOOP_POST_DELAY_MS (100) // milliseconds
#define AS3935_EVENT_LOOP_QUEUE_SIZE    (16)
#define AS3935_EVENT_TASK_NAME          "as3935_evt_tsk"
#define AS3935_EVENT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
#define AS3935_EVENT_TASK_PRIORITY      (tskIDLE_PRIORITY + 6)

/** 
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define ENSURE_TRUE(ACTION) do { BaseType_t __res = (ACTION); assert(__res == pdTRUE); (void)__res; } while (0)

/**
 * @brief AS3935 monitor event base definition.
 */
ESP_EVENT_DEFINE_BASE(ESP_AS3935_EVENT);

/**
 * static constant declerations
 */
static const char *TAG = "as3935";

/**
 * @brief esp AS3935 device state machine structure.
*/
typedef struct ESP_AS3935_DEVICE_TAG {
    uint32_t                irq_io_num;          /*!< as3935 interrupt pin to mcu */
    as3935_device_t         device;              /*!< as3935 device parent class */     
    esp_event_loop_handle_t event_loop_handle;   /*!< as3935 event loop handle */
    QueueHandle_t           event_queue_handle;  /*!< as3935 event queue handle */ 
    TaskHandle_t            task_monitor_handle; /*!< as3935 task monitor handle */ 
    i2c_as3935_handle_t     i2c_as3935_handle;   /*!< I2C as3935 handle */
    SemaphoreHandle_t       i2c_mutex_handle;    /*!< I2C master bus mutex handle */
} esp_as3935_device_t;

/** 
 * functions and subrountines
*/

static inline uint8_t i2c_as3935_convert_lightning_distance_km(i2c_as3935_lightning_distances_t distance) {
    switch(distance) {
        case I2C_AS3935_L_DISTANCE_OVERHEAD:
            return 0;
        case I2C_AS3935_L_DISTANCE_5KM:
            return 5;
        case I2C_AS3935_L_DISTANCE_6KM:
            return 6;
        case I2C_AS3935_L_DISTANCE_8KM:
            return 8;
        case I2C_AS3935_L_DISTANCE_10KM:
            return 10;
        case I2C_AS3935_L_DISTANCE_12KM:
            return 12;
        case I2C_AS3935_L_DISTANCE_14KM:
            return 14;
        case I2C_AS3935_L_DISTANCE_17KM:
            return 17;
        case I2C_AS3935_L_DISTANCE_20KM:
            return 20;
        case I2C_AS3935_L_DISTANCE_24KM:
            return 24;
        case I2C_AS3935_L_DISTANCE_27KM:
            return 27;
        case I2C_AS3935_L_DISTANCE_31KM:
            return 31;
        case I2C_AS3935_L_DISTANCE_34KM:
            return 34;
        case I2C_AS3935_L_DISTANCE_37KM:
            return 37;
        case I2C_AS3935_L_DISTANCE_40KM:
            return 40;
        case I2C_AS3935_L_DISTANCE_OO_RANGE:
            return 255;
        default:
            return 255;
    }
}

static inline void IRAM_ATTR as3935_monitor_gpio_isr_handler( void *pvParameters ) {
    esp_as3935_device_t *esp_as3935_device = (esp_as3935_device_t *)pvParameters;
    xQueueSendFromISR(esp_as3935_device->event_queue_handle, &esp_as3935_device->irq_io_num, NULL);
}

static inline void as3935_monitor_task_entry( void *pvParameters ) {
    esp_as3935_device_t *esp_as3935_device = (esp_as3935_device_t *)pvParameters;
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(esp_as3935_device->event_queue_handle, &io_num, portMAX_DELAY)) {
            
            /* wait at least 2ms before reading the interrupt register */
            vTaskDelay(pdMS_TO_TICKS(I2C_AS3935_INTERRUPT_DELAY_MS));
            
            /* ensure i2c master bus mutex is available before reading as3935 registers */
            ENSURE_TRUE( xSemaphoreTake(esp_as3935_device->i2c_mutex_handle, AS3935_MUTEX_WAIT_TIME) );
            
            i2c_as3935_interrupt_states_t irq_state;
            if(i2c_as3935_get_interrupt_state(esp_as3935_device->i2c_as3935_handle, &irq_state) != 0) {
                ESP_LOGE(TAG, "as3935 device read interrupt state (register 0x03) failed");
            } else {
                if(irq_state == I2C_AS3935_INT_NOISE) {
                    /* set parent device fields to defaults */
                    esp_as3935_device->device.lightning_distance = I2C_AS3935_L_DISTANCE_OO_RANGE;
                    esp_as3935_device->device.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, I2C_AS3935_INT_NOISE,
                                  &(esp_as3935_device->device), sizeof(as3935_device_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else if(irq_state == I2C_AS3935_INT_DISTURBER) {
                    /* set parent device fields to defaults */
                    esp_as3935_device->device.lightning_distance = I2C_AS3935_L_DISTANCE_OO_RANGE;
                    esp_as3935_device->device.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, I2C_AS3935_INT_DISTURBER,
                                  &(esp_as3935_device->device), sizeof(as3935_device_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else if(irq_state == I2C_AS3935_INT_LIGHTNING) {
                    uint32_t lightning_energy;
                    i2c_as3935_lightning_distances_t lightning_distance;

                    if(i2c_as3935_get_lightning_event(esp_as3935_device->i2c_as3935_handle, &lightning_distance, &lightning_energy) != 0) {
                        ESP_LOGE(TAG, "as3935 device read lightning distance and energy failed");
                    } else {
                        /* set parent device fields to defaults */
                        esp_as3935_device->device.lightning_distance = lightning_distance;
                        esp_as3935_device->device.lightning_energy   = lightning_energy;

                        /* send signal to notify that one unknown statement has been met */
                        esp_event_post_to(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, I2C_AS3935_INT_LIGHTNING,
                                  &(esp_as3935_device->device), sizeof(as3935_device_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                    }
                } else if(irq_state == I2C_AS3935_INT_NONE) {
                    /* set parent device fields to defaults */
                    esp_as3935_device->device.lightning_distance = I2C_AS3935_L_DISTANCE_OO_RANGE;
                    esp_as3935_device->device.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, I2C_AS3935_INT_NONE,
                                  &(esp_as3935_device->device), sizeof(as3935_device_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else {
                    /* set parent device fields to defaults */
                    esp_as3935_device->device.lightning_distance = I2C_AS3935_L_DISTANCE_OO_RANGE;
                    esp_as3935_device->device.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, 200,
                                  &(esp_as3935_device->device), sizeof(as3935_device_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                }
            }
            /* ensure i2c master bus mutex is released */
            ENSURE_TRUE( xSemaphoreGive(esp_as3935_device->i2c_mutex_handle) );
        }
        /* drive the event loop */
        esp_event_loop_run(esp_as3935_device->event_loop_handle, pdMS_TO_TICKS(AS3935_EVENT_LOOP_POOL_DELAY_MS));
    }
    vTaskDelete( NULL );
}


as3935_monitor_handle_t as3935_monitor_init(i2c_master_bus_handle_t bus_handle, const i2c_as3935_config_t *as3935_config) {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE; 
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL<<as3935_config->irq_io_num);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure gpio with the given settings
    gpio_config(&io_conf);

    /* create as3935 device state object */
    esp_as3935_device_t *esp_as3935_device = calloc(1, sizeof(esp_as3935_device_t));
    if (!esp_as3935_device) {
        ESP_LOGE(TAG, "calloc memory for esp_as3935_device_t failed");
        goto err_device;
    }

    /* create i2c mutex handle */
    esp_as3935_device->i2c_mutex_handle = xSemaphoreCreateMutex();
    if(esp_as3935_device->i2c_mutex_handle == NULL) {
        ESP_LOGE(TAG, "create i2c mutex failed");
        goto err_emutex;
    }

    /* copy config to as3935 state object */
    esp_as3935_device->irq_io_num = as3935_config->irq_io_num;

    /* create event loop handle */
    esp_event_loop_args_t loop_args = {
        .queue_size = AS3935_EVENT_LOOP_QUEUE_SIZE,
        .task_name = NULL
    };
    if (esp_event_loop_create(&loop_args, &esp_as3935_device->event_loop_handle) != ESP_OK) {
        ESP_LOGE(TAG, "create event loop failed");
        goto err_eloop;
    }

    /* create a event queue to handle gpio event from isr */
    esp_as3935_device->event_queue_handle = xQueueCreate(10, sizeof(uint32_t));
    if (!esp_as3935_device->event_queue_handle) {
        ESP_LOGE(TAG, "create event queue handle failed");
        goto err_equeue;
    }

    /* create i2c as3935 handle */
    esp_err_t dev_err = i2c_as3935_init(bus_handle, as3935_config, &esp_as3935_device->i2c_as3935_handle);
    if(dev_err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_bus_device_create as3935 handle initialization failed %s", esp_err_to_name(dev_err));
        goto err_i2c_as3935_init;
    }

    /* create as3935 monitor task handle */
    BaseType_t err = xTaskCreatePinnedToCore( 
        as3935_monitor_task_entry, 
        AS3935_EVENT_TASK_NAME, 
        AS3935_EVENT_TASK_STACK_SIZE, 
        esp_as3935_device, 
        AS3935_EVENT_TASK_PRIORITY,
        &esp_as3935_device->task_monitor_handle, 
        APP_CPU_NUM );
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create as3935 monitor task on CPU(1) failed");
        goto err_task_create;
    }
    
    
    ESP_LOGI(TAG, "as3935 device init OK");
    return esp_as3935_device;

    /* error handling */
err_task_create:
    vTaskDelete(esp_as3935_device->task_monitor_handle);
err_i2c_as3935_init:
    i2c_as3935_rm(esp_as3935_device->i2c_as3935_handle);
err_equeue:
    vQueueDelete(esp_as3935_device->event_queue_handle);
err_eloop:
    esp_event_loop_delete(esp_as3935_device->event_loop_handle);
err_emutex:
    vSemaphoreDelete(esp_as3935_device->i2c_mutex_handle);
err_device:
    free(esp_as3935_device);
    return NULL;
}

esp_err_t as3935_monitor_deinit(as3935_monitor_handle_t monitor_handle) {
    esp_as3935_device_t *esp_as3935_device = (esp_as3935_device_t *)monitor_handle;

    /* free-up resources */
    vTaskDelete(esp_as3935_device->task_monitor_handle);
    esp_event_loop_delete(esp_as3935_device->event_loop_handle);
    vQueueDelete(esp_as3935_device->event_queue_handle);
    vSemaphoreDelete(esp_as3935_device->i2c_mutex_handle);
    esp_err_t err = i2c_as3935_rm(esp_as3935_device->i2c_as3935_handle);
    free(esp_as3935_device);

    return err;
}

esp_err_t as3935_monitor_add_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler, void *handler_args) {
    esp_as3935_device_t *esp_as3935_device = (esp_as3935_device_t *)monitor_handle;

    /* install as3935 monitor gpio isr service */
    gpio_install_isr_service(AS3935_IRQ_FLAG_DEFAULT);

    /* hook as3935 monitor isr handler for specific gpio pin and as3935 state object */
    gpio_isr_handler_add(esp_as3935_device->irq_io_num, as3935_monitor_gpio_isr_handler, (void *)esp_as3935_device);

    /* hook esp event handler for caller */
    return esp_event_handler_register_with(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t as3935_monitor_remove_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler) {
    esp_as3935_device_t *esp_as3935_device = (esp_as3935_device_t *)monitor_handle;

    /* remove isr handler for gpio number */
    gpio_isr_handler_remove(esp_as3935_device->irq_io_num);

    /* remove esp event handler from caller */
    return esp_event_handler_unregister_with(esp_as3935_device->event_loop_handle, ESP_AS3935_EVENT, ESP_EVENT_ANY_ID, event_handler);
}

static inline esp_err_t i2c_as3935_get_registers(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_as3935_get_0x00_register(as3935_handle) );

    ESP_ERROR_CHECK( i2c_as3935_get_0x01_register(as3935_handle) );

    ESP_ERROR_CHECK( i2c_as3935_get_0x02_register(as3935_handle) );

    ESP_ERROR_CHECK( i2c_as3935_get_0x03_register(as3935_handle) );

    ESP_ERROR_CHECK( i2c_as3935_get_0x08_register(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_0x00_register(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_00, &as3935_handle->reg_0x00.reg) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_0x01_register(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_01, &as3935_handle->reg_0x01.reg) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_0x02_register(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_02, &as3935_handle->reg_0x02.reg) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_0x03_register(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_03, &as3935_handle->reg_0x03.reg) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_0x08_register(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_08, &as3935_handle->reg_0x08.reg) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_0x00_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x00_register_t reg) {
    ESP_ARG_CHECK( as3935_handle );

    i2c_as3935_0x00_register_t reg_0x00 = { .reg = reg.reg };
    reg_0x00.bits.reserved = 0;

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_00, reg_0x00.reg) );

    /* set device handle register 0x00 */
    ESP_ERROR_CHECK( i2c_as3935_get_0x00_register(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_0x01_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x01_register_t reg) {
    ESP_ARG_CHECK( as3935_handle );

    i2c_as3935_0x01_register_t reg_0x01 = { .reg = reg.reg };
    reg_0x01.bits.reserved = 0;

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_01, reg_0x01.reg) );

    /* set device handle register 0x01 */
    ESP_ERROR_CHECK( i2c_as3935_get_0x01_register(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_0x02_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x02_register_t reg) {
    ESP_ARG_CHECK( as3935_handle );

    i2c_as3935_0x02_register_t reg_0x02 = { .reg = reg.reg };
    reg_0x02.bits.reserved = 0;

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_02, reg_0x02.reg) );

    /* set device handle register 0x02 */
    ESP_ERROR_CHECK( i2c_as3935_get_0x02_register(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_0x03_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x03_register_t reg) {
    ESP_ARG_CHECK( as3935_handle );

    i2c_as3935_0x03_register_t reg_0x03 = { .reg = reg.reg };
    reg_0x03.bits.reserved = 0;

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_03, reg_0x03.reg) );

    /* set device handle register 0x03 */
    ESP_ERROR_CHECK( i2c_as3935_get_0x03_register(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_0x08_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x08_register_t reg) {
    ESP_ARG_CHECK( as3935_handle );

    i2c_as3935_0x08_register_t reg_0x08 = { .reg = reg.reg };
    reg_0x08.bits.reserved = 0;

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_08, reg_0x08.reg) );

    /* set device handle register 0x08 */
    ESP_ERROR_CHECK( i2c_as3935_get_0x08_register(as3935_handle) );

    return ESP_OK;
}


esp_err_t i2c_as3935_init(i2c_master_bus_handle_t bus_handle, const i2c_as3935_config_t *as3935_config, i2c_as3935_handle_t *as3935_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && as3935_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_AS3935_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, as3935_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, as3935 device handle initialization failed", as3935_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_as3935_handle_t out_handle = (i2c_as3935_handle_t)calloc(1, sizeof(i2c_as3935_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as3935 device");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = as3935_config->dev_config.device_address,
        .scl_speed_hz       = as3935_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* copy configuration */
    out_handle->irq_io_enabled = as3935_config->irq_io_enabled;
    out_handle->irq_io_num     = as3935_config->irq_io_num;

    /* as3935 attempt to read device registers */
    ESP_GOTO_ON_ERROR(i2c_as3935_get_registers(out_handle), err_handle, TAG, "i2c as3925 read device registers failed");

    /* set device handle */
    *as3935_handle = out_handle;

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_as3935_register_isr(i2c_as3935_handle_t as3935_handle, const as3935_isr_t isr) {
    /* validate arguments */
    ESP_ARG_CHECK( as3935_handle );

    ESP_RETURN_ON_ERROR( gpio_isr_handler_add(as3935_handle->irq_io_num, ((gpio_isr_t) * (isr)), ((void *) as3935_handle)), TAG, "isr handler add failed" );
    ESP_RETURN_ON_ERROR( gpio_intr_enable(as3935_handle->irq_io_num), TAG, "interrupt enable failed" );

    return ESP_OK;
}

esp_err_t i2c_as3935_reset_to_defaults(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_CMD_PRESET_DEFAULT, I2C_AS3935_REG_RST) );

    ESP_ERROR_CHECK( i2c_as3935_get_registers(as3935_handle) );

    return ESP_OK;
}

esp_err_t i2c_as3935_calibrate_rco(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_as3935_disable_power(as3935_handle) );
    ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_CMD_CALIB_RCO, I2C_AS3935_REG_RST) );

    ESP_ERROR_CHECK( i2c_as3935_set_display_oscillator_on_irq(as3935_handle, I2C_AS3935_OSCILLATOR_SYSTEM_RC, true));
    vTaskDelay(pdMS_TO_TICKS(I2C_AS3935_CALIBRATION_DELAY_MS));
    ESP_ERROR_CHECK( i2c_as3935_set_display_oscillator_on_irq(as3935_handle, I2C_AS3935_OSCILLATOR_SYSTEM_RC, false));

    return ESP_OK;
}

esp_err_t i2c_as3935_clear_lightning_statistics(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    //ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_02, I2C_AS3935_REG_96) );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t i2c_as3935_enable_power(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x00 from device handle */
    i2c_as3935_0x00_register_t reg_0x00 = as3935_handle->reg_0x00;

    /* set register 0x00[0] bit */
    reg_0x00.bits.power_state = I2C_AS3935_POWER_ON;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x00_register(as3935_handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_analog_frontend(i2c_as3935_handle_t as3935_handle, i2c_as3935_analog_frontends_t *const analog_frontend) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x00[5:1]
    ESP_ERROR_CHECK( i2c_as3935_get_0x00_register(as3935_handle) );

    *analog_frontend = as3935_handle->reg_0x00.bits.analog_frontend;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_watchdog_threshold(i2c_as3935_handle_t as3935_handle, i2c_as3935_watchdog_thresholds_t *const watchdog_threshold) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x01[3:0]
    ESP_ERROR_CHECK( i2c_as3935_get_0x01_register(as3935_handle) );

    *watchdog_threshold = as3935_handle->reg_0x01.bits.watchdog_threshold;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_noise_floor_level(i2c_as3935_handle_t as3935_handle, i2c_as3935_noise_levels_t *const noise_level) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x01[6:4]
    ESP_ERROR_CHECK( i2c_as3935_get_0x01_register(as3935_handle) );

    *noise_level = as3935_handle->reg_0x01.bits.noise_floor_level;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_spike_rejection(i2c_as3935_handle_t as3935_handle, uint8_t *const spike_rejection) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x02[3:0]
    ESP_ERROR_CHECK( i2c_as3935_get_0x02_register(as3935_handle) );

    *spike_rejection = as3935_handle->reg_0x02.bits.spike_rejection & 0b1111;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_minimum_lightnings(i2c_as3935_handle_t as3935_handle, i2c_as3935_minimum_lightnings_t *const min_lightnings) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x02[5:4]
    ESP_ERROR_CHECK( i2c_as3935_get_0x02_register(as3935_handle) );

    *min_lightnings = as3935_handle->reg_0x02.bits.min_num_lightning;

    return ESP_OK;
}

esp_err_t i2c_as3935_enable_disturber_detection(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x03 from device handle */
    i2c_as3935_0x03_register_t reg_0x03 = as3935_handle->reg_0x03;

    /* set register 0x03[5] bit */
    reg_0x03.bits.disturber_detection_state = I2C_AS3935_DISTURBER_DETECTION_ENABLED;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x03_register(as3935_handle, reg_0x03) );

    return ESP_OK;

}

esp_err_t i2c_as3935_get_frequency_division_ratio(i2c_as3935_handle_t as3935_handle, i2c_as3935_frequency_division_ratios_t *const ratio) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x03[7:6]
    ESP_ERROR_CHECK( i2c_as3935_get_0x03_register(as3935_handle) );

    *ratio = as3935_handle->reg_0x03.bits.freq_div_ratio;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_display_oscillator_on_irq(i2c_as3935_handle_t as3935_handle, const i2c_as3935_oscillator_modes_t oscillator_mode, bool *const enabled) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x08[5]|[6]|[7]
    ESP_ERROR_CHECK( i2c_as3935_get_0x08_register(as3935_handle) );

    *enabled = false;

    switch (oscillator_mode) {
        case I2C_AS3935_OSCILLATOR_ANTENNA_LC:
            if(as3935_handle->reg_0x08.bits.display_lco_state == I2C_AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        case I2C_AS3935_OSCILLATOR_SYSTEM_RC:
            if(as3935_handle->reg_0x08.bits.display_srco_state == I2C_AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        case I2C_AS3935_OSCILLATOR_TIMER_RC:
            if(as3935_handle->reg_0x08.bits.display_trco_state == I2C_AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t i2c_as3935_get_internal_capacitors(i2c_as3935_handle_t as3935_handle, uint8_t *const value) {
    ESP_ARG_CHECK( as3935_handle );

    // reg 0x08[3:0]
    ESP_ERROR_CHECK( i2c_as3935_get_0x08_register(as3935_handle) );

    *value = as3935_handle->reg_0x08.bits.tuning_capacitors & 0b1111;

    return ESP_OK;
}


esp_err_t i2c_as3935_disable_power(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x00 from device handle */
    i2c_as3935_0x00_register_t reg_0x00 = as3935_handle->reg_0x00;

    /* set register 0x00[0] bit */
    reg_0x00.bits.power_state = I2C_AS3935_POWER_OFF;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x00_register(as3935_handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_analog_frontend(i2c_as3935_handle_t as3935_handle, const i2c_as3935_analog_frontends_t analog_frontend) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x00 from device handle */
    i2c_as3935_0x00_register_t reg_0x00 = as3935_handle->reg_0x00;

    /* set register 0x00[5:1] bits */
    reg_0x00.bits.analog_frontend = analog_frontend;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x00_register(as3935_handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_watchdog_threshold(i2c_as3935_handle_t as3935_handle, const i2c_as3935_watchdog_thresholds_t watchdog_threshold) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x01 from device handle */
    i2c_as3935_0x01_register_t reg_0x01 = as3935_handle->reg_0x01;

    /* set register 0x01[3:0] bits */
    reg_0x01.bits.watchdog_threshold = watchdog_threshold;

    /* set device register 0x01 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x01_register(as3935_handle, reg_0x01) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_noise_floor_level(i2c_as3935_handle_t as3935_handle, const i2c_as3935_noise_levels_t noise_level) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x01 from device handle */
    i2c_as3935_0x01_register_t reg_0x01 = as3935_handle->reg_0x01;

    /* set register 0x01[6:4] bits */
    reg_0x01.bits.noise_floor_level = noise_level;

    /* set device register 0x01 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x01_register(as3935_handle, reg_0x01) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_spike_rejection(i2c_as3935_handle_t as3935_handle, const uint8_t spike_rejection) {
    ESP_ARG_CHECK( as3935_handle );

    if (spike_rejection > 15)
        return ESP_ERR_INVALID_ARG;

    /* copy register 0x02 from device handle */
    i2c_as3935_0x02_register_t reg_0x02 = as3935_handle->reg_0x02;

    /* set register 0x02[3:0] bits */
    reg_0x02.bits.spike_rejection &= ~0b1111;
    reg_0x02.bits.spike_rejection |= spike_rejection;

    /* set device register 0x02 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x02_register(as3935_handle, reg_0x02) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_minimum_lightnings(i2c_as3935_handle_t as3935_handle, const i2c_as3935_minimum_lightnings_t min_lightnings) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x02 from device handle */
    i2c_as3935_0x02_register_t reg_0x02 = as3935_handle->reg_0x02;

    /* set register 0x02[5:4] bits */
    reg_0x02.bits.min_num_lightning = min_lightnings;

    /* set device register 0x02 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x02_register(as3935_handle, reg_0x02) );

    return ESP_OK;
}

esp_err_t i2c_as3935_disable_disturber_detection(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x03 from device handle */
    i2c_as3935_0x03_register_t reg_0x03 = as3935_handle->reg_0x03;

    /* set register 0x03[5] bit */
    reg_0x03.bits.disturber_detection_state = I2C_AS3935_DISTURBER_DETECTION_DISABLED;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x03_register(as3935_handle, reg_0x03) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_frequency_division_ratio(i2c_as3935_handle_t as3935_handle, const i2c_as3935_frequency_division_ratios_t ratio) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x03 from device handle */
    i2c_as3935_0x03_register_t reg_0x03 = as3935_handle->reg_0x03;

    /* set register 0x03[7:6] bits */
    reg_0x03.bits.freq_div_ratio = ratio;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x03_register(as3935_handle, reg_0x03) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_display_oscillator_on_irq(i2c_as3935_handle_t as3935_handle, const i2c_as3935_oscillator_modes_t oscillator_mode, const bool enabled) {
    ESP_ARG_CHECK( as3935_handle );

    /* copy register 0x08 from device handle */
    i2c_as3935_0x08_register_t reg_0x08 = as3935_handle->reg_0x08;

    /* set register 0x08[5]|[6]|[7] bits */
    switch (oscillator_mode) {
        case I2C_AS3935_OSCILLATOR_ANTENNA_LC:
            if(enabled == true) reg_0x08.bits.display_lco_state = I2C_AS3935_CO_IRQ_PIN_ENABLED;
            break;
        case I2C_AS3935_OSCILLATOR_SYSTEM_RC:
            if(enabled == true) reg_0x08.bits.display_srco_state = I2C_AS3935_CO_IRQ_PIN_ENABLED;
            break;
        case I2C_AS3935_OSCILLATOR_TIMER_RC:
            if(enabled == true) reg_0x08.bits.display_trco_state = I2C_AS3935_CO_IRQ_PIN_ENABLED;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* set device register 0x08 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x08_register(as3935_handle, reg_0x08) );

    return ESP_OK;
}

esp_err_t i2c_as3935_set_internal_capacitors(i2c_as3935_handle_t as3935_handle, const uint8_t value) {
    ESP_ARG_CHECK( as3935_handle );

    if (value > 15)
        return ESP_ERR_INVALID_ARG;

    /* copy register 0x08 from device handle */
    i2c_as3935_0x08_register_t reg_0x08 = as3935_handle->reg_0x08;

    /* set register 0x08[3:0] bits */
    reg_0x08.bits.tuning_capacitors &= ~0b1111;
    reg_0x08.bits.tuning_capacitors |= value;

    /* set device register 0x08 */
    ESP_ERROR_CHECK( i2c_as3935_set_0x08_register(as3935_handle, reg_0x08) );

    return ESP_OK;
}

esp_err_t i2c_as3935_get_interrupt_state(i2c_as3935_handle_t as3935_handle, i2c_as3935_interrupt_states_t *const state) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_as3935_get_0x03_register(as3935_handle) );

    *state = as3935_handle->reg_0x03.bits.irq_state;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_lightning_distance(i2c_as3935_handle_t as3935_handle, i2c_as3935_lightning_distances_t *const distance) {
    i2c_as3935_0x07_register_t reg_0x07;

    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_07, &reg_0x07.reg) );

    *distance = reg_0x07.bits.lightning_distance;

    return ESP_OK;
}

esp_err_t i2c_as3935_get_lightning_distance_km(i2c_as3935_handle_t as3935_handle, uint8_t *const distance) {
    i2c_as3935_lightning_distances_t val;

    ESP_ERROR_CHECK( i2c_as3935_get_lightning_distance(as3935_handle, &val) );

    *distance = i2c_as3935_convert_lightning_distance_km(val);

    return ESP_OK;
}

esp_err_t i2c_as3935_get_lightning_energy(i2c_as3935_handle_t as3935_handle, uint32_t *const energy) {
    bit24_uint8_buffer_t data;

    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_master_bus_read_byte24(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_04, &data) );

    data[2] &= 0b11111;

    *energy = 0;
    
    for (size_t i = 0; i < 3; ++i)
        *energy |= data[i] << (i * 8);

    return ESP_OK;
}

esp_err_t i2c_as3935_get_lightning_event(i2c_as3935_handle_t as3935_handle, i2c_as3935_lightning_distances_t *const distance, uint32_t *const energy) {
    ESP_ARG_CHECK( as3935_handle );

    ESP_ERROR_CHECK( i2c_as3935_get_lightning_distance(as3935_handle, distance) );

    ESP_ERROR_CHECK( i2c_as3935_get_lightning_energy(as3935_handle, energy) );

    return ESP_OK;
}


esp_err_t i2c_as3935_remove(i2c_as3935_handle_t as3935_handle) {
    ESP_ARG_CHECK( as3935_handle );

    return i2c_master_bus_rm_device(as3935_handle->i2c_dev_handle);
}