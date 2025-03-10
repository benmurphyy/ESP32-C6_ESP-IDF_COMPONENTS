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
 * @file max30105.c
 *
 * ESP-IDF driver for MAX30105 sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/max30105.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * MAX30105 definitions
 */

#define I2C_MAX30105_REG_INT_STS1_R              UINT8_C(0x00) /*!< max30105 */ 
#define I2C_MAX30105_REG_INT_STS2_R              UINT8_C(0x01) /*!< max30105 */ 
#define I2C_MAX30105_REG_INT_ENB1_RW             UINT8_C(0x02) /*!< max30105 */ 
#define I2C_MAX30105_REG_INT_ENB2_RW             UINT8_C(0x03) /*!< max30105 */ 

#define I2C_MAX30105_REG_FIFO_WR_PTR_RW          UINT8_C(0x04) /*!< max30105 */ 
#define I2C_MAX30105_REG_FIFO_OVF_CNT_RW         UINT8_C(0x05) /*!< max30105 */ 
#define I2C_MAX30105_REG_FIFO_RD_PTR_RW          UINT8_C(0x06) /*!< max30105 */ 
#define I2C_MAX30105_REG_FIFO_DATA_RW            UINT8_C(0x07) /*!< max30105 */ 

#define I2C_MAX30105_REG_FIFO_CONFIG_RW          UINT8_C(0x08) /*!< max30105 */
#define I2C_MAX30105_REG_MODE_CONFIG_RW          UINT8_C(0x09) /*!< max30105 */
#define I2C_MAX30105_REG_SPO2_CONFIG_RW          UINT8_C(0x0A) /*!< max30105 */
#define I2C_MAX30105_REG_LED1_PA_RW              UINT8_C(0x0C) /*!< max30105 */
#define I2C_MAX30105_REG_LED2_PA_RW              UINT8_C(0x0D) /*!< max30105 */
#define I2C_MAX30105_REG_LED3_PA_RW              UINT8_C(0x0E) /*!< max30105 */
#define I2C_MAX30105_REG_PILOT_PA_RW             UINT8_C(0x10) /*!< max30105 */
#define I2C_MAX30105_REG_MLED1_MC_RW             UINT8_C(0x11) /*!< max30105 */
#define I2C_MAX30105_REG_MLED2_MC_RW             UINT8_C(0x12) /*!< max30105 */

#define I2C_MAX30105_REG_DIETEMP_INT_R           UINT8_C(0x1F) /*!< max30105 */
#define I2C_MAX30105_REG_DIETEMP_FRAC_R          UINT8_C(0x20) /*!< max30105 */
#define I2C_MAX30105_REG_DIETEMP_CONFIG_R        UINT8_C(0x21) /*!< max30105 */

#define I2C_MAX30105_REG_PROX_INT_THLD_RW        UINT8_C(0x30) /*!< max30105 */
#define I2C_MAX30105_REG_REV_ID_R                UINT8_C(0xFE) /*!< max30105 */
#define I2C_MAX30105_REG_PART_ID_R               UINT8_C(0xFE) /*!< max30105 */



#define I2C_MAX30105_DATA_POLL_TIMEOUT_MS       UINT16_C(100)
#define I2C_MAX30105_DATA_READY_DELAY_MS        UINT16_C(2)
#define I2C_MAX30105_POWERUP_DELAY_MS           UINT16_C(120)
#define I2C_MAX30105_RESET_DELAY_MS             UINT16_C(25)
#define I2C_MAX30105_SETUP_DELAY_MS             UINT16_C(15)
#define I2C_MAX30105_APPSTART_DELAY_MS          UINT16_C(10)    /*!< max30105 delay after initialization before application start-up */
#define I2C_MAX30105_CMD_DELAY_MS               UINT16_C(5)     /*!< max30105 delay before attempting I2C transactions after a command is issued */
#define I2C_MAX30105_TX_RX_DELAY_MS             UINT16_C(10)    /*!< max30105 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "max30105";



esp_err_t i2c_max30105_get_interrupt_status1_register(i2c_max30105_handle_t max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_STS1_R, &max30105_handle->irq_status1_reg.reg), TAG, "read interrupt status 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_max30105_get_interrupt_status2_register(i2c_max30105_handle_t max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_STS2_R, &max30105_handle->irq_status2_reg.reg), TAG, "read interrupt status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_max30105_get_interrupt_enable1_register(i2c_max30105_handle_t max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_ENB1_RW, &max30105_handle->irq_enable1_reg.reg), TAG, "read interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_max30105_set_interrupt_enable1_register(i2c_max30105_handle_t max30105_handle, const i2c_max30105_interrupt_enable1_register_t irq_enable1_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* copy register */
    i2c_max30105_interrupt_enable1_register_t irq_enable1 = { .reg = irq_enable1_reg.reg };

    /* set register reserved settings */
    irq_enable1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_ENB1_RW, irq_enable1.reg), TAG, "write interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_max30105_get_interrupt_enable1_register(max30105_handle), TAG, "read interrupt enable 1 register for set interrupt enable 1 register failed" );

    return ESP_OK;
}

esp_err_t i2c_max30105_get_interrupt_enable2_register(i2c_max30105_handle_t max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_ENB2_RW, &max30105_handle->irq_enable2_reg.reg), TAG, "read interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_max30105_set_interrupt_enable2_register(i2c_max30105_handle_t max30105_handle, const i2c_max30105_interrupt_enable2_register_t irq_enable2_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* copy register */
    i2c_max30105_interrupt_enable2_register_t irq_enable2 = { .reg = irq_enable2_reg.reg };

    /* set register reserved settings */
    irq_enable2.bits.reserved1 = 0;
    irq_enable2.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_INT_ENB2_RW, irq_enable2.reg), TAG, "write interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_max30105_get_interrupt_enable2_register(max30105_handle), TAG, "read interrupt enable 1 register for set interrupt enable 2 register failed" );

    return ESP_OK;
}

esp_err_t i2c_max30105_get_mode_configuration_register(i2c_max30105_handle_t max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_MODE_CONFIG_RW, &max30105_handle->mode_config_reg.reg), TAG, "read mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_max30105_set_mode_configuration_register(i2c_max30105_handle_t max30105_handle, const i2c_max30105_mode_configuration_register_t mode_config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( max30105_handle );

    /* copy register */
    i2c_max30105_mode_configuration_register_t mode_config = { .reg = mode_config_reg.reg };

    /* set register reserved settings */
    mode_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(max30105_handle->i2c_dev_handle, I2C_MAX30105_REG_MODE_CONFIG_RW, mode_config.reg), TAG, "write mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_MAX30105_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_max30105_get_mode_configuration_register(max30105_handle), TAG, "read mode configuration register for set mode configuration register failed" );

    return ESP_OK;
}

