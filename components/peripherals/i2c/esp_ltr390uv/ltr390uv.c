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
 * @file ltr390uv.c
 *
 * ESP-IDF driver for LTR390UV sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ltr390uv.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * LTR390UV definitions
 */

#define I2C_LTR390UV_REG_MAIN_CTRL_RW       UINT8_C(0x00) /* 0x00 */
#define I2C_LTR390UV_REG_ALS_UVS_MEAS_RW    UINT8_C(0x04)
#define I2C_LTR390UV_REG_ALS_UVS_GAIN_RW    UINT8_C(0x05)
#define I2C_LTR390UV_REG_PART_ID_R          UINT8_C(0x06)
#define I2C_LTR390UV_REG_MAIN_STS_R         UINT8_C(0x07)

#define I2C_LTR390UV_REG_ALS_DATA_0_R       UINT8_C(0x0D) /* LSB */
#define I2C_LTR390UV_REG_ALS_DATA_1_R       UINT8_C(0x0E)
#define I2C_LTR390UV_REG_ALS_DATA_2_R       UINT8_C(0x0F) /* MSB */

#define I2C_LTR390UV_REG_UVS_DATA_0_R       UINT8_C(0x10) /* LSB */
#define I2C_LTR390UV_REG_UVS_DATA_1_R       UINT8_C(0x11)
#define I2C_LTR390UV_REG_UVS_DATA_2_R       UINT8_C(0x12) /* MSB */

#define I2C_LTR390UV_REG_INT_CFG_RW         UINT8_C(0x19)
#define I2C_LTR390UV_REG_INT_PST_RW         UINT8_C(0x1A)

#define I2C_LTR390UV_REG_ALS_THRES_UP_0_RW  UINT8_C(0x21) /* LSB */
#define I2C_LTR390UV_REG_ALS_THRES_UP_1_RW  UINT8_C(0x22)
#define I2C_LTR390UV_REG_ALS_THRES_UP_2_RW  UINT8_C(0x23) /* MSB */

#define I2C_LTR390UV_REG_ALS_THRES_LO_0_RW  UINT8_C(0x24) /* LSB */
#define I2C_LTR390UV_REG_ALS_THRES_LO_1_RW  UINT8_C(0x25)
#define I2C_LTR390UV_REG_ALS_THRES_LO_2_RW  UINT8_C(0x26) /* MSB */

#define I2C_LTR390UV_SENSITIVITY_MAX        (2300.0f)       /* see datasheet, section 4.5 */
#define I2C_LTR390UV_INTEGRATION_TIME_MAX   (4.0f * 100.0f) /* I2C_LTR390UV_SR_20BIT */
#define I2C_LTR390UV_GAIN_MAX               (18.0f)         /* I2C_LTR390UV_MG_X18 */

#define I2C_LTR390UV_DATA_POLL_TIMEOUT_MS  UINT16_C(500)
#define I2C_LTR390UV_DATA_READY_DELAY_MS   UINT16_C(2)
#define I2C_LTR390UV_POWERUP_DELAY_MS      UINT16_C(120)
#define I2C_LTR390UV_RESET_DELAY_MS        UINT16_C(25)
#define I2C_LTR390UV_WAKEUP_DELAY_MS       UINT16_C(15)
#define I2C_LTR390UV_APPSTART_DELAY_MS     UINT16_C(10)    /*!< ltr390uv delay after initialization before application start-up */
#define I2C_LTR390UV_CMD_DELAY_MS          UINT16_C(5)     /*!< ltr390uv delay before attempting I2C transactions after a command is issued */
#define I2C_LTR390UV_TX_RX_DELAY_MS        UINT16_C(10)    /*!< ltr390uv delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "ltr390uv";

/**
 * @brief Reads resolution factor from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param resolution_factor LTR390UV resolution factor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_resolution_factor(i2c_ltr390uv_handle_t ltr390uv_handle, float *const resolution_factor) {
    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle);

    /* determine resolution */
    switch(ltr390uv_handle->measure_reg.bits.sensor_resolution) {
        case I2C_LTR390UV_SR_20BIT:
            *resolution_factor = 20.0f;
            break;
        case I2C_LTR390UV_SR_19BIT:
            *resolution_factor = 19.0f;
            break;
        case I2C_LTR390UV_SR_18BIT:
            *resolution_factor = 18.0f;
            break;
        case I2C_LTR390UV_SR_17BIT:
            *resolution_factor = 17.0f;
            break;
        case I2C_LTR390UV_SR_16BIT:
            *resolution_factor = 16.0f;
            break;
        case I2C_LTR390UV_SR_13BIT:
            *resolution_factor = 13.0f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Reads resolution integration time LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param resolution_it LTR390UV resolution integration time.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_resolution_it(i2c_ltr390uv_handle_t ltr390uv_handle, float *const resolution_it) {
    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle);

    /* determine resolution integration time */
    switch(ltr390uv_handle->measure_reg.bits.sensor_resolution) {
        case I2C_LTR390UV_SR_20BIT:
            *resolution_it = 4.0f;
            break;
        case I2C_LTR390UV_SR_19BIT:
            *resolution_it = 2.0f;
            break;
        case I2C_LTR390UV_SR_18BIT:
            *resolution_it = 1.0f;
            break;
        case I2C_LTR390UV_SR_17BIT:
            *resolution_it = 0.5f;
            break;
        case I2C_LTR390UV_SR_16BIT:
            *resolution_it = 0.25f;
            break;
        case I2C_LTR390UV_SR_13BIT:
            *resolution_it = 0.125f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Reads gain multiplier from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param gain_multiplier LTR390UV gain multiplier.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_gain_multiplier(i2c_ltr390uv_handle_t ltr390uv_handle, float *const gain_multiplier) {
    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle);

    /* determine gain */
    switch(ltr390uv_handle->gain_reg.bits.measurement_gain) {
        case I2C_LTR390UV_MG_X1:
            *gain_multiplier = 1.0f;
            break;
        case I2C_LTR390UV_MG_X3:
            *gain_multiplier = 3.0f;
            break;
        case I2C_LTR390UV_MG_X6:
            *gain_multiplier = 6.0f;
            break;
        case I2C_LTR390UV_MG_X9:
            *gain_multiplier = 9.0f;
            break;
        case I2C_LTR390UV_MG_X18:
            *gain_multiplier = 18.0f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Reads UV sensitivity value from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param uv_sensitivity LTR390UV UV sensitivity value.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_uv_sensitivity(i2c_ltr390uv_handle_t ltr390uv_handle, float *const uv_sensitivity) {
    float resolution_it, gain_multiplier;

    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle);

    /* attempt to determine gain multiplier */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_gain_multiplier(ltr390uv_handle, &gain_multiplier), TAG, "read gain multiplier for get uv sensitivity failed" );

    /* attempt to determine resolution integration time */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_resolution_it(ltr390uv_handle, &resolution_it), TAG, "read resolution integration time for get uv sensitivity failed" );

    /* set sensitivity by linearly scaling against known value in the datasheet */
    float gain_scale = gain_multiplier / I2C_LTR390UV_GAIN_MAX;
    float intg_scale = (resolution_it * 100.0f) / I2C_LTR390UV_INTEGRATION_TIME_MAX;
    *uv_sensitivity  = I2C_LTR390UV_SENSITIVITY_MAX * gain_scale * intg_scale;

    return ESP_OK;
}

/**
 * @brief Reads sensor counts based on configured mode from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param sensor_counts LTR390UV sensor counts based on configured mode.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_sensor_counts(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const sensor_counts) {
    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle && sensor_counts);

    /* initialize local variables */
    esp_err_t    ret           = ESP_OK;
    uint64_t     start_time    = esp_timer_get_time(); /* set start time for timeout monitoring */
    bit24_bytes_t rx           = { 0 };
    bool         data_is_ready = false;

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_ltr390uv_get_data_status(ltr390uv_handle, &data_is_ready), err, TAG, "data ready ready for get light counts failed." );
        
        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, I2C_LTR390UV_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* validate operation mode */
    switch (ltr390uv_handle->control_reg.bits.operation_mode) {
        case I2C_LTR390UV_OM_ALS:
            /* attempt i2c read transaction */
            ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte24(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_DATA_0_R, &rx), err, TAG, "read als counts failed" );
            break;
        case I2C_LTR390UV_OM_UVS:
            /* attempt i2c read transaction */
            ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte24(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_UVS_DATA_0_R, &rx), err, TAG, "read uvs counts failed" );
            break;
        default:
            ESP_LOGE(TAG, "Invalid operation mode");
            return ESP_ERR_INVALID_ARG;
    }

    /* concat values */
    *sensor_counts = rx[2] * 65536 + rx[1] * 256 + rx[0];  // (rx[2] << 16)| (rx[1] << 8) | rx[0]

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief Reads and initializes handle registers.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ltr390uv_get_registers(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt to read control register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_control_register(ltr390uv_handle), TAG, "read control register for get registers failed" );

    /* attempt to read measure register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_measure_register(ltr390uv_handle), TAG, "read measure register for get registers failed" );

    /* attempt to read gain register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_gain_register(ltr390uv_handle), TAG, "read gain register for get registers failed" );

    /* attempt to read interrupt configuration register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_interrupt_configuration_register(ltr390uv_handle), TAG, "read interrupt configuration register for get registers failed" );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_status_register(ltr390uv_handle), TAG, "read status register for get registers failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_control_register(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_MAIN_CTRL_RW, &ltr390uv_handle->control_reg.reg), TAG, "read control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_control_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_control_register_t control_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_control_register_t reg = { .reg = control_reg.reg };
    reg.bits.reserved1 = 0;
    reg.bits.reserved2 = 0;
    reg.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_MAIN_CTRL_RW, reg.reg), TAG, "write control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_control_register(ltr390uv_handle), TAG, "read control register for set control register failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_measure_register(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_UVS_MEAS_RW, &ltr390uv_handle->measure_reg.reg), TAG, "read measure register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_measure_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measure_register_t measure_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_measure_register_t reg = { .reg = measure_reg.reg };
    reg.bits.reserved1 = 0;
    reg.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_UVS_MEAS_RW, reg.reg), TAG, "write measure register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_measure_register(ltr390uv_handle), TAG, "read measure register for set measure register failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_gain_register(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_UVS_GAIN_RW, &ltr390uv_handle->gain_reg.reg), TAG, "read gain register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_gain_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_gain_register_t gain_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_gain_register_t reg = { .reg = gain_reg.reg };
    reg.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_UVS_GAIN_RW, reg.reg), TAG, "write gain register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_gain_register(ltr390uv_handle), TAG, "read gain register for set gain register failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_interrupt_configuration_register(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_INT_CFG_RW, &ltr390uv_handle->irq_config_reg.reg), TAG, "read interrupt configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_interrupt_configuration_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_interrupt_configuration_register_t interrupt_config_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_interrupt_configuration_register_t reg = { .reg = interrupt_config_reg.reg };
    reg.bits.reserved1 = 0;
    reg.bits.reserved2 = 0;
    reg.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_INT_CFG_RW, reg.reg), TAG, "write interrupt configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_interrupt_configuration_register(ltr390uv_handle), TAG, "read interrupt configuration register for set interrupt configuration register failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_status_register(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_MAIN_STS_R, &ltr390uv_handle->status_reg.reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_init(i2c_master_bus_handle_t bus_handle, const i2c_ltr390uv_config_t *ltr390uv_config, i2c_ltr390uv_handle_t *ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ltr390uv_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, ltr390uv_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ltr390uv device handle initialization failed", ltr390uv_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_ltr390uv_handle_t out_handle = (i2c_ltr390uv_handle_t)calloc(1, sizeof(i2c_ltr390uv_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ltr390uv device, init failed");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ltr390uv_config->dev_config.device_address,
        .scl_speed_hz       = ltr390uv_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_CMD_DELAY_MS));

    /* copy config to handle */
    out_handle->window_factor     = ltr390uv_config->window_factor;
    out_handle->sensor_resolution = ltr390uv_config->sensor_resolution;
    out_handle->measurement_gain  = ltr390uv_config->measurement_gain;
    out_handle->measurement_rate  = ltr390uv_config->measurement_rate;

    /* attempt soft-reset */
    ESP_GOTO_ON_ERROR(i2c_ltr390uv_reset(out_handle), err_handle, TAG, "soft-reset for init failed");

    /* set device handle */
    *ltr390uv_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_APPSTART_DELAY_MS));

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

esp_err_t i2c_ltr390uv_get_ambient_light(i2c_ltr390uv_handle_t ltr390uv_handle, float *const ambient_light) {
    uint32_t counts;
    float gain_multiplier, resolution_it;

    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle && ambient_light);

    /* validate operation mode */
    if(ltr390uv_handle->control_reg.bits.operation_mode != I2C_LTR390UV_OM_ALS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_mode(ltr390uv_handle, I2C_LTR390UV_OM_ALS), TAG, "write operation mode for get ambient light failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_sensor_counts(ltr390uv_handle, &counts), TAG, "read light counts for get ambient light failed" );

    /* attempt to determine gain multiplier */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_gain_multiplier(ltr390uv_handle, &gain_multiplier), TAG, "read gain multiplier for get ambient light failed" );

    /* attempt to determine resolution integration time */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_resolution_it(ltr390uv_handle, &resolution_it), TAG, "read resolution integration time for get ambient light failed" );

    /* convert light counts to lux */
    *ambient_light = ((0.6f * counts) / (gain_multiplier * resolution_it)) * ltr390uv_handle->window_factor;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_als(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const sensor_counts) {
    uint32_t counts;

    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle && sensor_counts);

    /* validate operation mode */
    if(ltr390uv_handle->control_reg.bits.operation_mode != I2C_LTR390UV_OM_ALS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_mode(ltr390uv_handle, I2C_LTR390UV_OM_ALS), TAG, "write operation mode for get als failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_sensor_counts(ltr390uv_handle, &counts), TAG, "read light counts for get als failed" );

    /* set output parameter */
    *sensor_counts = counts;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_ultraviolet_index(i2c_ltr390uv_handle_t ltr390uv_handle, float *const ultraviolet_index) {
    uint32_t counts;
    float uv_sensitivity;

    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle && ultraviolet_index);

    /* validate operation mode */
    if(ltr390uv_handle->control_reg.bits.operation_mode != I2C_LTR390UV_OM_UVS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_mode(ltr390uv_handle, I2C_LTR390UV_OM_UVS), TAG, "write operation mode for get ultraviolet index failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_sensor_counts(ltr390uv_handle, &counts), TAG, "read light counts for get ultraviolet index failed" );

    /* attempt to determine uv sensitivity */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_uv_sensitivity(ltr390uv_handle, &uv_sensitivity), TAG, "read uv sensitivity for get ultraviolet index failed" );

    /* convert light counts to uvi */
    *ultraviolet_index = (counts / uv_sensitivity) * ltr390uv_handle->window_factor;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_uvs(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const sensor_counts) {
    uint32_t counts;

    /* validate arguments */
    ESP_ARG_CHECK(ltr390uv_handle && sensor_counts);

    /* validate operation mode */
    if(ltr390uv_handle->control_reg.bits.operation_mode != I2C_LTR390UV_OM_UVS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_mode(ltr390uv_handle, I2C_LTR390UV_OM_UVS), TAG, "write operation mode for get uvs failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_sensor_counts(ltr390uv_handle, &counts), TAG, "read light counts for get uvs failed" );

    /* set output parameter */
    *sensor_counts = counts;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_data_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_status_register(ltr390uv_handle), TAG, "read status register for get data status failed" );

    /* set output parameter */
    *ready = ltr390uv_handle->status_reg.bits.data_status;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_power_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const power_on) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_status_register(ltr390uv_handle), TAG, "read status register for get power status failed" );

    /* set output parameter */
    *power_on = ltr390uv_handle->status_reg.bits.power_status;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_interrupt_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const interrupt) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_status_register(ltr390uv_handle), TAG, "read status register for get interrupt status failed" );

    /* set output parameter */
    *interrupt = ltr390uv_handle->status_reg.bits.irq_status;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const data_ready, bool *const power_on, bool *const interrupt) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_status_register(ltr390uv_handle), TAG, "read status register for get status failed" );

    /* set output parameter */
    *data_ready = ltr390uv_handle->status_reg.bits.data_status;
    *power_on   = ltr390uv_handle->status_reg.bits.power_status;
    *interrupt  = ltr390uv_handle->status_reg.bits.irq_status;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_thresholds(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const lower_threshold, uint32_t *const uppper_threshold) {
    bit24_bytes_t lower, upper;

    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_byte24(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_THRES_LO_0_RW, &lower), TAG, "read lower threshold failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_byte24(ltr390uv_handle->i2c_dev_handle, I2C_LTR390UV_REG_ALS_THRES_UP_0_RW, &upper), TAG, "read upper threshold failed" );

    /* set output parameters */  
    *lower_threshold  = (lower[2] << 16) | (lower[1] << 8) | lower[0];
    *uppper_threshold = (upper[2] << 16) | (upper[1] << 8) | upper[0];

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_thresholds(i2c_ltr390uv_handle_t ltr390uv_handle, const uint32_t lower_threshold, const uint32_t uppper_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    bit32_bytes_t lower;
    lower[0] = I2C_LTR390UV_REG_ALS_THRES_LO_0_RW;
    lower[1] = lower_threshold & 0x000000FF; // lsb
    lower[2] = lower_threshold >> 8;
    lower[3] = lower_threshold >> 16;

    bit32_bytes_t upper;
    upper[0] = I2C_LTR390UV_REG_ALS_THRES_UP_0_RW;
    upper[1] = uppper_threshold & 0x000000FF; // lsb
    upper[2] = uppper_threshold >> 8;
    upper[3] = uppper_threshold >> 16;

    ESP_RETURN_ON_ERROR( i2c_master_transmit(ltr390uv_handle->i2c_dev_handle, lower, BIT32_BYTE_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write lower threshold failed" );
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ltr390uv_handle->i2c_dev_handle, upper, BIT32_BYTE_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write upper threshold failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_mode(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_operation_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_control_register(ltr390uv_handle), TAG, "read control register for get mode failed" );

    /* set output parameter */
    *mode = ltr390uv_handle->control_reg.bits.operation_mode;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_mode(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_operation_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_control_register_t reg = { .reg = ltr390uv_handle->control_reg.reg };
    reg.bits.operation_mode = mode;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_control_register(ltr390uv_handle, reg), TAG, "write control register for set mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_resolution(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_sensor_resolutions_t *const resolution) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_measure_register(ltr390uv_handle), TAG, "read measure register for get resolution failed" );

    /* set output parameter */
    *resolution = ltr390uv_handle->measure_reg.bits.sensor_resolution;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_resolution(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_sensor_resolutions_t resolution) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_measure_register_t reg = { .reg = ltr390uv_handle->measure_reg.reg };
    reg.bits.sensor_resolution = resolution;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_measure_register(ltr390uv_handle, reg), TAG, "write measure register for set resolution failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_gain(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_measurement_gains_t *const gain) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_gain_register(ltr390uv_handle), TAG, "read gain register for get gain failed" );

    /* set output parameter */
    *gain = ltr390uv_handle->gain_reg.bits.measurement_gain;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_gain(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measurement_gains_t gain) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_gain_register_t reg = { .reg = ltr390uv_handle->measure_reg.reg };
    reg.bits.measurement_gain = gain;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_gain_register(ltr390uv_handle, reg), TAG, "write gain register for set gain failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_get_rate(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_measurement_rates_t *const rate) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_measure_register(ltr390uv_handle), TAG, "read measure register for get rate failed" );

    /* set output parameter */
    *rate = ltr390uv_handle->measure_reg.bits.measurement_rate;

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_set_rate(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measurement_rates_t rate) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_measure_register_t reg = { .reg = ltr390uv_handle->measure_reg.reg };
    reg.bits.measurement_rate = rate;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_measure_register(ltr390uv_handle, reg), TAG, "write measure register for set rate failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_enable_interrupt(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_ls_interrupts_t light_source) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_interrupt_configuration_register_t reg = { .reg = ltr390uv_handle->irq_config_reg.reg };
    reg.bits.irq_enabled      = true;
    reg.bits.irq_light_source = light_source;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_interrupt_configuration_register(ltr390uv_handle, reg), TAG, "write interrupt configuration register for enable interrupt failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_disable_interrupt(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_interrupt_configuration_register_t reg = { .reg = ltr390uv_handle->irq_config_reg.reg };
    reg.bits.irq_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_interrupt_configuration_register(ltr390uv_handle, reg), TAG, "write interrupt configuration register for disable interrupt failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_enable(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_control_register_t reg = { .reg = ltr390uv_handle->control_reg.reg };
    reg.bits.sensor_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_control_register(ltr390uv_handle, reg), TAG, "write control register for enable failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_WAKEUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_disable(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_control_register_t reg = { .reg = ltr390uv_handle->control_reg.reg };
    reg.bits.sensor_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_control_register(ltr390uv_handle, reg), TAG, "write control register for disable failed" );

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_reset(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* copy and initialize register */
    i2c_ltr390uv_control_register_t c_reg = { .reg = ltr390uv_handle->control_reg.reg };
    c_reg.bits.software_reset_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_control_register(ltr390uv_handle, c_reg), TAG, "write control register for reset failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_RESET_DELAY_MS));

    /* attempt to read registers */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_get_registers(ltr390uv_handle), TAG, "read registers for reset failed" );

    /* attempt device configuration */

    /* copy and initialize registers */
    c_reg = (i2c_ltr390uv_control_register_t) { .reg = ltr390uv_handle->control_reg.reg };
    i2c_ltr390uv_interrupt_configuration_register_t ic_reg = { .reg = ltr390uv_handle->irq_config_reg.reg };
    i2c_ltr390uv_measure_register_t m_reg = { .reg = ltr390uv_handle->measure_reg.reg };
    i2c_ltr390uv_gain_register_t    g_reg = { .reg = ltr390uv_handle->gain_reg.reg };

    ic_reg.bits.irq_enabled      = true;
    ic_reg.bits.irq_light_source = I2C_LTR390UV_LSI_ALS;

    m_reg.bits.sensor_resolution = ltr390uv_handle->sensor_resolution;
    m_reg.bits.measurement_rate  = ltr390uv_handle->measurement_rate;
    g_reg.bits.measurement_gain  = ltr390uv_handle->measurement_gain;
    c_reg.bits.sensor_enabled    = true;

    /* attempt i2c write transactions */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_measure_register(ltr390uv_handle, m_reg), TAG, "write measure register for reset failed" );
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_gain_register(ltr390uv_handle, g_reg), TAG, "write gain register for reset failed" );
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_interrupt_configuration_register(ltr390uv_handle, ic_reg), TAG, "write interrupt configuration register for reset failed" );
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_set_control_register(ltr390uv_handle, c_reg), TAG, "write control register for reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_LTR390UV_WAKEUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ltr390uv_remove(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    return i2c_master_bus_rm_device(ltr390uv_handle->i2c_dev_handle);
}

esp_err_t i2c_ltr390uv_delete(i2c_ltr390uv_handle_t ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ltr390uv_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_ltr390uv_remove(ltr390uv_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(ltr390uv_handle->i2c_dev_handle) {
        free(ltr390uv_handle->i2c_dev_handle);
        free(ltr390uv_handle);
    }

    return ESP_OK;
}