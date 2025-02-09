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
 * @file ahtxx.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for ahtxx sensor types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AHTXX_H__
#define __AHTXX_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>
#include <ahtxx_version.h>


/**
 * public constant definitions
 */

#define I2C_AHTXX_SCL_SPEED_HZ  UINT32_C(100000) //!< ahtxx I2C scl clock frequency (100KHz)
#define I2C_AHTXX_DEV_ADDR      UINT8_C(0x38)    //!< ahtxx I2C device address


/**
 * public macro definitions
 */

#define I2C_AHT30_CONFIG_DEFAULT {                  \
    .dev_address     = I2C_AHTXX_DEV_ADDR,          \
    .dev_clock_speed = I2C_AHTXX_SCL_SPEED_HZ,      \
    .aht_type        = I2C_AHTXX_AHT30 }

#define I2C_AHT25_CONFIG_DEFAULT {                  \
    .dev_address     = I2C_AHTXX_DEV_ADDR,          \
    .dev_clock_speed = I2C_AHTXX_SCL_SPEED_HZ,      \
    .aht_type        = I2C_AHTXX_AHT25 }

#define I2C_AHT21_CONFIG_DEFAULT {                  \
    .dev_address     = I2C_AHTXX_DEV_ADDR,          \
    .dev_clock_speed = I2C_AHTXX_SCL_SPEED_HZ,      \
    .aht_type        = I2C_AHTXX_AHT21 }

#define I2C_AHT20_CONFIG_DEFAULT {                  \
    .dev_address     = I2C_AHTXX_DEV_ADDR,          \
    .dev_clock_speed = I2C_AHTXX_SCL_SPEED_HZ,      \
    .aht_type        = I2C_AHTXX_AHT20 }

#define I2C_AHT10_CONFIG_DEFAULT {                  \
    .dev_address     = I2C_AHTXX_DEV_ADDR,          \
    .dev_clock_speed = I2C_AHTXX_SCL_SPEED_HZ,      \
    .aht_type        = I2C_AHTXX_AHT10 }


#ifdef __cplusplus
extern "C" {
#endif

/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief AHTXX types enumerator definition.
 * 
 * @note AHTXX types vary slightly with respect to setup and initialization according to available documentation.
 * The AHT10 is setup through the initialization command.  The AHT20(?), AHT21, AHT25 and AHT30 are setup by initializing 
 * 0x1b, 0x1c, and 0x1e registers.
 */
typedef enum i2c_ahtxx_types_e {
    I2C_AHTXX_AHT10,    /*!< */
    I2C_AHTXX_AHT20,
    I2C_AHTXX_AHT21,
    I2C_AHTXX_AHT25,
    I2C_AHTXX_AHT30
} i2c_ahtxx_types_t;

/**
 * @brief AHTXX status register structure definition.
 */
typedef union __attribute__((packed)) i2c_ahtxx_status_register_u {
    struct {
        uint8_t reserved1:3; /*!< reserved                       (bit:0-2)  */
        bool calibrated:1;   /*!< ahtxx is calibrated when true  (bit:3) */
        uint8_t reserved2:3; /*!< reserved                       (bit:4-6) */
        bool busy:1;         /*!< ahtxx is busy when true        (bit:7) */
    } bits;
    uint8_t reg;
} i2c_ahtxx_status_register_t;

/**
 * @brief AHTXX configuration structure definition.
 */
typedef struct i2c_ahtxx_config_s {
    uint16_t          dev_address;      /*!< i2c device address */
    uint32_t          dev_clock_speed;  /*!< i2c device scl clock speed  */
    i2c_ahtxx_types_t aht_type;         /*!< aht sensor type, see `i2c_ahtxx_types_t` enumerator for support sensor types */
} i2c_ahtxx_config_t;

/**
 * @brief AHTXX context structure.
 */
struct i2c_ahtxx_context_s {
    i2c_ahtxx_config_t      config;     /*!< configuration by the caller */
    i2c_master_dev_handle_t dev_handle; /*!< i2c device handle */
};

/**
 * @brief AHTXX context structure definition.
 */
typedef struct i2c_ahtxx_context_s i2c_ahtxx_context_t;

/**
 * @brief AHTXX handle structure definition.
 */
typedef struct i2c_ahtxx_context_s* i2c_ahtxx_handle_t;

/**
 * public function and subroutine declarations
 */

/**
 * @brief Reads status register from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param reg AHTXX status register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_status_register(i2c_ahtxx_handle_t handle, i2c_ahtxx_status_register_t *const reg);

/**
 * @brief Initializes an AHTXX device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ahtxx_config Configuration of AHTXX device.
 * @param[out] ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_init(const i2c_master_bus_handle_t bus_handle, const i2c_ahtxx_config_t *ahtxx_config, i2c_ahtxx_handle_t *const ahtxx_handle);

/**
 * @brief Reads temperature and relative humidity from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param temperature Temperature in degree Celsius.
 * @param humidity Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_measurement(i2c_ahtxx_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Similar to `i2c_aht2x_read_measurement` but it includes dewpoint in the results.
 *
 * @param[in] handle AHTXX device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dewpoint temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_measurements(i2c_ahtxx_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint);

/**
 * @brief Reads busy status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_busy_status(i2c_ahtxx_handle_t handle, bool *const busy);

/**
 * @brief Reads calibration status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_calibration_status(i2c_ahtxx_handle_t handle, bool *const calibrated);

/**
 * @brief Reads busy and calibrated status flags from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_get_status(i2c_ahtxx_handle_t handle, bool *const busy, bool *const calibrated);

/**
 * @brief Issues soft-reset and initializes AHTXX.  See datasheet for details.
 *
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_reset(i2c_ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus.
 *
 * @param[in] handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_remove(i2c_ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus and frees handle.
 * 
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ahtxx_delete(i2c_ahtxx_handle_t handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AHTXX_H__
