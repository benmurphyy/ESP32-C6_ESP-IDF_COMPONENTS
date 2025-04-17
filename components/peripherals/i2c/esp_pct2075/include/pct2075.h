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
 * @file pct2075.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for pct2075 sensor types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __PCT2075_H__
#define __PCT2075_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
//#include "pct2075_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_PCT2075_DEV_CLK_SPD   UINT32_C(100000) /*!< pct2075 i2c device scl clock frequency (100KHz) */
#define I2C_PCT2075_DEV_ADDR      UINT8_C(0x37)    /*!< pct2075 i2c device address */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `pct2075_config_t` to default configuration settings.
 */
#define I2C_PCT2075_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_PCT2075_DEV_ADDR,          \
    .i2c_clock_speed = I2C_PCT2075_DEV_CLK_SPD }



/**
 * public enumerator, union, and structure definitions
 */

typedef enum pct2075_os_polarities_e {
    PCT2075_OS_POL_ACTIVE_LOW = 0,
    PCT2075_OS_POL_ACTIVE_HIGH = 1
} pct2075_os_polarities_t;

typedef enum pct2075_os_operation_modes_e {
    PCT2075_OS_OP_MODE_COMPARATOR = 0,
    PCT2075_OS_OP_MODE_INTERRUPT = 1
} pct2075_os_operation_modes_t;

typedef enum pct2075_os_fault_queues_e {
    PCT2075_OS_FAULT_QUEUE_1 = (0b00),
    PCT2075_OS_FAULT_QUEUE_2 = (0b01),
    PCT2075_OS_FAULT_QUEUE_4 = (0b10),
    PCT2075_OS_FAULT_QUEUE_6 = (0b11)
} pct2075_os_fault_queues_t;

/**
 * @brief PCT2075 configuration register structure definition.
 */
typedef union __attribute__((packed)) pct2075_config_register_u {
    struct {
        bool                            shutdown_enabled:1; /*!<  pct2075 is shutdown when enabled   (bit:0)  */
        pct2075_os_operation_modes_t    operation_mode:1;   /*!< pct2075 os operation mode  (bit:1) */
        pct2075_os_polarities_t         polarity:1;         /*!< pct2075 os polarity    (bit:2) */
        pct2075_os_fault_queues_t       fault_queue:2;      /*!< pct2075 os fault queue programming   (bit:3-4) */
        uint8_t                         reserved:3;         /*!< reserved        (bit:5-7) */
    } bits;
    uint8_t reg;
} pct2075_config_register_t;

/**
 * @brief PCT2075 configuration structure definition.
 */
typedef struct pct2075_config_s {
    uint16_t          i2c_address;          /*!< pct2075 i2c device address */
    uint32_t          i2c_clock_speed;      /*!< pct2075 i2c device scl clock speed in hz */
} pct2075_config_t;

/**
 * @brief PCT2075 context structure.
 */
struct pct2075_context_t {
    pct2075_config_t        dev_config; /*!< pct2075 device configuration */
    i2c_master_dev_handle_t i2c_handle; /*!< pct2075 i2c device handle */
};

/**
 * @brief PCT2075 context structure definition.
 */
typedef struct pct2075_context_t pct2075_context_t;

/**
 * @brief PCT2075 handle structure definition.
 */
typedef struct pct2075_context_t* pct2075_handle_t;

/**
 * public function and subroutine declarations
 */

/**
 * @brief Reads configuration register from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] reg PCT2075 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_config_register(pct2075_handle_t handle, pct2075_config_register_t *const reg);

/**
 * @brief Writes configuration register to PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] reg PCT2075 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_config_register(pct2075_handle_t handle, const pct2075_config_register_t reg);

/**
 * @brief Initializes an PCT2075 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] pct2075_config PCT2075 device configuration.
 * @param[out] pct2075_handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_init(const i2c_master_bus_handle_t master_handle, const pct2075_config_t *pct2075_config, pct2075_handle_t *const pct2075_handle);

/**
 * @brief Read temperature measurement from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_temperature(pct2075_handle_t handle, float *const temperature);

/**
 * @brief Removes an PCT2075 device from master bus.
 *
 * @param[in] handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_remove(pct2075_handle_t handle);

/**
 * @brief Removes an PCT2075 device from master bus and frees handle.
 * 
 * @param handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_delete(pct2075_handle_t handle);

/**
 * @brief Converts PCT2075 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* PCT2075 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* pct2075_get_fw_version(void);

/**
 * @brief Converts PCT2075 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t PCT2075 firmware version number.
 */
int32_t pct2075_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __PCT2075_H__
