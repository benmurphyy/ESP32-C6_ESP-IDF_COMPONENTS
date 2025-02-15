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
 * @file bmp280.h
 * @defgroup drivers bmp280
 * @{
 *
 * ESP-IDF driver for bmp280 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP280_H__
#define __BMP280_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>
#include "bmp280_version.h"


/*
 * BMP280 definitions
*/
#define I2C_BMP280_DEV_CLK_SPD      UINT32_C(100000) //!< bmp280 I2C default clock frequency (100KHz)

/*
 * supported device addresses
*/
#define I2C_BMP280_DEV_ADDR_LO      UINT8_C(0x76) //!< bmp280 I2C address when ADDR pin floating/low
#define I2C_BMP280_DEV_ADDR_HI      UINT8_C(0x77) //!< bmp280 I2C address when ADDR pin high

/*
 * BMP280 macros
*/
#define I2C_BMP280_CONFIG_DEFAULT {                                              \
        .i2c_address                = I2C_BMP280_DEV_ADDR_HI,                    \
        .i2c_clock_speed            = I2C_BMP280_DEV_CLK_SPD,                    \
        .power_mode                 = I2C_BMP280_POWER_MODE_NORMAL,              \
        .iir_filter                 = I2C_BMP280_IIR_FILTER_OFF,                 \
        .pressure_oversampling      = I2C_BMP280_PRESSURE_OVERSAMPLING_4X,       \
        .temperature_oversampling   = I2C_BMP280_TEMPERATURE_OVERSAMPLING_4X,    \
        .standby_time               = I2C_BMP280_STANDBY_TIME_250MS }


#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMP280 enumerator and structure declarations
*/

/**
 * @brief BMP280 I2C IIR filters coefficient enumerator.
 */
typedef enum i2c_bmp280_iir_filters_e {
    I2C_BMP280_IIR_FILTER_OFF = (0b000),
    I2C_BMP280_IIR_FILTER_2   = (0b001),
    I2C_BMP280_IIR_FILTER_4   = (0b010),
    I2C_BMP280_IIR_FILTER_8   = (0b011),
    I2C_BMP280_IIR_FILTER_16  = (0b100)
} i2c_bmp280_iir_filters_t;

/**
 * @brief BMP280 I2C standby times enumerator.
 * 
 */
typedef enum i2c_bmp280_standby_times_e {
    I2C_BMP280_STANDBY_TIME_0_5MS  = (0b000),  //!< stand by time 0.5ms
    I2C_BMP280_STANDBY_TIME_62_5MS = (0b001),  //!< stand by time 62.5ms
    I2C_BMP280_STANDBY_TIME_125MS  = (0b010),  //!< stand by time 125ms
    I2C_BMP280_STANDBY_TIME_250MS  = (0b011),  //!< stand by time 250ms
    I2C_BMP280_STANDBY_TIME_500MS  = (0b100),  //!< stand by time 500ms
    I2C_BMP280_STANDBY_TIME_1000MS = (0b101),  //!< stand by time 1s
    I2C_BMP280_STANDBY_TIME_2000MS = (0b110),  //!< stand by time 2s BMP280, 10ms BME280
    I2C_BMP280_STANDBY_TIME_4000MS = (0b111)   //!< stand by time 4s BMP280, 20ms BME280
} i2c_bmp280_standby_times_t;

/**
 * @brief BMP280 I2C power modes enumerator.
 * 
 */
typedef enum i2c_bmp280_power_modes_e {
    I2C_BMP280_POWER_MODE_SLEEP   = (0b00), //!< sleep mode, default after power-up
    I2C_BMP280_POWER_MODE_FORCED  = (0b01), //!< measurement is initiated by user
    I2C_BMP280_POWER_MODE_FORCED1 = (0b10), //!< measurement is initiated by user
    I2C_BMP280_POWER_MODE_NORMAL  = (0b11)  //!< continuously cycles between active measurement and inactive (standby-time) periods
} i2c_bmp280_power_modes_t;

/**
 * @brief BMP280 I2C pressure oversampling enumerator.
 * 
 */
typedef enum i2c_bmp280_pressure_oversampling_e {
    I2C_BMP280_PRESSURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BMP280_PRESSURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    I2C_BMP280_PRESSURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    I2C_BMP280_PRESSURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    I2C_BMP280_PRESSURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    I2C_BMP280_PRESSURE_OVERSAMPLING_16X        = (0b101)   //!< ultra high resolution
} i2c_bmp280_pressure_oversampling_t;

/**
 * @brief BMP280 I2C temperature oversampling enumerator.
 * 
 */
typedef enum i2c_bmp280_temperature_oversampling_e {
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_16X        = (0b101),  //!< ultra high resolution
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_16X1       = (0b110),  //!< ultra high resolution
    I2C_BMP280_TEMPERATURE_OVERSAMPLING_16X2       = (0b111)   //!< ultra high resolution
} i2c_bmp280_temperature_oversampling_t;

/**
 * @brief BMP280 I2C status register (0xf3) structure.  The reset state is 0x00 for this register.
 * 
 */
typedef union __attribute__((packed)) i2c_bmp280_status_register_u {
    struct {
        bool    image_update:1; /*!< bmp280 automatically set to 1 when NVM data are being copied to image registers and back to 0 when done (bit:0)  */
        uint8_t reserved1:2;    /*!< reserved (bit:1-2) */
        bool    measuring:1;    /*!< bmp280 automatically set to 1 whenever a conversion is running and back to 0 when results transferred to data registers  (bit:3) */
        uint8_t reserved2:4;    /*!< reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_bmp280_status_register_t;

/**
 * @brief BMP280 I2C control measurement register (0xf4) structure.  The reset state is 0x00 for this register.
 * 
 */
typedef union __attribute__((packed)) i2c_bmp280_control_measurement_register_u {
    struct {
        i2c_bmp280_power_modes_t                power_mode:2;               /*!< bmp280 power mode of the device            (bit:0-1)  */
        i2c_bmp280_pressure_oversampling_t      pressure_oversampling:3;    /*!< bmp280 oversampling of pressure data       (bit:2-4) */
        i2c_bmp280_temperature_oversampling_t   temperature_oversampling:3; /*!< bmp280 oversampling of temperature data    (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bmp280_control_measurement_register_t;

/**
 * @brief BMP280 I2C configuration register (0xf5) structure.  The reset state is 0x00 for this register.
 * 
 */
typedef union __attribute__((packed)) i2c_bmp280_configuration_register_u {
    struct {
        bool                        spi_enabled:1;  /*!< bmp280 3-wire SPI interface enabled when true  (bit:0)  */
        uint8_t                     reserved:1;     /*!< bmp280 reserved                                (bit:1) */
        i2c_bmp280_iir_filters_t    iir_filter:3;   /*!< bmp280 time constant of the IIR filter         (bit:2-4) */
        i2c_bmp280_standby_times_t  standby_time:3; /*!< bmp280 inactive duration in normal mode        (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bmp280_configuration_register_t;


/**
 * @brief BMP280 temperature and pressure calibration factors structure definition.
 */
typedef struct i2c_bmp280_cal_factors_s {
    /* temperature and pressure compensation */
    uint16_t                dig_T1;
    int16_t                 dig_T2;
    int16_t                 dig_T3;
    uint16_t                dig_P1;
    int16_t                 dig_P2;
    int16_t                 dig_P3;
    int16_t                 dig_P4;
    int16_t                 dig_P5;
    int16_t                 dig_P6;
    int16_t                 dig_P7;
    int16_t                 dig_P8;
    int16_t                 dig_P9;
    int32_t                 t_fine;
} i2c_bmp280_cal_factors_t;

/**
 * @brief BMP280 configuration structure definition.
 */
typedef struct i2c_bmp280_config_s {
    uint16_t                                    i2c_address;      /*!< i2c device address */
    uint32_t                                    i2c_clock_speed;  /*!< i2c device scl clock speed  */
    i2c_bmp280_power_modes_t                    power_mode;                 /*!< bmp280 power mode setting */
    i2c_bmp280_iir_filters_t                    iir_filter;                 /*!< bmp280 IIR filter setting */
    i2c_bmp280_pressure_oversampling_t          pressure_oversampling;      /*!< bmp280 pressure oversampling setting */
    i2c_bmp280_temperature_oversampling_t       temperature_oversampling;   /*!< bmp280 temperature oversampling setting */
    i2c_bmp280_standby_times_t                  standby_time;               /*!< bmp280 stand-by time setting */
} i2c_bmp280_config_t;

/**
 * @brief BMP280 context structure.
 */
struct i2c_bmp280_context_t {
    i2c_bmp280_config_t                         dev_config;         /*!< bmp280 device configuration */  
    i2c_master_dev_handle_t                     i2c_handle;         /*!< I2C device handle */
    i2c_bmp280_cal_factors_t                   *dev_cal_factors;    /*!< bmp280 device calibration factors */
    uint8_t                                     sensor_type;        /*!< sensor type, should be bmp280 */
};

/**
 * @brief BMP280 context structure definition.
 */
typedef struct i2c_bmp280_context_t i2c_bmp280_context_t;

/**
 * @brief BMP280 handle structure definition.
 */
typedef struct i2c_bmp280_context_t* i2c_bmp280_handle_t;


/**
 * @brief Reads chip identification register from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] reg BMP280 chip identification register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_chip_id_register(i2c_bmp280_handle_t handle, uint8_t *const reg);

/**
 * @brief Reads status register from BMP280.
 * 
 * @param handle[in] BMP280 device handle.
 * @param[out] reg BMP280 status register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_status_register(i2c_bmp280_handle_t handle, i2c_bmp280_status_register_t *const reg);

/**
 * @brief Reads control measurement register from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] reg BMP280 control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_control_measurement_register(i2c_bmp280_handle_t handle, i2c_bmp280_control_measurement_register_t *const reg);

/**
 * @brief Writes control measurement register to BMP280. 
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] reg BMP280 control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_control_measurement_register(i2c_bmp280_handle_t handle, const i2c_bmp280_control_measurement_register_t reg);

/**
 * @brief Reads configuration register from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] reg BMP280 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_configuration_register(i2c_bmp280_handle_t handle, i2c_bmp280_configuration_register_t *const reg);

/**
 * @brief Writes configuration register to BMP280. 
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] reg BMP280 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_configuration_register(i2c_bmp280_handle_t handle, const i2c_bmp280_configuration_register_t reg);

/**
 * @brief Initializes an BMP280 device onto the master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] bmp280_config BMP280 device configuration.
 * @param[out] bmp280_handle BMP280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_init(i2c_master_bus_handle_t master_handle, const i2c_bmp280_config_t *bmp280_config, i2c_bmp280_handle_t *bmp280_handle);

/**
 * @brief Reads temperature and pressure measurements from BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] pressure Pressure in pascal.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_measurements(i2c_bmp280_handle_t handle, float *const temperature, float *const pressure);

/**
 * @brief Reads temperature measurement from BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_temperature(i2c_bmp280_handle_t handle, float *const temperature);

/**
 * @brief Reads pressure measurement from BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @param[out] pressure Pressure in pascal.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_pressure(i2c_bmp280_handle_t handle, float *const pressure);

/**
 * @brief Reads data status from BMP280.
 * 
 * @param[in] handle bmp280 device handle.
 * @param[out] ready data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_data_status(i2c_bmp280_handle_t handle, bool *const ready);

/**
 * @brief Reads power mode setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] power_mode Power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_power_mode(i2c_bmp280_handle_t handle, i2c_bmp280_power_modes_t *const power_mode);

/**
 * @brief Writes power mode setting to the BMP280.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] power_mode Power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_power_mode(i2c_bmp280_handle_t handle, const i2c_bmp280_power_modes_t power_mode);

/**
 * @brief Reads pressure oversampling setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] oversampling Pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_pressure_oversampling(i2c_bmp280_handle_t handle, i2c_bmp280_pressure_oversampling_t *const oversampling);

/**
 * @brief Writes pressure oversampling setting to BMP280.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] oversampling Pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_pressure_oversampling(i2c_bmp280_handle_t handle, const i2c_bmp280_pressure_oversampling_t oversampling);

/**
 * @brief Reads temperature oversampling setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] oversampling Temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_temperature_oversampling(i2c_bmp280_handle_t handle, i2c_bmp280_temperature_oversampling_t *const oversampling);

/**
 * @brief Writes temperature oversampling setting to BMP280.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] oversampling Temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_temperature_oversampling(i2c_bmp280_handle_t handle, const i2c_bmp280_temperature_oversampling_t oversampling);

/**
 * @brief Reads stand-by time setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] standby_time Stand-by time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_standby_time(i2c_bmp280_handle_t handle, i2c_bmp280_standby_times_t *const standby_time);

/**
 * @brief Writes stand-by time setting to BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] standby_time Stand-by time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_standby_time(i2c_bmp280_handle_t handle, const i2c_bmp280_standby_times_t standby_time);

/**
 * @brief Reads IIR filter setting to BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_get_iir_filter(i2c_bmp280_handle_t handle, i2c_bmp280_iir_filters_t *const iir_filter);

/**
 * @brief Writes IIR filter setting to BMP280.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_set_iir_filter(i2c_bmp280_handle_t handle, const i2c_bmp280_iir_filters_t iir_filter);

/**
 * @brief Issues soft-reset sensor and initializes BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_reset(i2c_bmp280_handle_t handle);

/**
 * @brief Removes an BMP280 device from master bus.
 *
 * @param[in] handle BMP280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_remove(i2c_bmp280_handle_t handle);

/**
 * @brief Removes an BMP280 device from master bus and frees handle.
 *
 * @param[in] handle BMP280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp280_delete(i2c_bmp280_handle_t handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP280_H__
