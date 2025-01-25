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
 * @file ltr390uv.h
 * @defgroup drivers ltr390uv
 * @{
 *
 * ESP-IDF driver for ltr390uv sensor
 * 
 * Source references:
 * https://github.com/esphome/esphome/blob/dev/esphome/components/ltr390/ltr390.cpp
 * https://github.com/DFRobot/DFRobot_LTR390UV/blob/master/DFRobot_LTR390UV.cpp
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LTR390UV_H__
#define __LTR390UV_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * LTR390UV definitions
 */
#define I2C_LTR390UV_SCL_SPEED_HZ           UINT32_C(100000) //!< ltr390uv I2C default clock frequency (100KHz)

#define I2C_LTR390UV_DEV_ADDR               UINT8_C(0x53) //!< ltr390uv I2C address



/*
 * LTR390UV macro definitions
 */
#define I2C_LTR390UV_CONFIG_DEFAULT {                      \
    .dev_config.device_address = I2C_LTR390UV_DEV_ADDR,    \
    .dev_config.scl_speed_hz   = I2C_LTR390UV_SCL_SPEED_HZ,\
    .window_factor             = 1,                        \
    .sensor_resolution         = I2C_LTR390UV_SR_18BIT,    \
    .measurement_rate          = I2C_LTR390UV_MR_100MS,    \
    .measurement_gain          = I2C_LTR390UV_MG_X3,       }


/*
* LTR390UV enumerator and sructure declerations
*/


/**
 * @brief LTR390UV I2C operation modes enumerator.
 */
typedef enum {
    I2C_LTR390UV_OM_ALS = 0,
    I2C_LTR390UV_OM_UVS = 1
} i2c_ltr390uv_operation_modes_t;

/**
 * @brief LTR390UV I2C sensor resolutions enumerator.
 */
typedef enum {
    I2C_LTR390UV_SR_20BIT = (0b000),    /*!< ltr390uv 20-bit resolution, conversion time = 400ms */
    I2C_LTR390UV_SR_19BIT = (0b001),    /*!< ltr390uv 19-bit resolution, conversion time = 200ms */
    I2C_LTR390UV_SR_18BIT = (0b010),    /*!< ltr390uv 18-bit resolution, conversion time = 100ms (default) */
    I2C_LTR390UV_SR_17BIT = (0b011),    /*!< ltr390uv 17-bit resolution, conversion time = 50ms */
    I2C_LTR390UV_SR_16BIT = (0b100),    /*!< ltr390uv 16-bit resolution, conversion time = 25ms */
    I2C_LTR390UV_SR_13BIT = (0b101),    /*!< ltr390uv 13-bit resolution, conversion time = 12.5ms */
} i2c_ltr390uv_sensor_resolutions_t;

/**
 * @brief LTR390UV I2C measurement rates enumerator.
 */
typedef enum {
    I2C_LTR390UV_MR_25MS    = (0b000),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_50MS    = (0b001),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_100MS   = (0b010),   /*!< ltr390uv  (default) */
    I2C_LTR390UV_MR_200MS   = (0b011),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_500MS   = (0b100),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_1000MS  = (0b101),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_2000MS  = (0b110),   /*!< ltr390uv  */
    I2C_LTR390UV_MR_2000_MS = (0b111),   /*!< ltr390uv  */
} i2c_ltr390uv_measurement_rates_t;

/**
 * @brief LTR390UV I2C measurement gains enumerator.
 */
typedef enum {
    I2C_LTR390UV_MG_X1  = (0b000),    /*!< ltr390uv  */
    I2C_LTR390UV_MG_X3  = (0b001),    /*!< ltr390uv  (default) */
    I2C_LTR390UV_MG_X6  = (0b010),    /*!< ltr390uv   */
    I2C_LTR390UV_MG_X9  = (0b011),    /*!< ltr390uv  */
    I2C_LTR390UV_MG_X18 = (0b100),    /*!< ltr390uv  */
} i2c_ltr390uv_measurement_gains_t;

/**
 * @brief LTR390UV I2C light source interrupts enumerator.
 */
typedef enum {
    I2C_LTR390UV_LSI_ALS = (0b01),
    I2C_LTR390UV_LSI_UVS = (0b11)
} i2c_ltr390uv_ls_interrupts_t;

/**
 * @brief LTR390UV I2C main control register (0x00 read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t                         reserved1:1;                /*!< reserved                       (bit:0)  */
        bool                            sensor_enabled:1;           /*!< ltr390uv light sensor (ALS/UVS) standby when false and active when true (bit:1) */
        uint8_t                         reserved2:1;                /*!< reserved                       (bit:2)  */
        i2c_ltr390uv_operation_modes_t  operation_mode:1;           /*!< ltr390uv operation mode (ALS or UVS) (bit:3) */
        bool                            software_reset_enabled:1;   /*!< ltr390uv software reset triggered when true (bit:4) */
        uint8_t                         reserved3:3;                /*!< reserved                       (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_ltr390uv_control_register_t;

/**
 * @brief LTR390UV I2C ALS UVS measurement register (0x04 read-write | POR State 0x22) structure.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_ltr390uv_measurement_rates_t    measurement_rate:3;     /*!<            (bit:0-2)  */
        uint8_t                             reserved1:1;            /*!< reserved   (bit:3) */
        i2c_ltr390uv_sensor_resolutions_t   sensor_resolution:3;    /*!< ltr390uv   (bit:4-6) */
        uint8_t                             reserved2:1;            /*!< reserved   (bit:7) */
    } bits;
    uint8_t reg;
} i2c_ltr390uv_measure_register_t;

/**
 * @brief LTR390UV I2C ALS UVS gain register (0x04 read-write | POR State 0x01) structure.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_ltr390uv_measurement_gains_t    measurement_gain:3;     /*!< ltr390uv   (bit:0-2) */
        uint8_t                             reserved:5;             /*!< reserved   (bit:4-6) */
    } bits;
    uint8_t reg;
} i2c_ltr390uv_gain_register_t;

/**
 * @brief LTR390UV I2C main status register (0x07 read-only | POR State 0x20) structure.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:3;    /*!< reserved   (bit:0-2)  */
        bool    data_status:1;  /*!< ltr390uv   (bit:3) */
        bool    irq_status:1;   /*!< ltr390uv   (bit:4) */
        bool    power_status:1; /*!< ltr390uv   (bit:5) */
        uint8_t reserved2:2;    /*!< reserved   (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_ltr390uv_status_register_t;

/**
 * @brief LTR390UV I2C interrupt configuration register (0x19 read-write | POR State 0x10) structure.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t                         reserved1:2;        /*!< reserved   (bit:0-1)  */
        bool                            irq_enabled:1;      /*!< ltr390uv   (bit:2) */
        uint8_t                         reserved2:1;        /*!< reserved   (bit:3) */
        i2c_ltr390uv_ls_interrupts_t    irq_light_source:2; /*!< ltr390uv   (bit:4-5) */
        uint8_t                         reserved3:2;        /*!< reserved   (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_ltr390uv_interrupt_configuration_register_t;


/**
 * @brief LTR390UV I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t                 dev_config;          /*!< I2C configuration for ltr390uv device */
    uint8_t                             window_factor;       /*!< ltr390uv window factor wfac = 1 for no window or clear window glass, wfac > 1 when device is under tinted window glass, calibrate under white LED */
    i2c_ltr390uv_sensor_resolutions_t   sensor_resolution;   /*!< ltr390uv sensor resolution */
    i2c_ltr390uv_measurement_rates_t    measurement_rate;    /*!< ltr390uv measurement rate */
    i2c_ltr390uv_measurement_gains_t    measurement_gain;    /*!< ltr390uv measurement gain */
} i2c_ltr390uv_config_t;

/**
 * @brief LTR390UV I2C device structure.
 */
struct i2c_ltr390uv_t {
    i2c_master_dev_handle_t                         i2c_dev_handle;         /*!< I2C device handle */
    uint8_t                                         window_factor;          /*!< ltr390uv window factor wfac = 1 for no window or clear window glass, wfac > 1 when device is under tinted window glass, calibrate under white LED */
    i2c_ltr390uv_sensor_resolutions_t               sensor_resolution;      /*!< ltr390uv sensor resolution */
    i2c_ltr390uv_measurement_rates_t                measurement_rate;       /*!< ltr390uv measurement rate */
    i2c_ltr390uv_measurement_gains_t                measurement_gain;       /*!< ltr390uv measurement gain */
    i2c_ltr390uv_control_register_t                 control_reg;            /*!< ltr390uv main control register */
    i2c_ltr390uv_measure_register_t                 measure_reg;            /*!< ltr390uv ALS UVS measure register */
    i2c_ltr390uv_gain_register_t                    gain_reg;               /*!< ltr390uv ALS UVS gain register */
    i2c_ltr390uv_interrupt_configuration_register_t irq_config_reg;         /*!< ltr390uv interrupt configuration register */
    i2c_ltr390uv_status_register_t                  status_reg;             /*!< ltr390uv main status register */
};

/**
 * @brief LTR390UV I2C device structure definition.
 */
typedef struct i2c_ltr390uv_t i2c_ltr390uv_t;

/**
 * @brief LTR390UV I2C device handle definition.
 */
typedef struct i2c_ltr390uv_t *i2c_ltr390uv_handle_t;



/**
 * @brief Reads control register from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_control_register(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Writes control register to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param control_reg LTR390UV control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_control_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_control_register_t control_reg);

/**
 * @brief Reads ALS UVS measure register from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_measure_register(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Writes ALS UVS measure register to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param measure_reg LTR390UV ALS UVS measure register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_measure_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measure_register_t measure_reg);

/**
 * @brief Reads ALS UVS gain register from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_gain_register(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Writes ALS UVS gain register to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param gain_reg LTR390UV ALS UVS gain register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_gain_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_gain_register_t gain_reg);

/**
 * @brief Reads interrupt configuration register from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_interrupt_configuration_register(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Writes interrupt configuration register to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param interrupt_config_reg LTR390UV interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_interrupt_configuration_register(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_interrupt_configuration_register_t interrupt_config_reg);

/**
 * @brief Reads status register from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_status_register(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Initializes an LTR390UV device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ltr390uv_config Configuration of LTR390UV device.
 * @param[out] ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_init(i2c_master_bus_handle_t bus_handle, const i2c_ltr390uv_config_t *ltr390uv_config, i2c_ltr390uv_handle_t *ltr390uv_handle);

/**
 * @brief Reads ambient light from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param ambient_light Ambient light in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_ambient_light(i2c_ltr390uv_handle_t ltr390uv_handle, float *const ambient_light);

/**
 * @brief Reads ALS sensor counts from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param sensor_counts Light.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_als(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const sensor_counts);

/**
 * @brief Reads ultraviolet index (UVI) from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param ultraviolet_index Ultraviolet index (UVI).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_ultraviolet_index(i2c_ltr390uv_handle_t ltr390uv_handle, float *const ultraviolet_index);

/**
 * @brief Reads UVS sensor counts from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param sensor_counts Ultraviolet light in mW/cm^2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_uvs(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const sensor_counts);


/**
 * @brief Reads data ready status flag from LTR390UV.  This flag is cleared after the register is read.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param[out] ready LTR390UV data is new and ready to read when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_data_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const ready);

/**
 * @brief Reads power status flag from LTR390UV.  This flag is cleared after the register is read.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param[out] power_on LTR390UV is power on event when true and all interrupt threshold settings in the registers have been reset to power on default state.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_power_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const power_on);

/**
 * @brief Reads interrupt status flag from LTR390UV.  This flag is cleared after the register is read.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param[out] interrupt LTR390UV interrupt is active when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_interrupt_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const interrupt);

/**
 * @brief Reads interrupt status flags from LTR390UV.  The flags are cleared after the register is read.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param data_ready LTR390UV data is new and ready to read when true.
 * @param power_on LTR390UV is power on event when true and all interrupt threshold settings in the registers have been reset to power on default state.
 * @param interrupt LTR390UV interrupt is active when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_status(i2c_ltr390uv_handle_t ltr390uv_handle, bool *const data_ready, bool *const power_on, bool *const interrupt);

/**
 * @brief Reads UVS/ALS lower and upper thresholds from LTR390UV.  The thresholds are used to trigger an interrupt when the light level exceeds the upper threshold or falls below the lower threshold.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param lower_threshold LTR390UV lower threshold in lux or mW/cm^2 setting.
 * @param uppper_threshold LTR390UV upper threshold in lux or mW/cm^2 setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_thresholds(i2c_ltr390uv_handle_t ltr390uv_handle, uint32_t *const lower_threshold, uint32_t *const uppper_threshold);

/**
 * @brief Writes UVS/ALS lower and upper thresholds to LTR390UV.  The thresholds are used to trigger an interrupt when the light level exceeds the upper threshold or falls below the lower threshold.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param lower_threshold Lower threshold in lux or mW/cm^2 setting.
 * @param uppper_threshold Upper threshold in lux or mW/cm^2 setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_thresholds(i2c_ltr390uv_handle_t ltr390uv_handle, const uint32_t lower_threshold, const uint32_t uppper_threshold);

/**
 * @brief Reads operation mode from LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param mode LTR390UV operation mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_mode(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_operation_modes_t *const mode);

/**
 * @brief Writes operation mode to LTR390UV.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param mode LTR390UV operation mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_mode(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_operation_modes_t mode);

/**
 * @brief Reads sensor resolution from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param resolution LTR390UV sensor resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_resolution(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_sensor_resolutions_t *const resolution);

/**
 * @brief Writes sensor resolution to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param resolution LTR390UV sensor resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_resolution(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_sensor_resolutions_t resolution);

/**
 * @brief Reads measurement gain from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param gain LTR390UV measurement gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_gain(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_measurement_gains_t *const gain);

/**
 * @brief Writes measurement gain to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param gain LTR390UV measurement gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_gain(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measurement_gains_t gain);

/**
 * @brief Reads measurement rate from LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param rate LTR390UV measurement rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_get_rate(i2c_ltr390uv_handle_t ltr390uv_handle, i2c_ltr390uv_measurement_rates_t *const rate);

/**
 * @brief Writes measurement rate to LTR390UV.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @param rate LTR390UV measurement rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_set_rate(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_measurement_rates_t rate);

/**
 * @brief Enables LTR390UV interrupts.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @param light_source LTR390UV interrupt light source (e.g. ALS or UVS).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_enable_interrupt(i2c_ltr390uv_handle_t ltr390uv_handle, const i2c_ltr390uv_ls_interrupts_t light_source);

/**
 * @brief Disables LTR390UV interrupts.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_disable_interrupt(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Activates LTR390UV for measurements. 
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_enable(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Places LTR390UV on standby (default).
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_disable(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Issues soft-reset and initializes LTR390UV.  See datasheet for details.
 *
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_reset(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Removes an LTR390UV device from master bus.
 *
 * @param[in] ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_remove(i2c_ltr390uv_handle_t ltr390uv_handle);

/**
 * @brief Removes an LTR390UV device from master bus and frees handle.
 * 
 * @param ltr390uv_handle LTR390UV device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ltr390uv_delete(i2c_ltr390uv_handle_t ltr390uv_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __LTR390UV_H__
