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
 * @file ina226.h
 * @defgroup drivers ina226
 * @{
 *
 * ESP-IDF driver for ina226 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __INA226_H__
#define __INA226_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * INA226 definitions
*/

#define I2C_INA226_DEV_CLK_SPD         UINT32_C(100000)    //!< ina226 I2C default clock frequency (100KHz)
#define I2C_INA226_XFR_TIMEOUT_MS       UINT16_C(500)        //!< ina226 I2C transaction timeout in milliseconds

#define I2C_INA226_ADDR_GND_GND         UINT8_C(0x40)       //!< ina226 I2C address, A1 pin - GND, A0 pin - GND
#define I2C_INA226_ADDR_GND_VS          UINT8_C(0x41)       //!< ina226 I2C address, A1 pin - GND, A0 pin - VS+
#define I2C_INA226_ADDR_GND_SDA         UINT8_C(0x42)       //!< ina226 I2C address, A1 pin - GND, A0 pin - SDA
#define I2C_INA226_ADDR_GND_SCL         UINT8_C(0x43)       //!< ina226 I2C address, A1 pin - GND, A0 pin - SCL
#define I2C_INA226_ADDR_VS_GND          UINT8_C(0x44)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - GND
#define I2C_INA226_ADDR_VS_VS           UINT8_C(0x45)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - VS+
#define I2C_INA226_ADDR_VS_SDA          UINT8_C(0x46)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - SDA
#define I2C_INA226_ADDR_VS_SCL          UINT8_C(0x47)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - SCL
#define I2C_INA226_ADDR_SDA_GND         UINT8_C(0x48)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - GND
#define I2C_INA226_ADDR_SDA_VS          UINT8_C(0x49)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - VS+
#define I2C_INA226_ADDR_SDA_SDA         UINT8_C(0x4a)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - SDA
#define I2C_INA226_ADDR_SDA_SCL         UINT8_C(0x4b)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - SCL
#define I2C_INA226_ADDR_SCL_GND         UINT8_C(0x4c)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - GND
#define I2C_INA226_ADDR_SCL_VS          UINT8_C(0x4d)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - VS+
#define I2C_INA226_ADDR_SCL_SDA         UINT8_C(0x4e)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - SDA
#define I2C_INA226_ADDR_SCL_SCL         UINT8_C(0x4f)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - SCL

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * INA226 macro definitions
*/
#define I2C_INA226_CONFIG_DEFAULT {                                     \
    .i2c_address                = I2C_INA226_ADDR_GND_GND,              \
    .i2c_clock_speed            = I2C_INA226_DEV_CLK_SPD,               \
    .averaging_mode             = INA226_AVG_MODE_1,                    \
    .shunt_voltage_conv_time    = INA226_VOLT_CONV_TIME_1_1MS,          \
    .bus_voltage_conv_time      = INA226_VOLT_CONV_TIME_1_1MS,          \
    .mode                       = INA226_OP_MODE_CONT_SHUNT_BUS,        \
    .shunt_resistance           = 0.0005,                               \
    .max_current                = 10                                    \
    }
                                /* rshunt is 0.5 milli ohms */
                                /* up to max 10 amps */


/*
 * INA226 enumerator and structure declarations
*/

/**
 * bus voltage range
 */
typedef enum ina226_bus_voltage_ranges_e {
    INA226_BUS_VOLT_RANGE_16V = 0, //!< 16V FSR
    INA226_BUS_VOLT_RANGE_32V      //!< 32V FSR (default)
} ina226_bus_voltage_ranges_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum ina226_gains_e {
    INA226_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA226_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA226_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA226_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina226_gains_t;

/**
 * ADC resolution/averaging
 */
typedef enum ina226_resolutions_e {
    INA226_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA226_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA226_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA226_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA226_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA226_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA226_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA226_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA226_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA226_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA226_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina226_resolutions_t;

typedef enum ina226_reset_states_e {
    INA226_RESET_STATE_DISABLED = 0,
    INA226_RESET_STATE_ENABLED  = 1
} ina226_reset_states_t;

typedef enum ina226_averaging_modes_e {
    INA226_AVG_MODE_1       = (0b000),  /*!< default */
    INA226_AVG_MODE_4       = (0b001),
    INA226_AVG_MODE_16      = (0b010),
    INA226_AVG_MODE_64      = (0b011),
    INA226_AVG_MODE_128     = (0b100),
    INA226_AVG_MODE_256     = (0b101),
    INA226_AVG_MODE_512     = (0b110),
    INA226_AVG_MODE_1024    = (0b111)
} ina226_averaging_modes_t;

typedef enum ina226_volt_conv_times_e {
    INA226_VOLT_CONV_TIME_140US     = (0b000),
    INA226_VOLT_CONV_TIME_204US     = (0b001),
    INA226_VOLT_CONV_TIME_332US     = (0b010),
    INA226_VOLT_CONV_TIME_588US     = (0b011),
    INA226_VOLT_CONV_TIME_1_1MS     = (0b100),  /*!< 1.1 ms default */
    INA226_VOLT_CONV_TIME_2_116MS   = (0b101),
    INA226_VOLT_CONV_TIME_4_156MS   = (0b110),
    INA226_VOLT_CONV_TIME_8_244MS   = (0b111)
} ina226_volt_conv_times_t;

typedef enum ina226_operating_modes_e {
    INA226_OP_MODE_SHUTDOWN         = (0b000),  /*!< device is powered down */
    INA226_OP_MODE_TRIG_SHUNT_VOLT  = (0b001),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_TRIG_BUS_VOLT    = (0b010),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_TRIG_SHUNT_BUS   = (0b011),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_SHUTDOWN2        = (0b100),  /*!< device is powered down */
    INA226_OP_MODE_CONT_SHUNT_VOLT  = (0b101),  /*!< normal operating mode */
    INA226_OP_MODE_CONT_BUS_VOLT    = (0b110),  /*!< normal operating mode */
    INA226_OP_MODE_CONT_SHUNT_BUS   = (0b111)   /*!< normal operating mode default */
} ina226_operating_modes_t;

typedef union __attribute__((packed)) ina226_config_register_u {
    struct {
        ina226_operating_modes_t    mode:3; 
        ina226_volt_conv_times_t    shun_volt_conv_time:3;
        ina226_volt_conv_times_t    bus_volt_conv_time:3;
        ina226_averaging_modes_t    avg_mode:3;
        uint16_t                    reserved:3;        
        bool                        reset_enabled:1;         /*!< reset state BIT15 */
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina226_config_register_t;

typedef union __attribute__((packed)) ina226_mask_enable_register_u {
    struct {
        bool        alert_latch_enable:1;       /*!<  (bit:0) */
        bool        alert_polarity_bit:1;       /*!<  (bit:1) */
        bool        math_overflow_flag:1;       /*!<  (bit:2) */
        bool        conversion_ready_flag:1;    /*!<  (bit:3) */
        bool        alert_func_flag:1;          /*!<  (bit:4) */
        uint16_t    reserved:5;                 /*!<  (bit:5-9) */
        bool        conversion_ready:1;         /*!<  (bit:10) */
        bool        power_over_limit:1;         /*!<  (bit:11) */
        bool        bus_volt_under_volt:1;      /*!<  (bit:12) */  
        bool        bus_volt_over_volt:1;       /*!<  (bit:13) */
        bool        shunt_volt_under_volt:1;    /*!<  (bit:14) */  
        bool        shunt_volt_over_volt:1;     /*!<  (bit:15) */
    } bits;                  /*!< represents the 16-bit mask/enable register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit mask/enable register as `uint16_t` */
} ina226_mask_enable_register_t;


/**
 * @brief i2c ina226 device configuration.
 */
typedef struct ina226_config_s {
    uint16_t                        i2c_address;        /*!< ina226 i2c device address */
    uint32_t                        i2c_clock_speed;    /*!< ina226 i2c device scl clock speed  */
    ina226_averaging_modes_t        averaging_mode;           /*!< averaging mode */
    ina226_volt_conv_times_t        shunt_voltage_conv_time;
    ina226_volt_conv_times_t        bus_voltage_conv_time;
    ina226_operating_modes_t        mode;               /*!< operating mode */
    float                           shunt_resistance;
    float                           max_current;
} ina226_config_t;


/**
 * @brief INA226 context structure.
 */
struct ina226_context_t {
    ina226_config_t             dev_config;       /*!< ina226 device configuration */
    i2c_master_dev_handle_t     i2c_handle;  /*!< ina226 I2C device handle */
    float                       current_lsb;
    float                       power_lsb;
};

typedef struct ina226_context_t ina226_context_t;
typedef struct ina226_context_t *ina226_handle_t;


esp_err_t ina226_get_configuration_register(ina226_handle_t handle, ina226_config_register_t *const reg);
esp_err_t ina226_set_configuration_register(ina226_handle_t handle, const ina226_config_register_t reg);

esp_err_t ina226_get_calibration_register(ina226_handle_t handle, uint16_t *const reg);
esp_err_t ina226_set_calibration_register(ina226_handle_t handle, const uint16_t reg);

/**
 * @brief initializes an INA226 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle
 * @param[in] ina226_config INA226 device configuration.
 * @param[out] ina226_handle INA226 device handle.
 * @return ESP_OK on success.
 */
esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle);

/**
 * @brief Reset device
 *
 * Same as power-on reset. Resets all registers to default values.
 * Calibration is required for the device to read current, otherwise
 * only shunt voltage readings will be valid.
 *
 * @param[in] handle INA226 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina226_reset(ina226_handle_t handle);

/**
 * @brief Get operating mode
 *
 * @param[in] handle INA226 device handle
 * @param[out] mode Operating mode
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_mode(ina226_handle_t handle, ina226_operating_modes_t *const mode);

/**
 * @brief Set operating mode
 *
 * @param[in] handle INA226 device handle
 * @param[out] mode Operating mode
 * @return ESP_OK on success.
 */
esp_err_t ina226_set_mode(ina226_handle_t handle, const ina226_operating_modes_t mode);

/**
 * @brief Perform calibration
 *
 * Current readings will be valid only after calibration
 *
 * @param[in] handle INA226 device handle
 * @param[in] max_current maximum expected current, A
 * @param[in] shunt_resistance shunt resistance, Ohm
 * @return ESP_OK on success.
 */
esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance);

/**
 * @brief Read bus voltage
 *
 * @param[in] handle INA226 device handle
 * @param[out] voltage bus voltage, V
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Read shunt voltage
 *
 * @param[in] handle INA226 device handle
 * @param[out] voltage shunt voltage, V
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Read current
 *
 * This function works properly only after calibration.
 *
 * @param[in] handle INA226 device handle
 * @param[out] current current, A
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_current(ina226_handle_t handle, float *const current);

/**
 * @brief Read power
 *
 * This function works properly only after calibration.
 *
 * @param[in] handle INA226 device handle
 * @param[out] power power, W
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_power(ina226_handle_t handle, float *const power);


/**
 * @brief Removes an INA226 device from master bus.
 *
 * @param[in] handle INA226 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina226_remove(ina226_handle_t handle);

/**
 * @brief Removes an INA226 device from master bus and frees handle.
 * 
 * @param[in] handle INA226 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_delete(ina226_handle_t handle);

/**
 * @brief Converts INA226 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* INA226 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ina226_get_fw_version(void);

/**
 * @brief Converts INA226 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t INA226 firmware version number.
 */
int32_t ina226_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __INA226_H__
