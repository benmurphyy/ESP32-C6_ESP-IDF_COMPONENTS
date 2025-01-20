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
 * @file as3935.h
 * @defgroup drivers as3935
 * @{
 *
 * ESP-IDF driver for as3935 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AS3935_H__
#define __AS3935_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_types.h>
#include <esp_event.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AS3935 definitions
*/
/* AS3935 I2C constants */
#define I2C_AS3935_SCL_SPEED_HZ          UINT32_C(100000)       //!< as3935 I2C default clock frequency (100KHz)

/* AS3935 I2C addresses */
#define I2C_AS3935_DEV_ADDR_1            UINT8_C(0x01)   //!< as3935 I2C address when ADD1 pin floating/low and ADD0 pin high
#define I2C_AS3935_DEV_ADDR_2            UINT8_C(0x02)   //!< as3935 I2C address when ADD0 pin floating/low and ADD1 pin high
#define I2C_AS3935_DEV_ADDR_3            UINT8_C(0x03)   //!< as3935 I2C address when ADD0 and ADD1 pins high

/* AS3935 I2C registers */
#define I2C_AS3935_REG_00                UINT8_C(0x00)   //!< as3935 I2C register to access AFE gain boost and power-down
#define I2C_AS3935_REG_01                UINT8_C(0x01)   //!< as3935 I2C register to access noise floor level and watchdog threshold (0-15)
#define I2C_AS3935_REG_02                UINT8_C(0x02)   //!< as3935 I2C register to access clearing statistics, minimum number of lightning, and spike rejection
#define I2C_AS3935_REG_03                UINT8_C(0x03)   //!< as3935 I2C register to access frequency division ratio and mask disturber
#define I2C_AS3935_REG_04                UINT8_C(0x04)   //!< as3935 I2C register to access energy of lightning (LSBYTE)
#define I2C_AS3935_REG_05                UINT8_C(0x05)   //!< as3935 I2C register to access energy of lightning (MSBYTE)
#define I2C_AS3935_REG_06                UINT8_C(0x06)   //!< as3935 I2C register to access energy of lightning (MMSBYTE)
#define I2C_AS3935_REG_07                UINT8_C(0x07)   //!< as3935 I2C register to access distance estimation of lightning
#define I2C_AS3935_REG_08                UINT8_C(0x08)   //!< as3935 I2C register to access internal tuning caps and display (LCO, SRCO, TRCO) on IRQ pin
#define I2C_AS3935_REG_RST               UINT8_C(0x96)   //!< as3935 I2C register to either calibrate or reset registers to default
/* AS3935 I2C command */
#define I2C_AS3935_CMD_PRESET_DEFAULT    UINT8_C(0x3c)   //!< as3935 I2C command to set all registers in default mode
#define I2C_AS3935_CMD_CALIB_RCO         UINT8_C(0x3d)   //!< as3935 I2C command to automatically calibrate the internal RC oscillators

#define I2C_AS3935_POWERUP_DELAY_MS      (25)
#define I2C_AS3935_APPSTART_DELAY_MS     (25)
#define I2C_AS3935_STARTUP_DELAY_MS      (2)            //!< as3935 I2C LCO start-up delay in milliseconds
#define I2C_AS3935_INTERRUPT_DELAY_MS    (2)            //!< as3935 I2C interrupt delay in milliseconds
#define I2C_AS3935_CALIBRATION_DELAY_MS  (2)            //!< as3935 I2C calibration delay in milliseconds for RC oscillators

/**
 * @brief declare of AS3935 monitor event base.
 */
ESP_EVENT_DECLARE_BASE(ESP_AS3935_EVENT);

/**
 * @brief CJMCU-3935 Board Wiring (I2C interface):
 *  ADD0 & ADD1 pin to VCC (3.3v) -> I2C address 0x03
 *  SI pin to VCC (3.3v) -> enable I2C interface
 *  SCL pin 10k-ohm pull-up resistor to VCC (3.3v)
 *  SCL pin to MCU SCL pin
 *  MOSI (SDA) pin to MCU SDA pin
 *  IRQ pin to MCU interrupt pin
 *  VCC pin to 3.3v
 *  GND pin to common ground
*/

/**
 * @brief AS3935 macro definitions
*/
#define I2C_AS3935_CONFIG_DEFAULT {                                 \  
        .dev_config.scl_speed_hz      = I2C_AS3935_SCL_SPEED_HZ,    \                                       
        .dev_config.device_address    = I2C_AS3935_DEV_ADDR_1,      \
        .irq_io_enabled               = true,                       \
        .irq_io_num                   = GPIO_NUM_10, }


/**
 * @brief AS3935 monitor handle type definition
 *
 */
typedef void *as3935_monitor_handle_t;

/**
 * @brief AS3935 enumerator and sructure declerations
*/

/**
 * @brief AS3935 analog frontends (AFE) REG0x00[5:1] enumerator.
 */
typedef enum {
    I2C_AS3935_AFE_INDOOR  = (0b10010),
    I2C_AS3935_AFE_OUTDOOR = (0b01110)
} i2c_as3935_analog_frontends_t;

/**
 * @brief AS3935 power states REG0x00[0] enumerator.
 */
typedef enum {
    I2C_AS3935_POWER_OFF  = 1,
    I2C_AS3935_POWER_ON   = 0
} i2c_as3935_power_states_t;

/**
 * @brief AS3935 clear statistics states REG0x02[6] enumerator.
 */
typedef enum {
    I2C_AS3935_CLEAR_STATS_DISABLED  = 1,
    I2C_AS3935_CLEAR_STATS_ENABLED   = 0
} i2c_as3935_clear_statistics_states_t;

/**
 * @brief AS3935 disturber detecton states REG0x03[5] enumerator.
 */
typedef enum {
    I2C_AS3935_DISTURBER_DETECTION_ENABLED  = 0,
    I2C_AS3935_DISTURBER_DETECTION_DISABLED = 1
} i2c_as3935_disturber_detection_states_t;

/**
 * @brief AS3935 CO IRQ pin states REG0x08[5]|[6]|[7] enumerator.
 */
typedef enum {
    I2C_AS3935_CO_IRQ_PIN_ENABLED   = 1,
    I2C_AS3935_CO_IRQ_PIN_DISABLED  = 0
} i2c_as3935_co_irq_pin_states_t;

/**
* @brief AS3935 watchdog thresholds REG0x01[3:0] enumerator.
*/
typedef enum {
    I2C_AS3935_WD_THRESHOLD_0  = (0b0000),
    I2C_AS3935_WD_THRESHOLD_1  = (0b0001),
    I2C_AS3935_WD_THRESHOLD_2  = (0b0010),
    I2C_AS3935_WD_THRESHOLD_3  = (0b0011),
    I2C_AS3935_WD_THRESHOLD_4  = (0b0100),
    I2C_AS3935_WD_THRESHOLD_5  = (0b0101),
    I2C_AS3935_WD_THRESHOLD_6  = (0b0110),
    I2C_AS3935_WD_THRESHOLD_7  = (0b0111),
    I2C_AS3935_WD_THRESHOLD_8  = (0b1000),
    I2C_AS3935_WD_THRESHOLD_9  = (0b1001),
    I2C_AS3935_WD_THRESHOLD_10 = (0b1010)
} i2c_as3935_watchdog_thresholds_t;

/**
* @brief AS3935 noise floor generator and evaluation REG0x01[6:4] enumerator.
*/
typedef enum {
    I2C_AS3935_NOISE_LEVEL_390_28   = (0b000),
    I2C_AS3935_NOISE_LEVEL_630_45   = (0b001),
    I2C_AS3935_NOISE_LEVEL_860_62   = (0b010),
    I2C_AS3935_NOISE_LEVEL_1100_78  = (0b011),
    I2C_AS3935_NOISE_LEVEL_1140_95  = (0b100),
    I2C_AS3935_NOISE_LEVEL_1570_112 = (0b101),
    I2C_AS3935_NOISE_LEVEL_1800_130 = (0b110),
    I2C_AS3935_NOISE_LEVEL_2000_146 = (0b111)
} i2c_as3935_noise_levels_t;

/**
* @brief AS3935 interrupt states REG0x03[3:0] enumerator.
*/
typedef enum {
    I2C_AS3935_INT_NOISE     = (0b0001),
    I2C_AS3935_INT_DISTURBER = (0b0100),
    I2C_AS3935_INT_LIGHTNING = (0b1000),
    I2C_AS3935_INT_NONE      = (0b0000)
} i2c_as3935_interrupt_states_t;

/**
* @brief AS3935 minimum number of lightning detections REG0x02[5:4] enumerator.
*/
typedef enum {
    I2C_AS3935_MIN_LIGHTNING_1  = (0b00),
    I2C_AS3935_MIN_LIGHTNING_5  = (0b01),
    I2C_AS3935_MIN_LIGHTNING_9  = (0b10),
    I2C_AS3935_MIN_LIGHTNING_16 = (0b11)
} i2c_as3935_minimum_lightnings_t;

/**
* @brief AS3935 frequency division ratio for antenna tunning REG0x03[7:3] LCO_FDIV enumerator.
*/
typedef enum {
    I2C_AS3935_FREQ_DIV_RATIO_16  = (0b00),
    I2C_AS3935_FREQ_DIV_RATIO_32  = (0b01),
    I2C_AS3935_FREQ_DIV_RATIO_64  = (0b10),
    I2C_AS3935_FREQ_DIV_RATIO_128 = (0b11)
} i2c_as3935_frequency_division_ratios_t;

/**
* @brief AS3935 lightning estimated distances REG0x07[5:0] enumerator.
*/
typedef enum {
    I2C_AS3935_L_DISTANCE_OVERHEAD  = (0b000001),
    I2C_AS3935_L_DISTANCE_5KM       = (0b000101),
    I2C_AS3935_L_DISTANCE_6KM       = (0b000110),
    I2C_AS3935_L_DISTANCE_8KM       = (0b001000),
    I2C_AS3935_L_DISTANCE_10KM      = (0b001010),
    I2C_AS3935_L_DISTANCE_12KM      = (0b001100),
    I2C_AS3935_L_DISTANCE_14KM      = (0b001110),
    I2C_AS3935_L_DISTANCE_17KM      = (0b010001),
    I2C_AS3935_L_DISTANCE_20KM      = (0b010100),
    I2C_AS3935_L_DISTANCE_24KM      = (0b011000),
    I2C_AS3935_L_DISTANCE_27KM      = (0b011011),
    I2C_AS3935_L_DISTANCE_31KM      = (0b011111),
    I2C_AS3935_L_DISTANCE_34KM      = (0b100010),
    I2C_AS3935_L_DISTANCE_37KM      = (0b100101),
    I2C_AS3935_L_DISTANCE_40KM      = (0b101000),
    I2C_AS3935_L_DISTANCE_OO_RANGE  = (0b111111)
} i2c_as3935_lightning_distances_t;

/**
 * @brief AS3935 oscillator calibration status results enumerator.
 */
typedef enum {
    I2C_AS3935_RCO_CALIBRATION_SUCCESSFUL,
    I2C_AS3935_RCO_CALIBRATION_UNSUCCESSFUL,
    I2C_AS3935_RCO_CALIBRATION_INCOMPLETE,
} i2c_as3935_rco_calibration_results_t;

/**
 * @brief AS3935 tuning mode oscillators enumerator.
 */
typedef enum {
    I2C_AS3935_OSCILLATOR_ANTENNA_LC,
    I2C_AS3935_OSCILLATOR_TIMER_RC,
    I2C_AS3935_OSCILLATOR_SYSTEM_RC,
} i2c_as3935_oscillator_modes_t;

/**
 * @brief AS3935 register 0x00 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x00_BIT_TAG {
        i2c_as3935_power_states_t       power_state:1;  /*!< power on or off                        (bit:0)   */
        i2c_as3935_analog_frontends_t   analog_frontend:5; /*!< analog front-end (AFE) and watchdog (bit:1-5) */
        uint8_t                         reserved:2;     /*!< reserved and set to 0                  (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x00_register_t;

/**
 * @brief AS3935 register 0x01 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x01_BIT_TAG {
        i2c_as3935_watchdog_thresholds_t watchdog_threshold:4; /*!< watchdog threhold  (bit:0-3) */
        i2c_as3935_noise_levels_t noise_floor_level:3;   /*!< noise floor level        (bit:4-6) */
        uint8_t         reserved:1;             /*!< reserved and set to 0             (bit:7)   */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x01_register_t;

/**
 * @brief AS3935 register 0x02 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x02_BIT_TAG {
        uint8_t                     spike_rejection:4;      /*!< spike rejection                (bit:0-3) */
        i2c_as3935_minimum_lightnings_t min_num_lightning:2; /*!< minimum number of lightning   (bit:4-5) */
        i2c_as3935_clear_statistics_states_t clear_stats_state:1; /*!< clear statistics         (bit:6)   */
        uint8_t                     reserved:1;             /*!< reserved and set to 0          (bit:7)   */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x02_register_t;

/**
 * @brief AS3935 register 0x03 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x03_BIT_TAG {
        i2c_as3935_interrupt_states_t           irq_state:4;    /*!< lightning event interrupt                  (bit:0-3) */
        uint8_t                                 reserved:1;     /*!< reserved and set to 0                      (bit:4)   */
        i2c_as3935_disturber_detection_states_t disturber_detection_state:1;    /*!< disturber detection state  (bit:5)   */
        i2c_as3935_frequency_division_ratios_t  freq_div_ratio:2;   /*!< lco frequency division                 (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x03_register_t;

/**
 * @brief AS3935 register 0x07 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x07_BIT_TAG {
        i2c_as3935_lightning_distances_t  lightning_distance:6; /*!< lightning distance estimation (bit:0-5) */
        uint8_t                           reserved:2;     /*!< reserved and set to 0               (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x07_register_t;

/**
 * @brief AS3935 register 0x08 structure.
 */
typedef union __attribute__((packed)) {
    struct REG_0x08_BIT_TAG {
        uint8_t   tuning_capacitors:4;                  /*!< internal tuning capacitors from 0 to 120pF in steps of 8pF (0-15) (bit:0-3) */
        uint8_t   reserved:1;                           /*!< reserved and set to 0           (bit:4) */
        i2c_as3935_co_irq_pin_states_t display_trco_state:1; /*!< display trco on irq pin    (bit:5) */
        i2c_as3935_co_irq_pin_states_t display_srco_state:1; /*!< display srco on irq pin    (bit:6) */
        i2c_as3935_co_irq_pin_states_t display_lco_state:1;  /*!< display lco on irq pin     (bit:7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} i2c_as3935_0x08_register_t;

/**
 * @brief AS3935 device event object structure.
 */
typedef struct AS3935_DEVICE_TAG {
    i2c_as3935_lightning_distances_t    lightning_distance;
    uint32_t                            lightning_energy; 
} as3935_device_t;

/**
 * @brief I2C AS3935 device configuration structure.
 */
typedef struct I2C_AS3935_CONFIG_TAG {
    i2c_device_config_t                     dev_config;    /*!< i2c configuration for as3935 device */
    bool                                    irq_io_enabled;
    uint32_t                                irq_io_num;    /*!< mcu interrupt pin number for as3935 device */
} i2c_as3935_config_t;

/**
 * @brief I2C AS3935 device handle structure.
 */
struct i2c_as3935_t {
    i2c_master_dev_handle_t     i2c_dev_handle;  /*!< I2C device handle */
    bool                        irq_io_enabled;
    uint32_t                    irq_io_num;    /*!< mcu interrupt pin number for as3935 device */
    i2c_as3935_0x00_register_t  reg_0x00;
    i2c_as3935_0x01_register_t  reg_0x01;
    i2c_as3935_0x02_register_t  reg_0x02;
    i2c_as3935_0x03_register_t  reg_0x03;
    i2c_as3935_0x08_register_t  reg_0x08;
};

/**
 * @brief AS3935 I2C device handle type definitions
*/
typedef struct i2c_as3935_t i2c_as3935_t;
typedef struct i2c_as3935_t *i2c_as3935_handle_t;

typedef gpio_isr_t as3935_isr_t;

/**
 * @brief initialize AS3935 monitor instance.
 * 
 * @param[in] bus_handle i2c master bus handle.
 * @param[in] as3935_config AS3935 configuration.
 * @return as3935_monitor_handle_t AS3935 monitor handle.
 */
as3935_monitor_handle_t as3935_monitor_init(i2c_master_bus_handle_t bus_handle, const i2c_as3935_config_t *as3935_config);

/**
 * @brief de-initialize AS3935 monitor instance.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_monitor_deinit(as3935_monitor_handle_t monitor_handle);

/**
 * @brief adds user defined event handler for AS3935 monitor.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @param[out] handler_args handler specific arguments.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_NO_MEM when unable to allocate memory for the handler.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t as3935_monitor_add_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler, void *handler_args);

/**
 * @brief removes user defined event handler for AS3935 monitor.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t as3935_monitor_remove_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler);

/**
 * @brief gets 0x00 register from AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_0x00_register(i2c_as3935_handle_t as3935_handle);

/**
 * @brief gets 0x01 register from AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_0x01_register(i2c_as3935_handle_t as3935_handle);

/**
 * @brief gets 0x02 register from AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_0x02_register(i2c_as3935_handle_t as3935_handle);

/**
 * @brief gets 0x03 register from AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_0x03_register(i2c_as3935_handle_t as3935_handle);

/**
 * @brief gets 0x08 register from AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_0x08_register(i2c_as3935_handle_t as3935_handle);

/**
 * @brief sets 0x00 register on AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[in] reg AS3935 0x00 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_set_0x00_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x00_register_t reg);

/**
 * @brief sets 0x01 register on AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[in] reg AS3935 0x01 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_set_0x01_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x01_register_t reg);

/**
 * @brief sets 0x02 register on AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[in] reg AS3935 0x02 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_set_0x02_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x02_register_t reg);

/**
 * @brief sets 0x03 register on AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[in] reg AS3935 0x03 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_set_0x03_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x03_register_t reg);

/**
 * @brief sets 0x08 register on AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[in] reg AS3935 0x08 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_set_0x08_register(i2c_as3935_handle_t as3935_handle, const i2c_as3935_0x08_register_t reg);


/**
 * @brief initializes an AS3935 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] as3935_config configuration of AS3935 device.
 * @param[out] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_init(i2c_master_bus_handle_t bus_handle, const i2c_as3935_config_t *as3935_config, i2c_as3935_handle_t *as3935_handle);

esp_err_t i2c_as3935_register_isr(i2c_as3935_handle_t as3935_handle, const as3935_isr_t isr);

/**
 * @brief resets AS3935 to defaults.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_reset_to_defaults(i2c_as3935_handle_t as3935_handle);

/**
 * @brief calibrates AS3935 RC oscillator.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_calibrate_rco(i2c_as3935_handle_t as3935_handle); // crash!!!

/**
 * @brief clears AS3935 lightning statistics.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_clear_lightning_statistics(i2c_as3935_handle_t as3935_handle);

esp_err_t i2c_as3935_enable_power(i2c_as3935_handle_t as3935_handle);
esp_err_t i2c_as3935_get_analog_frontend(i2c_as3935_handle_t as3935_handle, i2c_as3935_analog_frontends_t *const analog_frontend);
esp_err_t i2c_as3935_get_watchdog_threshold(i2c_as3935_handle_t as3935_handle, i2c_as3935_watchdog_thresholds_t *const watchdog_threshold);
esp_err_t i2c_as3935_get_noise_floor_level(i2c_as3935_handle_t as3935_handle, i2c_as3935_noise_levels_t *const noise_level);
esp_err_t i2c_as3935_get_spike_rejection(i2c_as3935_handle_t as3935_handle, uint8_t *const spike_rejection);
esp_err_t i2c_as3935_get_minimum_lightnings(i2c_as3935_handle_t as3935_handle, i2c_as3935_minimum_lightnings_t *const min_lightnings);
esp_err_t i2c_as3935_enable_disturber_detection(i2c_as3935_handle_t as3935_handle);
esp_err_t i2c_as3935_get_frequency_division_ratio(i2c_as3935_handle_t as3935_handle, i2c_as3935_frequency_division_ratios_t *const ratio);
esp_err_t i2c_as3935_get_display_oscillator_on_irq(i2c_as3935_handle_t as3935_handle, i2c_as3935_oscillator_modes_t oscillator_mode, bool *const enabled);
esp_err_t i2c_as3935_get_internal_capacitors(i2c_as3935_handle_t as3935_handle, uint8_t *const value);

esp_err_t i2c_as3935_disable_power(i2c_as3935_handle_t as3935_handle);
esp_err_t i2c_as3935_set_analog_frontend(i2c_as3935_handle_t as3935_handle, const i2c_as3935_analog_frontends_t analog_frontend);
esp_err_t i2c_as3935_set_watchdog_threshold(i2c_as3935_handle_t as3935_handle, const i2c_as3935_watchdog_thresholds_t watchdog_threshold);
esp_err_t i2c_as3935_set_noise_floor_level(i2c_as3935_handle_t as3935_handle, const i2c_as3935_noise_levels_t noise_level);
esp_err_t i2c_as3935_set_spike_rejection(i2c_as3935_handle_t as3935_handle, const uint8_t spike_rejection);
esp_err_t i2c_as3935_set_minimum_lightnings(i2c_as3935_handle_t as3935_handle, const i2c_as3935_minimum_lightnings_t min_lightnings);
esp_err_t i2c_as3935_disable_disturber_detection(i2c_as3935_handle_t as3935_handle);
esp_err_t i2c_as3935_set_frequency_division_ratio(i2c_as3935_handle_t as3935_handle, const i2c_as3935_frequency_division_ratios_t ratio);
esp_err_t i2c_as3935_set_display_oscillator_on_irq(i2c_as3935_handle_t as3935_handle, const i2c_as3935_oscillator_modes_t oscillator_mode, const bool enabled);
esp_err_t i2c_as3935_set_internal_capacitors(i2c_as3935_handle_t as3935_handle, const uint8_t value);

/**
 * @brief gets interrupt state of AS3935.
 * 
 * @param[in] as3935_handle AS3935 device handle.
 * @param[out] state interrupt state of AS3935.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_get_interrupt_state(i2c_as3935_handle_t as3935_handle, i2c_as3935_interrupt_states_t *const state);
esp_err_t i2c_as3935_get_lightning_energy(i2c_as3935_handle_t as3935_handle, uint32_t *const energy);
esp_err_t i2c_as3935_get_lightning_distance(i2c_as3935_handle_t as3935_handle, i2c_as3935_lightning_distances_t *const distance);
esp_err_t i2c_as3935_get_lightning_distance_km(i2c_as3935_handle_t as3935_handle, uint8_t *const distance);
esp_err_t i2c_as3935_get_lightning_event(i2c_as3935_handle_t as3935_handle, i2c_as3935_lightning_distances_t *const distance, uint32_t *const energy);

/**
 * @brief Removes an AS3935 device from I2C master bus.
 *
 * @param[in] as3935_handle as3935 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_remove(i2c_as3935_handle_t as3935_handle);

/**
 * @brief Removes an AS3935 device from master bus and frees handle.
 * 
 * @param as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as3935_delete(i2c_as3935_handle_t as3935_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AS3935_H__
