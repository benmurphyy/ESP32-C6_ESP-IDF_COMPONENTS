# ESP-IDF I2C Peripheral Components
Welcome to the I2C peripheral components library with ESP-IDF support.  This library of components is relatively active and updated on a regular basis and a list of operational components and the state of each component under development is available below.

ESP-IDF I2C device peripheral components supported at this time are listed as follows:
 
 - ASAIR AHTXX
 - AKM AK8975
 - OSRAM AS7341
 - ROHM BH1750FVI
 - Bosch BME680 - Work in Progress, Coming Soon!! 
 - Bosch BMP280
 - Bosch BMP390
 - ScioSense CCS811
 - ScioSense ENS160
 - Texas Instruments HDC1080
 - Honeywell HMC5883L
 - Lite-On LTR390UV
 - Maxim-Integrated MAX30105 - Work in Progress
 - Melexis MLX90614
 - Memsic MMC56X3
 - InvenSense MPU6050
 - Sensirion SGP4X
 - Sensirion SHT4X
 - Generic SSD1306 (128x32 and 128x64)
 - Infineon TLV493D - Work in Progress
 - Vishay VEML7700

The above components have a dependency on the `esp_driver_i2c_ext` component that must be included in your project.  This component extends I2C master bus functionality to simplify common I2C transactions, it includes transmit and receive buffers of various sizes, and byte manipulation features.

Components are developed when needed for product development and/or prototyping purposes, and shared with the ESP32 community for ESP-IDF developers.  Custom component development is available upon request, please contact me directly, and I would be happy to discuss the details of your component needs.  If any problems arise please feel free to log an issue and if you would to contribute please contact me.

Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)