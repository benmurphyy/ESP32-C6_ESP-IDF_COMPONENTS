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
 * @file uuid.c
 *
 * ESP-IDF UUID generator
 * 
 * Motivated by: https://github.com/RobTillaart/UUID/blob/master/README.md
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "include/uuid.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <esp_timer.h>


/* constant definitions */
#define UUID_ARGS_SIZE      2
#define UUID_RANDOM_SIZE    4
#define UUID_BUFFER_SIZE    37
#define UUID_HASH_MAX_SIZE  60


/* Marsaglia initializer 'constants' */
static uint32_t     uuid_m_w = 0;
static uint32_t     uuid_m_z = 0;
/* uuid mode - variant-4 (default) */
static uuid_modes_t uuid_mode = UUID_MODE_VARIANT4;




/**
 * @brief An example of a simple pseudo-random number generator is the 
 * multiply-with-carry method invented by George Marsaglia.
 * 
 * @note Two initializers (not null), see `uuid_seed` function.
 * 
 * @return uint32_t Random generated number.
 */
static inline uint32_t uuid_random(void) {
    uuid_m_z = 36969L * (uuid_m_z & 65535L) + (uuid_m_z >> 16);
    uuid_m_w = 18000L * (uuid_m_w & 65535L) + (uuid_m_w >> 16);
    return (uuid_m_z << 16) + uuid_m_w;
}

/**
 * @brief An example of a simple hash generator from a seeding string.
 * 
 * @param str Hash seeding string (i.e. __TIME__, __DATE__, __FILE__)
 * @return uint32_t Generated hash.
 */
static inline uint32_t uuid_hash(const char * str) {
  //  very simple hash function.
  uint32_t hash = strnlen(str, UUID_HASH_MAX_SIZE);
  for (char *p = (char *) str; *p != 0; p++) {
    //  use a big prime as magic number.
    hash = hash * 2000099957 + (uint8_t) *p;
  }
  return hash;
}

void uuid_init(void) {
    //  seed - differs per construction on compile time constants
    //         and multiplication with esp_timer_get_time() (microseconds).
    uint32_t s2 = uuid_hash(__TIME__) * (uint32_t)esp_timer_get_time();
    uint32_t s1 = s2 ^ uuid_hash(__DATE__);
             s2 = s2 ^ uuid_hash(__FILE__);
    uuid_seed(2, s1, s2);
    uuid_set_mode(UUID_MODE_VARIANT4);
}

void uuid_seed(uint8_t size, ... ) {
    va_list valist;
    uint32_t s[UUID_ARGS_SIZE] = { 0, 0 };

    /* validate size */
    if(size > 2) size = 2;

    /* initialize valist for number of arguments */
    va_start(valist, size);

    /* iterate through each argument */
    for(int32_t i = 0; i < size; i++) {
        s[i] = va_arg(valist, uint32_t);
    }

    /* clean memory reserved for valist */
    va_end(valist);

    /* prevent 0 as value */
    if(s[0] == 0) s[0] = 1;
    if(s[1] == 0) s[1] = 2;

    /* set Marsaglia constants */
    uuid_m_w = s[0];
    uuid_m_z = s[1];
}

const char* uuid_generate(void) {
    static char uuid_buffer[UUID_BUFFER_SIZE];
    uint32_t ar[UUID_RANDOM_SIZE];

    /* generate 4 random numbers */
    for (int32_t i = 0; i < 4; i++) {
        ar[i] = uuid_random();
    }

    /*
        Conforming to RFC-4122 Specification:

            - byte 7: four most significant bits ==> 0100  --> always 4
            - byte 9: two  most significant bits ==> 10    --> always {8, 9, A, B}.

        patch bits for version 1 and variant 4
    */
    if (uuid_mode == UUID_MODE_VARIANT4) {
        ar[1] &= 0xFFF0FFFF;   //  remove 4 bits.
        ar[1] |= 0x00040000;   //  variant 4
        ar[2] &= 0xFFFFFFF3;   //  remove 2 bits
        ar[2] |= 0x00000008;   //  version 1
    }
  
    /*  process 16 bytes build up the char array. */
    for (int32_t i = 0, j = 0; i < 16; i++) {
        /*
            multiples of 4 between 8 and 20 get a -.
            note we are processing 2 digits in one loop.
        */
        if ((i & 0x1) == 0) {
            if ((4 <= i) && (i <= 10)) {
                uuid_buffer[j++] = '-';
            }
        }
  
        /* process one byte at the time instead of a nibble */
        uint8_t nr = i / 4;
        uint8_t xx = ar[nr];
        uint8_t ch = xx & 0x0F;

        uuid_buffer[j++] = (ch < 10) ? '0' + ch : ('a' - 10) + ch;

        ch = (xx >> 4) & 0x0F;
        ar[nr] >>= 8;

        uuid_buffer[j++] = (ch < 10) ? '0' + ch : ('a' - 10) + ch;
    }
  
    /* null terminator - string */
    uuid_buffer[UUID_BUFFER_SIZE-1] = 0x00;

    return (const char*)uuid_buffer;
}

void uuid_set_mode(const uuid_modes_t mode) {
    uuid_mode = mode;
}

uuid_modes_t uuid_get_mode(void) {
    return uuid_mode;
}

const char* uuid_get_fw_version(void) {
    return (const char*)UUID_FW_VERSION_STR;
}

int32_t uuid_get_fw_version_number(void) {
    return UUID_FW_VERSION_INT32;
}
