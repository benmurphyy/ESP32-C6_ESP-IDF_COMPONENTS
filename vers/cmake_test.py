#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import sys
# C://Users//lavco//OneDrive//Documents//PlatformIO//Projects//ESP32-S3_ESP-IDF_COMPONENTS//vers//cmake_test.py

# C://Users//lavco//OneDrive//Documents//PlatformIO//Projects//ESP32-S3_ESP-IDF_COMPONENTS//components//test.txt
# "C://Users//lavco//OneDrive//Documents//PlatformIO//Projects//ESP32-S3_ESP-IDF_COMPONENTS//components//utilities/esp_uuid" "1.2.2"

out_file_path = "C://Users//lavco//OneDrive//Documents//PlatformIO//Projects//ESP32-S3_ESP-IDF_COMPONENTS//components//test.txt"

# total arguments
n = len(sys.argv)

f = open(out_file_path, "a")
for i in range(n):
    # printing all the arguments
    f.write( str(sys.argv[i]) )
    f.write("\n")
f.close()