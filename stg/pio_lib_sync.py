#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import json, yaml, sys



##################################################################################
# start of script
##################################################################################
# total arguments
n = len(sys.argv)

# validate the number of arguments
if n != 2: 
    print("Usage: python pio_lib_sync.py OK")
    sys.exit(1)

# validate 'OK' argument
if sys.argv[1] != "OK":
    print("Usage: python pio_lib_sync.py OK")
    sys.exit(1)



sys.exit(0)