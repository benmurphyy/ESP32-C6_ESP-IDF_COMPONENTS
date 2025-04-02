#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import config, subprocess


###############################################################################################################
#### MAIN ENTRY POINT ####
###############################################################################################################

# upload each component to the ESP-IDF Components Registry and PlatformIO Library Registry
for component in config.component_list:
    print(component.name + " --> ESP-IDF Components Registry")
    subprocess.run(["powershell", "compote component upload --name " + component.name + " --project-dir '" + component.path + "' --dest-dir '" + config.component_archive_path + "'"])
    print(component.name + " --> PlatformIO Library Registry")
    subprocess.run(["powershell", "pio pkg publish '" + component.path +"' --owner '" + config.pio_owner + "' --type library --no-interactive"])

