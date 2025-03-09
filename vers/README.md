# PlatformIO Versioning
The PlatformIO versioning tool is a Python script that increments the last version number and updates the following files in each component:
- idf_component.yml
- library.json
- include/[component_name]_version.h

Once the version number is incremented, the script attempts to upload each component to the `ESP Component Registry` and `PlatformIO Library Registry`.  The version number part to increment (i.g. major, minor, patch) is specified in the `config.py` file.

## Python Script Files
The Python script depends on the following files:
- bin/fw_version_seed.json: version number seed file
- constant.py: constants used in the script
- config.py: configuration file for the script
- component.py: component object model
- update_fw_version.py: main script

## GitHub Commit Tags
TODO: implement git commit with version number tag.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)