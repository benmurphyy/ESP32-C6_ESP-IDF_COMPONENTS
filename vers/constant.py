#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file

COMPONENT_JSON_FILENAME = "library.json"
"""Component JSON file name"""
COMPONENT_YAML_FILENAME = "idf_component.yml"
"""Component YAML file name"""

# [Component-Name]_FW_VERSION_MAJOR
C_HEADER_VERSION_MAJOR = "_FW_VERSION_MAJOR"
"""C header file definition for major version: [Component-Name]_FW_VERSION_MAJOR"""
C_HEADER_VERSION_MINOR = "_FW_VERSION_MINOR"
"""C header file definition for minor version: [Component-Name]_FW_VERSION_MINOR"""
C_HEADER_VERSION_PATCH = "_FW_VERSION_PATCH"
"""C header file definition for patch version: [Component-Name]_FW_VERSION_PATCH"""

C_HEADER_SUFFIX = "_version.h"

C_DEFINITION_PREFIX = "#define "
REGEXP_C_DEFINITION_PREFIX = "^#define "
REGEXP_VERSION_NUMBER = " +[0-9]"

class FWVersionUpdateParts:
    """
    Firmware Version Update Parts Enumerator
    
    Defines which part of the version number will be updated which
    is either the major, minor or patch.  The patch part of the 
    version number is configured by default.
    """
    ### FW Version Update Parts (Enum) ###
    FW_VERSION_UPDATE_MAJOR = 0
    """Major firmware version update part"""
    FW_VERSION_UPDATE_MINOR = 1
    """Minor firmware version update part"""
    FW_VERSION_UPDATE_PATCH = 2
    """Patch firmware version update part"""