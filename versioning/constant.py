#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file

COMPONENT_JSON_FILENAME = "library.json"
COMPONENT_YAML_FILENAME = "idf_component.yml"

# [Component-Name]_FW_VERSION_MAJOR
C_HEADER_VERSION_MAJOR = "FW_VERSION_MAJOR"
"""[Component-Name]_FW_VERSION_MAJOR"""
C_HEADER_VERSION_MINOR = "FW_VERSION_MINOR"
"""[Component-Name]_FW_VERSION_MINOR"""
C_HEADER_VERSION_PATCH = "FW_VERSION_PATCH"
"""[Component-Name]_FW_VERSION_PATCH"""

class FWVersionUpdateTypes:
    """Firmware version update types enumerator"""
    ### FW Version Update Types (Enum) ###
    FW_VERSION_UPDATE_MAJOR = 0
    """Major firmware version update type"""
    FW_VERSION_UPDATE_MINOR = 1
    """Minor firmware version update type"""
    FW_VERSION_UPDATE_PATCH = 2
    """Patch firmware version update type"""