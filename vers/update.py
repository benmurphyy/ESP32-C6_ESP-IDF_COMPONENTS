#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import config, json, re, subprocess
import constant as const
from component import Component
from constant import FWVersionUpdateParts
from ruamel.yaml import YAML


def version_number_parts() -> tuple:
    """
    version_number_parts

    Latest version number parts (major, minor, patch) from seed file.

    Returns:
        tuple: Version number parts (major, minor, patch).
    
    """
    # read fw version info seed file
    with open(config.fw_version_seed_filepath, 'r') as fp:
        version_info = json.load(fp)
    
    # set fw version number parts
    major = version_info['version_major']
    minor = version_info['version_minor']
    patch = version_info['version_patch']
        
    # return fw version number
    return (major, minor, patch)


def version_number() -> str:
    """
    version_number

    Latest version number from seed file.

    Returns:
        str: Version number (x.x.x).
    
    """
    # read fw version info seed file
    with open(config.fw_version_seed_filepath, 'r') as fp:
        version_info = json.load(fp)
    
    # set fw version number parts
    major = version_info['version_major']
    minor = version_info['version_minor']
    patch = version_info['version_patch']
        
    # return fw version number
    return str(major) + "." + str(minor) + "." + str(patch)

def update_version_number() -> str:
    """
    update_version_number

    Increments version number in seed file and returns an updated version number (x.x.x) from seed file.

    Returns:
        str: Updated version number (x.x.x).
        
    """
    # read fw version info seed file
    with open(config.fw_version_seed_filepath, 'r') as fp:
        version_info = json.load(fp)
    
    # set fw version number parts
    major = version_info['version_major']
    minor = version_info['version_minor']
    patch = version_info['version_patch']
    
    # update fw version part by update type
    if config.fw_version_update_type == FWVersionUpdateParts.FW_VERSION_UPDATE_MAJOR:
        major += 1
        version_info['version_major'] = major
    elif config.fw_version_update_type == FWVersionUpdateParts.FW_VERSION_UPDATE_MINOR:
        minor += 1
        version_info['version_minor'] = minor
    elif config.fw_version_update_type == FWVersionUpdateParts.FW_VERSION_UPDATE_PATCH:
        patch += 1
        version_info['version_patch'] = patch
    
    # write fw version info
    with open(config.fw_version_seed_filepath, 'w') as fp:
        json.dump(version_info, fp)
        
    # return fw version number
    return str(major) + "." + str(minor) + "." + str(patch)


def update_yaml_version(path: str, version_number: str) -> str:
    """
    update_yaml_version
    
    Updates the version number in the idf_component.yml file.

    Args:
        filepath (str): Contains the path to the idf_component.yml file
        version_number (str): Contains the version number to be updated (x.x.x)
        
    """
    filepath = path + "\\" + const.COMPONENT_YAML_FILENAME
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.default_flow_style = False

    with open(filepath, 'r') as fp:
        component = yaml.load(fp)
        
    component['version'] = version_number

    with open(filepath, 'w') as yaml_file:
        yaml.dump(component, yaml_file)


def update_json_version(path: str, version_number: str):
    """
    update_json_version
    
    Updates the version number in the library.json file.

    Args:
        filepath (str): Contains the path to the library.json file
        version_number (str): Contains the version number to be updated (x.x.x)
        
    """
    filepath = path + "\\" + const.COMPONENT_JSON_FILENAME
    
    with open(filepath, 'r') as fp:
        library = json.load(fp)
        
    library['version'] = version_number
    
    with open(filepath, 'w') as fp:
        json.dump(library, fp, indent=2)

def update_c_version(path: str, header_name: str, version_major: int, version_minor: int, version_patch: int):
    """
    update_c_version

    Updates the version number in the c version header file.
    
    /** Major version number (X.x.x) */
        #define [COMPONENT_NAME]_FW_VERSION_MAJOR 1
    /** Minor version number (x.X.x) */
        #define [COMPONENT_NAME]_FW_VERSION_MINOR 1
    /** Patch version number (x.x.X) */
        #define [COMPONENT_NAME]_FW_VERSION_PATCH 2

    Args:
        path (str): Path to component directory.
        header_name (str): Name of C header file.
        version_major (int): Major version number part.
        version_minor (int): Minor version number part.
        version_patch (int): Patch version number part.
        
    """
    major = const.C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_MAJOR
    minor = const.C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_MINOR
    patch = const.C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_PATCH
    
    major_reg_ex = const.REGEXP_C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_MAJOR + const.REGEXP_VERSION_NUMBER
    minor_reg_ex = const.REGEXP_C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_MINOR + const.REGEXP_VERSION_NUMBER
    patch_reg_ex = const.REGEXP_C_DEFINITION_PREFIX + header_name.upper() + const.C_HEADER_VERSION_PATCH + const.REGEXP_VERSION_NUMBER
    
    header_fp = path + "\\include\\" + header_name.lower() + const.C_HEADER_SUFFIX
    
    with open(header_fp, 'r') as fp:
        header_lines = fp.readlines()
     
    with open(header_fp, 'w') as fp:
        for line in header_lines:
            x_major = re.search(major_reg_ex, line)
            x_minor = re.search(minor_reg_ex, line)
            x_patch = re.search(patch_reg_ex, line)
            
            if x_major != None or x_minor != None or x_patch != None:
                if x_major != None:
                    fp.write(major + " " + str(version_major) + "\n")
                if x_minor != None:
                    fp.write(minor + " " + str(version_minor) + "\n")
                if x_patch != None:
                    fp.write(patch + " " + str(version_patch) + "\n")
            else:
                fp.write(line)


def update_version(component: Component, version_major: int, version_minor: int, version_patch: int):
    """
    update_version
    
    Updates the component's version number by version number parts.

    Args:
        component (Component): Instantiate component object.
        version_major (int): Major version number part.
        version_minor (int): Minor version number part.
        version_patch (int): Patch version number part.
        
    """
    # update version number
    ver_num = str(version_major) + "." + str(version_minor) + "." + str(version_patch)
    
    # update yaml
    if component.update_component_yaml == True: 
        update_yaml_version(component.path, ver_num)
    
    # update json
    if component.update_library_json == True: 
        update_json_version(component.path, ver_num)
    
    # update c version header
    if component.update_c_version_header == True: 
        update_c_version(component.path, component.header_name, version_major, version_minor, version_patch)


###############################################################################################################
#### MAIN ENTRY POINT ####
###############################################################################################################

# get current version number
current_version = version_number()

# increment version number
update_version_number()

# update version number for each component
for component in config.component_list:
    version_tpl = version_number_parts()
    update_version(component, version_tpl[0], version_tpl[1], version_tpl[2])
    print(component.name + " version (" + current_version + ") -> " + version_number())
    subprocess.run(["powershell", "compote component upload --name " + component.name + " --project-dir '" + component.path + "' --dest-dir '" + config.component_archive_path + "'"])
    subprocess.run(["powershell", "pio pkg publish '" + component.path +"' --owner '" + config.pio_owner + "' --type library --no-interactive"])


# Shell=True should not be used, security venerability
#subprocess.run(["powershell", "compote component upload --name " + component.name + " --project-dir '" + component.path + "' --dest-dir '" + config.component_archive_path + "'"], shell=True)
#subprocess.run(["powershell", "pio pkg publish '" + component.path +"' --owner '" + config.pio_owner + "' --type library --no-interactive"], shell=True)

#component = config.component_list[5]
#version_tpl = version_number_parts()
#update_version(component, version_tpl[0], version_tpl[1], version_tpl[2])
#print(component.name + " version: " + version_number())