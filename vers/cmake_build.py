#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import json, yaml, sys

COMPONENT_JSON_FILENAME = "library.json"
COMPONENT_YAML_FILENAME = "idf_component.yml"

def update_yaml_version(path: str, version_number: str) -> str:
    """
    update_yaml_version
    
    Updates the version number in the idf_component.yml file.

    Args:
        filepath (str): Contains the path to the idf_component.yml file
        version_number (str): Contains the version number to be updated (x.x.x)
        
    """
    filepath = path + "\\" + COMPONENT_YAML_FILENAME
    
    with open(filepath, 'r', encoding = "utf-8") as fp:
        component = yaml.safe_load(fp)
        
    component['version'] = version_number

    with open(filepath, 'w') as yaml_file:
        yaml.dump(component, yaml_file, default_flow_style = False, allow_unicode = True, encoding = None)
        
        
def update_json_version(path: str, version_number: str):
    """
    update_json_version
    
    Updates the version number in the library.json file.

    Args:
        filepath (str): Contains the path to the library.json file
        version_number (str): Contains the version number to be updated (x.x.x)
        
    """
    filepath = path + "\\" + COMPONENT_JSON_FILENAME
    
    with open(filepath, 'r') as fp:
        library = json.load(fp)
        
    library['version'] = version_number
    
    with open(filepath, 'w') as fp:
        json.dump(library, fp, indent=2)


##################################################################################
# start of script
##################################################################################
# total arguments
n = len(sys.argv)

# validate the number of arguments
if n != 3: 
    print("Usage: python cmake_build.py <component_path> <version_number>")
    sys.exit(1)

# get the component path and version number from the command line arguments
component_path = sys.argv[1]
version_number = sys.argv[2]

# update the version number in the idf_component.yml file
update_yaml_version(component_path, version_number)

# update the version number in the library.json file
update_json_version(component_path, version_number)

sys.exit(0)