#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import config

class Component:
    
    def __init__(self, name: str, header_name: str, relative_path: str, update_c_version_header: bool = True, update_component_yaml: bool = True, update_library_json: bool = True) -> None:
        self._name = name
        self._header_name = header_name
        self._relative_path = relative_path
        self._update_c_version_header = update_c_version_header
        self._update_component_yaml = update_component_yaml
        self._update_library_json = update_library_json
        
    @property
    def name(self) -> str:
        return self._name
    
    @property
    def header_name(self) -> str:
        return self._header_name
    
    @property
    def relative_path(self) -> str:
        return self.relative_path
    
    @property
    def path(self) -> str:
        return config.component_working_path + "\\" + self._relative_path
    
    @property
    def update_c_version_header(self) -> bool:
        return self._update_c_version_header
    
    @property
    def update_component_yaml(self) -> bool:
        return self._update_component_yaml
    
    @property
    def update_library_json(self) -> bool:
        return self._update_library_json
    

        