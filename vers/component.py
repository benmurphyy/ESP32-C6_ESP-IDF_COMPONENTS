#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file
import config

class Component:
    
    def __init__(self, name: str, header_name: str, relative_path: str) -> None:
        self._name = name
        self._header_name = header_name
        self._relative_path = relative_path
        
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
    

        