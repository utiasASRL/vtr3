
# Copyright 2021, Autonomous Space Robotics Lab (ASRL)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import os.path as osp
import yaml

SETUP_DIR = os.getenv('VTRTEMP')
DEFAULT_DIR = os.getenv('VTRTEMP')

def confirm_setup(data):
    if not osp.isdir(data["data_dir"]):
        return False

    os.chdir(SETUP_DIR)
    yaml_data = {"/**": {"ros__parameters": data}}
    with open('setup_params.yaml', 'w') as setup_file:
        yaml.dump(yaml_data, setup_file)
    
    return True

def get_default_dir():
    return DEFAULT_DIR

def get_available_subdirs(dir_path=DEFAULT_DIR):
    this_level_subdirs = [subdir for subdir in os.listdir(dir_path) if osp.isdir(osp.join(dir_path, subdir))]
    available_subdirs = []
    for subdir in this_level_subdirs:
        if subdir == "graph":
            available_subdirs.append(dir_path)
            continue
        next_level_subdirs = get_available_subdirs(osp.join(dir_path, subdir))
        available_subdirs.extend(next_level_subdirs)
    
    return available_subdirs
        

