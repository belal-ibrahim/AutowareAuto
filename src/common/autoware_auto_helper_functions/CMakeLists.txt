# Copyright 2019 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_helper_functions)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
endif()

# Ament Exporting
autoware_install(HAS_INCLUDE)
ament_package()
