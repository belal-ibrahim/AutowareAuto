# Copyright 2018 Apex.AI, Inc.
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

### Build the nodes
project(xsens_node)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xsens_driver REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lidar_utils REQUIRED)

include_directories(include)

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB xsens_imu_node)
add_library(${CLOUD_LIB} SHARED
  include/xsens_node/xsens_imu_node.hpp
  include/xsens_node/visibility_control.hpp
  src/xsens_imu_node.cpp)
autoware_set_compile_options(${CLOUD_LIB})
ament_target_dependencies(${CLOUD_LIB}
  "lidar_utils"
  "rclcpp_lifecycle"
  "xsens_driver"
  "serial_driver"
  "sensor_msgs")

# generate executable for ros1-style standalone nodes
file(COPY param/xsens_test.param.yaml DESTINATION "${CMAKE_BINARY_DIR}/param")  #copy the param file into the build directory
set(CLOUD_EXEC "xsens_imu_node_exe")
add_executable(${CLOUD_EXEC} src/xsens_imu_node_main.cpp)
autoware_set_compile_options(${CLOUD_EXEC})
target_link_libraries(${CLOUD_EXEC} ${CLOUD_LIB})

if(BUILD_TESTING)
    autoware_static_code_analysis()
endif()

## install stuff
autoware_install(
  LIBRARIES  ${CLOUD_LIB}
  EXECUTABLES ${CLOUD_EXEC}
)

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${CLOUD_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${CLOUD_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion)

# Ament Exporting
ament_export_dependencies("sensor_msgs" "xsens_driver" "serial_driver")
ament_export_interfaces(${BLOCK_EXEC})
ament_package()
