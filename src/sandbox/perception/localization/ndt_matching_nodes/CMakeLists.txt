# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ndt_matching_nodes)

## dependencies
find_package(apex_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(apex_auto_msgs REQUIRED)
find_package(apex_tf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apexcpp REQUIRED)
find_package(ndt_matching REQUIRED)
find_package(helper_functions REQUIRED)
find_package(time_policies REQUIRED)

add_library(${PROJECT_NAME} SHARED
  include/ndt_matching_nodes/ndt_matching_node.hpp
  include/ndt_matching_nodes/visibility_control.hpp
  src/ndt_matching_node.cpp
)

# includes
target_include_directories(${PROJECT_NAME} PUBLIC include)
apex_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "apex_auto_msgs"
  "apex_tf"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "apexcpp"
  "helper_functions"
  "ndt_matching"
  "time_policies"
)

set(ndt_matching_NODE_EXE ndt_matching_node_exe)
add_executable(${ndt_matching_NODE_EXE} src/ndt_matching_node_main.cpp)
target_link_libraries(${ndt_matching_NODE_EXE} ${PROJECT_NAME})
apex_set_compile_options(${ndt_matching_NODE_EXE})
ament_target_dependencies(${ndt_matching_NODE_EXE} "apexcpp")

# if(BUILD_TESTING)
#   # run linters
#   apex_static_code_analysis()
#   #TODO(y.tsuji): Disable pclint since the time_policy based subscriber has pclint error
#   # find_package(ament_cmake_pclint)
#   # ament_pclint(CHECK_HEADERS)
#
#   ## Integration test - see monitor_analyzer for a multi-executable example
#   find_package(tf2_msgs REQUIRED)
#   find_package(rclcpp REQUIRED)
#   set(PURE_PURSUIT_NODE_TEST ndt_matching_integration_test_exe)
#   add_executable(${PURE_PURSUIT_NODE_TEST} test/src/ndt_matching_integration_test.cpp)
#   ament_target_dependencies(${PURE_PURSUIT_NODE_TEST}
#     "rclcpp"
#     "tf2_msgs"
#     "apex_auto_msgs"
#     "geometry_msgs"
#     "std_msgs"
#     "apexcpp"
#     "tf2"
#     "helper_functions")
#   apex_set_compile_options(${PURE_PURSUIT_NODE_TEST})
#   apex_install(
#       EXECUTABLES "${PURE_PURSUIT_NODE_TEST}"
#   )
#
#   find_package(apex_launchtest_cmake)
#   add_apex_launchtest(test/src/ndt_matching_node.test.py)
# endif()

## install stuff
apex_install(
  HAS_PARAM
  LIBRARIES ${PROJECT_NAME}
  EXECUTABLES ${ndt_matching_NODE_EXE}
)
# Ament Exporting
ament_export_dependencies(
  "apex_auto_msgs"
  "apex_tf"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "apexcpp"
  "helper_functions"
  "ndt_matching"
  "time_policies"
)
ament_export_interfaces(${ndt_matching_NODE_EXE})
ament_package()
