cmake_minimum_required(VERSION 3.5)

project(autoware_test_tools)

find_package(autoware_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(osrf_testing_tools_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
endif()

ament_export_dependencies(osrf_testing_tools_cpp ament_cmake_gtest)

autoware_install(HAS_CMAKE)

ament_package(
  CONFIG_EXTRAS "autoware_test_tools-extras.cmake"
)
