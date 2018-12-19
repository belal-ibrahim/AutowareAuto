# Copyright 2018 Apex.AI, Inc.
# All rights reserved.
# target is a target

macro(_setup_memory_tools_once)
  if(DEFINED _setup_memory_tools_once_flag)
    return()
  endif()
  set(_setup_memory_tools_once_flag TRUE)
  find_package(osrf_testing_tools_cpp REQUIRED)

  get_target_property(__extra_memory_tools_env_vars
    osrf_testing_tools_cpp::memory_tools LIBRARY_PRELOAD_ENVIRONMENT_VARIABLE)
endmacro()

macro(autoware_test_tools_add_gtest target)
  _setup_memory_tools_once()
  ament_add_gtest(${target}
    ${ARGN}
    ENV ${__extra_memory_tools_env_vars}
  )

  target_link_libraries(${target} osrf_testing_tools_cpp::memory_tools)
  ament_target_dependencies(${target} autoware_test_tools)
endmacro()
