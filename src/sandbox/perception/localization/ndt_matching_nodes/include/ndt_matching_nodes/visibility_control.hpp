// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#ifndef NDT_MATCHING_NODES__VISIBILITY_CONTROL_HPP_
#define NDT_MATCHING_NODES__VISIBILITY_CONTROL_HPP_

#include <apexutils/apexdef.h>

////////////////////////////////////////////////////////////////////////////////
#if defined(APEX_WINDOWS)
  #if defined(ndt_matching_NODES_BUILDING_DLL) || \
  defined(ndt_matching_NODES_EXPORTS)
    #define NDT_MATCHING_NODES_PUBLIC __declspec(dllexport)
    #define ndt_matching_NODES_LOCAL
  #else  // defined(ndt_matching_NODES_BUILDING_DLL) ||
         // defined(ndt_matching_NODES_EXPORTS)
    #define NDT_MATCHING_NODES_PUBLIC __declspec(dllimport)
    #define ndt_matching_NODES_LOCAL
  #endif  // defined(ndt_matching_NODES_BUILDING_DLL) ||
          // defined(ndt_matching_NODES_EXPORTS)
#elif defined(APEX_LINUX)
  #define NDT_MATCHING_NODES_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_OSX)
  #define NDT_MATCHING_NODES_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_QNX)
  #define NDT_MATCHING_NODES_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(APEX_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(APEX_WINDOWS)

#endif  // NDT_MATCHING_NODES__VISIBILITY_CONTROL_HPP_
