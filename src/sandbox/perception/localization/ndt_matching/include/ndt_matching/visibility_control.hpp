// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#ifndef NDT_MATCHING__VISIBILITY_CONTROL_HPP_
#define NDT_MATCHING__VISIBILITY_CONTROL_HPP_

#include <apexutils/apexdef.h>

////////////////////////////////////////////////////////////////////////////////
#if defined(APEX_WINDOWS)
  #if defined(ndt_matching_BUILDING_DLL) || defined(ndt_matching_EXPORTS)
    #define NDT_MATCHING_PUBLIC __declspec(dllexport)
    #define ndt_matching_LOCAL
  #else  // defined(ndt_matching_BUILDING_DLL) || defined(ndt_matching_EXPORTS)
    #define NDT_MATCHING_PUBLIC __declspec(dllimport)
    #define ndt_matching_LOCAL
  #endif  // defined(ndt_matching_BUILDING_DLL) || defined(ndt_matching_EXPORTS)
#elif defined(APEX_LINUX)
  #define NDT_MATCHING_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_OSX)
  #define NDT_MATCHING_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_LOCAL __attribute__((visibility("hidden")))
#elif defined(APEX_QNX)
  #define NDT_MATCHING_PUBLIC __attribute__((visibility("default")))
  #define ndt_matching_LOCAL __attribute__((visibility("hidden")))
#else  // defined(APEX_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(APEX_WINDOWS)

#include <eigen3/Eigen/Dense>

// Convenience hack
namespace autoware_bridge
{
namespace ndt_matching
{
using real_t = float32_t;
using Vector3 = Eigen::Vector3f;
using Matrix3 = Eigen::Matrix3f;
}  // namespace ndt_matching
}  // namespace autoware_bridge

#endif  // NDT_MATCHING__VISIBILITY_CONTROL_HPP_
