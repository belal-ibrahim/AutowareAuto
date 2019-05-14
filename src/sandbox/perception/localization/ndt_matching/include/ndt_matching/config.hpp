/// \copyright Copyright 2017-2019 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the configuration class for the voxel grid data structure

#ifndef NDT_MATCHING__CONFIG_HPP_
#define NDT_MATCHING__CONFIG_HPP_

#include <apexcpp/apexcpp.hpp>
#include <ndt_matching/visibility_control.hpp>
#include <helper_functions/helper_functions.hpp>

namespace autoware_bridge
{
namespace ndt_matching
{

/// \brief A configuration class for the PurePursuit class.
class NDT_MATCHING_PUBLIC Config
{
public:
  /// \brief Constructor
  /// \param[in] resolution The minimum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] step_size The maximum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] translation_eps The conversion ratio from the speed to
  ///            the lookahead distance. The ratio is equal to the duration (s)
  /// \param[in] maximum_iteration The boolean whether using the interpolation
  ///            for determining the target position
  /// \param[in] is_delay_compensation The boolean whethre using the delay compensation
  ///            for estimating the current vehicle position at the current timestamp
  /// \param[in] emergency_stop_distance The emergency stop distance for the emergency stop
  /// \param[in] speed_thres_traveling_direction The speed threshold for
  ///            determining the traveling direction
  /// \param[in] distance_front_rear_wheel The distance between front and rear wheels
  Config(
    const real_t resolution,
    const real_t step_size,
    const real_t translation_eps,
    const uint32_t maximum_iteration,
    const std::string & map_file);
  /// \brief Gets the minimum lookahead distance for the pure pursuit
  /// \return Fixed value
  real_t get_resolution() const;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  real_t get_step_size() const;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  real_t get_translation_eps() const;
  /// \brief Gets the boolean whether using the interpolation to get the target point
  /// \return Fixed value
  uint32_t get_maximum_iteration() const;
  /// \brief Gets the boolean whether using the interpolation to get the target point
  /// \return Fixed value
  const std::string & get_map_file() const;

private:
  real_t m_resolution;
  real_t m_step_size;
  real_t m_translation_eps;
  uint32_t m_maximum_iteration;
  std::string m_map_file;
};  // class Config
}  // namespace ndt_matching
}  // namespace autoware_bridge

#endif  // NDT_MATCHING__CONFIG_HPP_
