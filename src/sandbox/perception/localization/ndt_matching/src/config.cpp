// Copyright 2017-2019 Apex.AI, Inc.
// All rights reserved.
#include <limits>
#include <stdexcept>
#include <string>
#include "ndt_matching/config.hpp"

namespace autoware_bridge
{
namespace ndt_matching
{

////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const real_t resolution,
  const real_t step_size,
  const real_t translation_eps,
  const uint32_t maximum_iteration,
  const std::string & map_file)
: m_resolution(resolution),
  m_step_size(step_size),
  m_translation_eps(translation_eps),
  m_maximum_iteration(maximum_iteration),
  m_map_file(map_file)
{
  if (m_resolution <= 0.0F) {
    throw std::domain_error("ndt_matching::Config minimum lookahead distance is lower than 0");
  }
  if (m_step_size <= 0.0F) {
    throw std::domain_error("ndt_matching::Config maximum lookahead distance is lower than 0");
  }
  if (m_translation_eps <= 0.0F) {
    throw std::domain_error("ndt_matching::Config: speed to lookahead ratio is lower than 0");
  }
  if (m_maximum_iteration == 0U) {
    throw std::domain_error("ndt_matching::Config: maximum_iteration is lower than 0");
  }
}
////////////////////////////////////////////////////////////////////////////////
real_t Config::get_resolution() const
{
  return m_resolution;
}
////////////////////////////////////////////////////////////////////////////////
real_t Config::get_step_size() const
{
  return m_step_size;
}
////////////////////////////////////////////////////////////////////////////////
real_t Config::get_translation_eps() const
{
  return m_translation_eps;
}
////////////////////////////////////////////////////////////////////////////////
uint32_t Config::get_maximum_iteration() const
{
  return m_maximum_iteration;
}
const std::string & Config::get_map_file() const
{
  return m_map_file;
}
}  // namespace ndt_matching
}  // namespace autoware_bridge
