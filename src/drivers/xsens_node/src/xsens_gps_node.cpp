// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include "xsens_driver/xsens_gps_translator.hpp"
#include "xsens_node/visibility_control.hpp"
#include "xsens_node/xsens_common_node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `xsens_driver`
namespace xsens_node
{

using XsensImuNode = XsensCommonNode<
  xsens_driver::XsensGpsTranslator, sensor_msgs::msg::NavSatFix
>;

}  // namespace xsens_node
}  // namespace drivers
}  // namespace autoware
