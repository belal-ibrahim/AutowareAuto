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
#include <chrono>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "xsens_node/xsens_imu_node.hpp"

namespace autoware
{
namespace drivers
{
namespace xsens_node
{
XsensImuNode::XsensImuNode(
  const std::string & node_name,
  const std::string & topic,
  const std::string & device_name,
  uint32_t baud_rate,
  flow_control_t flow_control,
  parity_t parity,
  stop_bits_t stop_bits,
  const std::string & frame_id,
  const xsens_driver::XsensTranslator::Config & config)
: SerialDriverNode<XsensImuNode, xsens_driver::XsensTranslator::Packet, sensor_msgs::msg::Imu>(
    node_name,
    topic,
    device_name,
    baud_rate,
    flow_control,
    parity,
    stop_bits),
  m_translator(config),
  m_frame_id(frame_id)
{
}

////////////////////////////////////////////////////////////////////////////////
XsensImuNode::XsensImuNode(
  const std::string & node_name,
  const std::string & node_namespace)
: SerialDriverNode(node_name, node_namespace),
  m_translator(
      {
      }),
  m_frame_id(declare_parameter("frame_id").get<std::string>().c_str())
{
}
////////////////////////////////////////////////////////////////////////////////
void XsensImuNode::init_output(sensor_msgs::msg::Imu & output)
{
  (void)output;
}

////////////////////////////////////////////////////////////////////////////////
bool XsensImuNode::convert(
  const xsens_driver::XsensTranslator::Packet & pkt,
  sensor_msgs::msg::Imu & output)
{
  m_imu.push_back(output);
  m_translator.convert(pkt, m_imu);
  m_imu.clear();
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool XsensImuNode::get_output_remainder(sensor_msgs::msg::Imu & output)
{
  // The assumption checked in the constructor is that the PointCloud size is bigger than
  // the PointBlocks, which can fully contain a packet. The use case of this method is in case
  // PacketT > OutputT, which is not the case here.
  (void)output;
  return false;
}

}  // namespace xsens_node
}  // namespace drivers
}  // namespace autoware
