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


/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines a simple ROS 2 xsens driver that publishes full point clouds

#ifndef XSENS_NODE__XSENS_IMU_NODE_HPP_
#define XSENS_NODE__XSENS_IMU_NODE_HPP_

#include <string>
#include <vector>
#include "serial_driver/serial_driver_node.hpp"
#include "xsens_driver/xsens_imu_translator.hpp"
#include "xsens_node/visibility_control.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `xsens_driver`
namespace xsens_node
{

using autoware::drivers::serial_driver::flow_control_t;
using autoware::drivers::serial_driver::parity_t;
using autoware::drivers::serial_driver::stop_bits_t;

class XSENS_NODE_PUBLIC XsensImuNode
  : public serial_driver::SerialDriverNode<
    XsensImuNode,
    xsens_driver::XsensImuTranslator::Packet,
    sensor_msgs::msg::Imu>
{
public:
  /// \brief Default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp internals
  /// \param[in] topic Name of the topic to publish output on
  /// \param[in] device_name Name of the serial device.
  /// \param[in] baud_rate Baud rate to read from the serial port.
  /// \param[in] flow_control Whether to use hardware, software or no flow control.
  /// \param[in] parity Parity of the serial transmission.
  /// \param[in] stop_bits Stop bits of the serial transmission.
  /// \param[in] frame_id Frame id for the published point cloud messages
  /// \param[in] config Config struct with rpm, transform, radial and angle pruning params
  /// \throw std::runtime_error If cloud_size is not sufficiently large
  XsensImuNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & device_name,
    uint32_t baud_rate,
    flow_control_t flow_control,
    parity_t parity,
    stop_bits_t stop_bits,
    const std::string & frame_id,
    const xsens_driver::XsensImuTranslator::Config & config);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] node_namespace Namespace for this node
  XsensImuNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  void init_output(sensor_msgs::msg::Imu & output);
  bool convert(
    const xsens_driver::XsensImuTranslator::Packet & pkt,
    sensor_msgs::msg::Imu & output);
  bool get_output_remainder(sensor_msgs::msg::Imu & output);

private:
  xsens_driver::XsensImuTranslator m_translator;

  const std::string m_frame_id;

  std::vector<sensor_msgs::msg::Imu> m_imu;
};  // class XsensImuNode

}  // namespace xsens_node
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_NODE__XSENS_IMU_NODE_HPP_
