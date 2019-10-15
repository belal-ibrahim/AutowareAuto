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

#ifndef XSENS_NODE__XSENS_COMMON_NODE_HPP_
#define XSENS_NODE__XSENS_COMMON_NODE_HPP_

#include <string>
#include <vector>
#include "serial_driver/serial_driver_node.hpp"
#include "xsens_node/visibility_control.hpp"


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

template<typename TranslatorT, typename MessageT>
class XSENS_NODE_PUBLIC XsensCommonNode
  : public serial_driver::SerialDriverNode<
    XsensCommonNode<TranslatorT, MessageT>,
    typename TranslatorT::Packet,
    MessageT>
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
  XsensCommonNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & device_name,
    uint32_t baud_rate,
    flow_control_t flow_control,
    parity_t parity,
    stop_bits_t stop_bits,
    const std::string & frame_id,
    const typename TranslatorT::Config & config)
  : autoware::drivers::serial_driver::SerialDriverNode<XsensCommonNode<TranslatorT, MessageT>,
      typename TranslatorT::Packet, MessageT>(
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

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] node_namespace Namespace for this node
  XsensCommonNode(
    const std::string & node_name,
    const std::string & node_namespace = "")


  : autoware::drivers::serial_driver::SerialDriverNode<XsensCommonNode<TranslatorT, MessageT>,
      typename TranslatorT::Packet, MessageT>(

      node_name, node_namespace),
    m_translator(
        {
        }),
    m_frame_id(this->declare_parameter("frame_id").template get<std::string>().c_str())
  {
  }

  void init_output(MessageT & output)
  {
    (void)output;
  }

  bool convert(
    const typename TranslatorT::Packet & pkt,
    MessageT & output)
  {
    return m_translator.convert(pkt, output);
  }

  bool get_output_remainder(MessageT & output)
  {
    (void)output;
    return false;
  }

private:
  TranslatorT m_translator;

  const std::string m_frame_id;

  std::vector<MessageT> m_imu;
};  // class XsensCommonNode

}  // namespace xsens_node
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_NODE__XSENS_COMMON_NODE_HPP_
