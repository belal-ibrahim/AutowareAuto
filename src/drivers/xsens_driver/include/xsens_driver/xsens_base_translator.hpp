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

#ifndef XSENS_DRIVER__XSENS_BASE_TRANSLATOR_HPP_
#define XSENS_DRIVER__XSENS_BASE_TRANSLATOR_HPP_

#include <xsens_driver/xsens_common.hpp>
#include <xsens_driver/visibility_control.hpp>
#include <cstdint>
#include <mutex>
#include <numeric>
#include <vector>

#include "helper_functions/crtp.hpp"

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
namespace xsens_driver
{

template<typename Derived, typename MessageT>
class XSENS_DRIVER_PUBLIC XsensBaseTranslator
  : public autoware::common::helper_functions::crtp<Derived>
{
public:
  struct Packet
  {
    uint8_t data;
  };

protected:
  enum class State
  {
    START,
    PREAMBLE_READ,
    BID_READ,
    MID_READ,
    LENGTH_READ,
  };

  std::vector<uint8_t> raw_message_;

  std::mutex raw_message_mutex_;

  State current_state_;

  MID current_mid_;

  std::size_t current_length_;

public:
  XsensBaseTranslator()
  : current_state_(State::START) {}

  bool use_double_precision(int32_t data_id)
  {
    if ((data_id & 0x0003) == 0x3) {
      return true;
    } else if ((data_id & 0x0003) == 0x0) {
      return false;
    } else {
      throw std::runtime_error("fixed point precision not supported.");
    }
  }

  bool convert(const Packet & pkt, std::vector<MessageT> & output)
  {
    switch (current_state_) {
      case State::START:
        if (pkt.data == 0xFA) {
          current_state_ = State::PREAMBLE_READ;
        } else {
          current_state_ = State::START;
        }
        return false;
      case State::PREAMBLE_READ:
        if (pkt.data == 0xFF) {
          current_state_ = State::BID_READ;
        } else {
          current_state_ = State::START;
        }
        return false;
      case State::BID_READ:
        current_mid_ = MID_from_int(pkt.data);
        current_state_ = State::MID_READ;
        return false;
      case State::MID_READ:
        current_length_ = pkt.data;
        current_state_ = State::LENGTH_READ;
        return false;
      case State::LENGTH_READ:
        std::lock_guard<std::mutex> lock(raw_message_mutex_);
        if (raw_message_.size() == current_length_) {
          std::size_t checksum = 0xFF;
          // NOTE(esteve): workaround for uncrustify. The standard ROS 2 configuration does not
          // understand nested < > in templates
          using MID_underlying_type = std::underlying_type_t<decltype(current_mid_)>;
          checksum += static_cast<MID_underlying_type>(current_mid_);
          checksum += current_length_;
          checksum += pkt.data;
          std::size_t sum =
            std::accumulate(std::begin(raw_message_), std::end(raw_message_), checksum);
          if (0xFF & sum) {
            // Checksum error, start over
            current_state_ = State::START;
            current_length_ = 0;
            raw_message_.clear();
            return false;
          } else {
            if (current_mid_ == MID::MT_DATA) {
              // TODO(esteve): parse legacy data
            } else if (current_mid_ == MID::MT_DATA2) {
              parse_mtdata2(output);
            }
            current_state_ = State::START;
            current_length_ = 0;
            raw_message_.clear();
          }
          return true;
        } else {
          raw_message_.push_back(pkt.data);
          return false;
        }
        return false;
    }
    return false;
  }

  void parse_mtdata2(std::vector<MessageT> & output)
  {
    auto data = raw_message_;

    MessageT message;

    while (data.size() > 0) {
      int32_t data_id = data[1] | data[0] << 8;
      int32_t message_size = data[2];

      auto content = decltype(data)(
        std::begin(data) + 3,
        std::begin(data) + 3 + message_size);

      data = decltype(data)(
        std::begin(data) + 3 + message_size,
        std::end(data));

      int32_t group = data_id & 0xF800;
      XDIGroup xdigroup = XDIGroup_from_int(static_cast<uint16_t>(group));
      this->impl().parse_xdigroup_mtdata2(xdigroup, message, data_id, content);
    }
    output.push_back(message);
  }
};

}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_DRIVER__XSENS_BASE_TRANSLATOR_HPP_