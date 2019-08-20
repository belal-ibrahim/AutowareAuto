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

#include <cstring>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
#include <iostream>

#include <iomanip>

#include "sensor_msgs/msg/imu.hpp"
#include "xsens_driver/xsens_imu_translator.hpp"

namespace autoware
{
namespace drivers
{
namespace xsens_driver
{

MID MID_from_int(uint16_t value)
{
  switch (value) {
    case 0x42:
      return MID::ERROR;
    case 0x3E:
      return MID::WAKE_UP;
    case 0x3F:
      return MID::WAKE_UP_ACK;
    case 0x30:
      return MID::GO_TO_CONFIG;
    case 0x10:
      return MID::GO_TO_MEASUREMENT;
    case 0x40:
      return MID::RESET;
    case 0x00:
      return MID::REQ_DID;
    case 0x01:
      return MID::DEVICE_ID;
    case 0x1C:
      return MID::REQ_PRODUCT_CODE;
    case 0x1D:
      return MID::PRODUCT_CODE;
    case 0x12:
      return MID::REQ_FW_REV;
    case 0x13:
      return MID::FIRMWARE_REV;
    case 0x0E:
      return MID::RESTORE_FACTORY_DEF;
    case 0x18:
      return MID::SET_BAUDRATE;
    case 0x24:
      return MID::RUN_SELFTEST;
    case 0x25:
      return MID::SELFTEST_ACK;
    case 0xDA:
      return MID::SET_ERROR_MODE;
    case 0xDC:
      return MID::SET_TRANSMIT_DELAY;
    case 0x48:
      return MID::SET_OPTION_FLAGS;
    case 0x84:
      return MID::SET_LOCATION_ID;
    case 0x2C:
      return MID::SET_SYNC_SETTINGS;
    case 0x0C:
      return MID::REQ_CONFIGURATION;
    case 0x0D:
      return MID::CONFIGURATION;
    case 0x04:
      return MID::SET_PERIOD;
    case 0x86:
      return MID::SET_EXT_OUTPUT_MODE;
    case 0xC0:
      return MID::SET_OUTPUT_CONFIGURATION;
    case 0x8E:
      return MID::SET_STRING_OUTPUT_TYPE;
    case 0xEC:
      return MID::SET_ALIGNMENT_ROTATION;
    case 0xD0:
      return MID::SET_OUTPUT_MODE;
    case 0xD2:
      return MID::SET_OUTPUT_SETTINGS;
    case 0x34:
      return MID::REQ_DATA;
    case 0x32:
      return MID::MT_DATA;
    case 0x36:
      return MID::MT_DATA2;
    case 0xA4:
      return MID::RESET_ORIENTATION;
    case 0x60:
      return MID::SET_UTC_TIME;
    case 0xA8:
      return MID::ADJUST_UTC_TIME;
    case 0x61:
      return MID::UTC_TIME;
    case 0x62:
      return MID::REQ_AVAILABLE_SCENARIOS;
    case 0x63:
      return MID::AVAILABLE_SCENARIOS;
    case 0x64:
      return MID::SET_CURRENT_SCENARIO;
    case 0x66:
      return MID::SET_GRAVITY_MAGNITUDE;
    case 0x6E:
      return MID::SET_LAT_LON_ALT;
    case 0x22:
      return MID::SET_NO_ROTATION;
    default:
      throw std::runtime_error("Unknown value: " + std::to_string(value));
  }
}

XDIGroup XDIGroup_from_int(uint16_t value)
{
  switch (value) {
    case 0x0800:
      return XDIGroup::TEMPERATURE;
    case 0x1000:
      return XDIGroup::TIMESTAMP;
    case 0x2000:
      return XDIGroup::ORIENTATION_DATA;
    case 0x3000:
      return XDIGroup::PRESSURE;
    case 0x4000:
      return XDIGroup::ACCELERATION;
    case 0x5000:
      return XDIGroup::POSITION;
    case 0x7000:
      return XDIGroup::GNSS;
    case 0x8000:
      return XDIGroup::ANGULAR_VELOCITY;
    case 0x8800:
      return XDIGroup::GPS;
    case 0xA000:
      return XDIGroup::SENSOR_COMPONENT_READOUT;
    case 0xB000:
      return XDIGroup::ANALOG_IN;
    case 0xC000:
      return XDIGroup::MAGNETIC;
    case 0xD000:
      return XDIGroup::VELOCITY;
    case 0xE000:
      return XDIGroup::STATUS;
    default:
      throw std::runtime_error("Unknown value: " + std::to_string(value));
  }
}

////////////////////////////////////////////////////////////////////////////////
XsensTranslator::XsensTranslator(const Config &)
: current_state_(State::START)
{
}

bool XsensTranslator::convert(const Packet & pkt, std::vector<sensor_msgs::msg::Imu> & output)
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

bool XsensTranslator::use_double_precision(int32_t data_id)
{
  if ((data_id & 0x0003) == 0x3) {
    return true;
  } else if ((data_id & 0x0003) == 0x0) {
    return false;
  } else {
    throw std::runtime_error("fixed point precision not supported.");
  }
}

void XsensTranslator::parse_mtdata2(std::vector<sensor_msgs::msg::Imu> & output)
{
  auto data = raw_message_;

  sensor_msgs::msg::Imu message;

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
    switch (xdigroup) {
      case XDIGroup::TEMPERATURE:
        break;
      case XDIGroup::TIMESTAMP:
        parse_timestamp(message, data_id, content);
        break;
      case XDIGroup::ORIENTATION_DATA:
        parse_orientation_data(message, data_id, content);
        break;
      case XDIGroup::PRESSURE:
        break;
      case XDIGroup::ACCELERATION:
        parse_acceleration(message, data_id, content);
        break;
      case XDIGroup::POSITION:
        break;
      case XDIGroup::GNSS:
        break;
      case XDIGroup::ANGULAR_VELOCITY:
        parse_angular_velocity(message, data_id, content);
        break;
      case XDIGroup::GPS:
        break;
      case XDIGroup::SENSOR_COMPONENT_READOUT:
        break;
      case XDIGroup::ANALOG_IN:
        break;
      case XDIGroup::MAGNETIC:
        break;
      case XDIGroup::VELOCITY:
        break;
      case XDIGroup::STATUS:
        break;
      default:
        throw std::runtime_error("Unknown group");
    }
  }
  output.push_back(message);
}

void XsensTranslator::parse_timestamp(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  (void)message;
  (void)data_id;
  (void)content;
}

void XsensTranslator::parse_acceleration(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  switch (data_id & 0x000C) {
    case 0x00:
      message.header.frame_id = "ENU";
      break;
    case 0x04:
      message.header.frame_id = "NED";
      break;
    case 0x08:
      message.header.frame_id = "NWU";
      break;
  }

  if (use_double_precision(data_id)) {
    parse_acceleration_internal<double>(message, content);
  } else {
    parse_acceleration_internal<float>(message, content);
  }
}

template<typename T>
void XsensTranslator::parse_acceleration_internal(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  constexpr std::size_t kNumber_of_values = 3;
  T values[kNumber_of_values];

  std::size_t array_size = sizeof(T);

  // Bytes comes in big-endian, so we need to re-store them in little-endian
  for (std::size_t i = 0; i < array_size; ++i) {
    for (std::size_t j = 0; j < kNumber_of_values; ++j) {
      reinterpret_cast<uint8_t *>(&values[j])[i] = content[array_size * (j + 1) - i - 1];
    }
  }

  message.linear_acceleration.x = values[0];
  message.linear_acceleration.y = values[1];
  message.linear_acceleration.z = values[2];
}

void XsensTranslator::parse_orientation_data(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  switch (data_id & 0x000C) {
    case 0x00:
      message.header.frame_id = "ENU";
      break;
    case 0x04:
      message.header.frame_id = "NED";
      break;
    case 0x08:
      message.header.frame_id = "NWU";
      break;
  }

  switch (data_id & 0x00F0) {
    case 0x10:
      // Quaternion
      if (use_double_precision(data_id)) {
        parse_orientation_quaternion<double>(message, content);
      } else {
        parse_orientation_quaternion<float>(message, content);
      }
      break;
    case 0x20:
      // Rotation Matrix
      break;
    case 0x30:
      // Euler Angles
      break;
  }
}

void XsensTranslator::parse_angular_velocity(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  switch (data_id & 0x000C) {
    case 0x00:
      message.header.frame_id = "ENU";
      break;
    case 0x04:
      message.header.frame_id = "NED";
      break;
    case 0x08:
      message.header.frame_id = "NWU";
      break;
  }

  switch (data_id & 0x00F0) {
    case 0x20:
      // Rate of Turn
      if (use_double_precision(data_id)) {
        parse_angular_velocity_rate_of_turn<double>(message, content);
      } else {
        parse_angular_velocity_rate_of_turn<float>(message, content);
      }
      break;
    case 0x30:
      // Delta Q
      break;
    case 0x40:
      // RateOfTurnHR
      if (use_double_precision(data_id)) {
        parse_angular_velocity_rate_of_turn<double>(message, content);
      } else {
        parse_angular_velocity_rate_of_turn<float>(message, content);
      }
      break;
  }
}

template<typename T>
void XsensTranslator::parse_orientation_quaternion(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  constexpr std::size_t kNumber_of_values = 4;
  T values[kNumber_of_values];

  std::size_t array_size = sizeof(T);

  // Bytes comes in big-endian, so we need to re-store them in little-endian
  for (std::size_t i = 0; i < array_size; ++i) {
    for (std::size_t j = 0; j < kNumber_of_values; ++j) {
      reinterpret_cast<uint8_t *>(&values[j])[i] = content[array_size * (j + 1) - i - 1];
    }
  }

  message.orientation.x = values[0];
  message.orientation.y = values[1];
  message.orientation.z = values[2];
  message.orientation.w = values[3];
}

template<typename T>
void XsensTranslator::parse_angular_velocity_rate_of_turn(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  constexpr std::size_t kNumber_of_values = 3;
  T values[kNumber_of_values];

  std::size_t array_size = sizeof(T);

  // Bytes comes in big-endian, so we need to re-store them in little-endian
  for (std::size_t i = 0; i < array_size; ++i) {
    for (std::size_t j = 0; j < kNumber_of_values; ++j) {
      reinterpret_cast<uint8_t *>(&values[j])[i] = content[array_size * (j + 1) - i - 1];
    }
  }

  message.angular_velocity.x = values[0];
  message.angular_velocity.y = values[1];
  message.angular_velocity.z = values[2];
}
}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware
