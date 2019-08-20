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

#include "xsens_driver/xsens_imu_translator.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include <cmath>

#include <sensor_msgs/msg/imu.hpp>

using autoware::drivers::xsens_driver::XsensTranslator;
using autoware::drivers::xsens_driver::MID;

class xsens_driver : public ::testing::Test
{
public:
  xsens_driver()
  {}

protected:
  XsensTranslator::Packet pkt;
  std::vector<sensor_msgs::msg::Imu> out;
};  // class xsens_driver

TEST_F(xsens_driver, basic)
{
  const XsensTranslator::Config cfg{};
  XsensTranslator driver(cfg);
  pkt.data = 0xFA;
  ASSERT_FALSE(driver.convert(pkt, out));
  pkt.data = 0xFF;
  ASSERT_FALSE(driver.convert(pkt, out));
  pkt.data = static_cast<std::underlying_type_t<MID>>(MID::MT_DATA2);
  ASSERT_FALSE(driver.convert(pkt, out));
  uint8_t length = 78;
  pkt.data = length;
  ASSERT_FALSE(driver.convert(pkt, out));
  uint8_t data[] = {
    0x10, 0x60, 0x04, 0x22, 0xD5, 0x76, 0xBF, 0x80, 0x20, 0x0C, 0x3B, 0x1F, 0x46,
    0xE7, 0x3A, 0x88, 0xF8, 0x56, 0x3B, 0x83, 0xB9, 0xCB, 0x40, 0x20, 0x0C, 0x3F, 0x32, 0xB2, 0x28,
    0x3E, 0x40, 0x58, 0xE9, 0x41, 0x1C, 0x58, 0x12, 0xC0, 0x20, 0x0C, 0xBE, 0x98, 0x84, 0xB3, 0xBF,
    0x00, 0xC2, 0xB6, 0x3F, 0x06, 0x42, 0xFB, 0xE0, 0x20, 0x04, 0x01, 0x80, 0x00, 0x45, 0x20, 0x10,
    0x10, 0x3F, 0x7F, 0xD5, 0xB8, 0x3C, 0x1A, 0xBA, 0xDB, 0xBD, 0x0D, 0xEB, 0x1D, 0x39, 0xFC, 0x2B,
    0x81, 0xF6
  };

  for(uint8_t i = 0; i < length; ++i) {
    pkt.data = data[i];
    ASSERT_FALSE(driver.convert(pkt, out));
  }
  pkt.data = data[length];
  ASSERT_TRUE(driver.convert(pkt, out));
}

int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
