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

#include "xsens_driver/xsens_gps_translator.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include <cmath>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

using autoware::drivers::xsens_driver::XsensGpsTranslator;
using autoware::drivers::xsens_driver::MID;

class xsens_driver : public ::testing::Test
{
public:
  xsens_driver()
  {}

protected:
  XsensGpsTranslator::Packet pkt;
  sensor_msgs::msg::NavSatFix out;
};  // class xsens_driver

TEST_F(xsens_driver, basic)
{
  std::vector<uint8_t> data = {
    0x70, 0x10, 0x5E, 0x1C, 0x10, 0x5C, 0x4A, 0x07, 0xE3, 0x08, 0x1E, 0x0A, 0x2E, 0x38, 0xF7, 0x00,
    0x00, 0x03, 0xEE, 0x0E, 0xDF, 0x8E, 0x8A, 0x03, 0x03, 0x0E, 0x00, 0xB7, 0x39, 0x00, 0x17, 0x16,
    0x4E, 0x8B, 0x08, 0xFF, 0xFF, 0xC0, 0xC3, 0x00, 0x00, 0x35, 0xD3, 0x00, 0x00, 0x05, 0x6B, 0x00,
    0x00, 0x0B, 0x29, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xFF, 0xFF, 0xF9, 0x00, 0x00, 0x00, 0x16, 0x00,
    0x00, 0x00, 0x1E, 0x01, 0x80, 0x7A, 0x89, 0x00, 0x00, 0x01, 0x17, 0x00, 0xFE, 0x43, 0xCE, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x99, 0x00, 0x88, 0x00, 0x47, 0x00, 0x70, 0x00, 0x4C, 0x00, 0x3F, 0x00,
    0x2B, 0x10, 0x60, 0x04, 0x22, 0xD5, 0x58, 0x97, 0x2A
  };

  const XsensGpsTranslator::Config cfg{};
  XsensGpsTranslator driver(cfg);
  pkt.data = 0xFA;
  ASSERT_FALSE(driver.convert(pkt, out));
  pkt.data = 0xFF;
  ASSERT_FALSE(driver.convert(pkt, out));
  pkt.data = static_cast<std::underlying_type_t<MID>>(MID::MT_DATA2);
  ASSERT_FALSE(driver.convert(pkt, out));
  uint8_t length = data.size() - 1;
  pkt.data = length;
  ASSERT_FALSE(driver.convert(pkt, out));

  for(uint8_t i = 0; i < length; ++i) {
    pkt.data = data[i];
    ASSERT_FALSE(driver.convert(pkt, out));
  }
  pkt.data = data[length];
  ASSERT_TRUE(driver.convert(pkt, out));

  ASSERT_FLOAT_EQ(37.4246, out.latitude);
  ASSERT_FLOAT_EQ(-122.10012, out.longitude);
  ASSERT_FLOAT_EQ(-16.188999, out.altitude);
}

int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
