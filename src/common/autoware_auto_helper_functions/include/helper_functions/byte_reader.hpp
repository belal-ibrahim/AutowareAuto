// Copyright 2017-2019 Apex.AI, Inc.
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
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__BYTE_READER_HPP_
#define HELPER_FUNCTIONS__BYTE_READER_HPP_

#include <cstdint>
#include <vector>

namespace autoware
{
namespace common
{
namespace helper_functions
{
class ByteReader
{
private:
  const std::vector<uint8_t> & byte_vector_;
  std::size_t index_;

public:
  explicit ByteReader(const std::vector<uint8_t> & byte_vector)
  : byte_vector_(byte_vector),
    index_(0)
  {
  }

  template<typename T>
  void read(T & value)
  {
    std::size_t type_size = sizeof(T);
    for (std::size_t i = 0; i < type_size; ++i) {
      reinterpret_cast<uint8_t *>(&value)[i] = byte_vector_[index_ + type_size - 1 - i];
    }
    index_ += type_size;
  }

  void skip(std::size_t count)
  {
    index_ += count;
  }
};
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__BYTE_READER_HPP_
