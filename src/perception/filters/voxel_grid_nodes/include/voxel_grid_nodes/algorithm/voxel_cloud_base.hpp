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
/// \brief This file defines the algorithmic interface for applying voxel grid downsampling to a
///        PointCloud2 message
#ifndef VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_BASE_HPP_
#define VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_BASE_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxel_grid/voxel_grid.hpp>
#include <voxel_grid_nodes/visibility_control.hpp>
#include <string>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
/// \brief Polymorphic algorithms for dynamically dispatching to voxel grids for use with various
///        message types
namespace algorithm
{
/// \brief A pure interface meant for dynamically dispatching to different voxel grid instances
///        at runtime. Specialized for PointCloud2 types
class VOXEL_GRID_NODES_PUBLIC VoxelCloudBase
{
public:
  /// \brief Virtual destructor
  virtual ~VoxelCloudBase();
  /// \brief Inserts points into the voxel grid data structure, overwrites internal header
  /// \param[in] msg A point cloud to insert into the voxel grid. Assumed to have the structure XYZI
  virtual void insert(const sensor_msgs::msg::PointCloud2 & msg) = 0;

  /// \brief Get accumulated downsampled points. Internally resets the internal grid. Header is
  ///        taken from last insert
  /// \return The downsampled point cloud
  virtual const sensor_msgs::msg::PointCloud2 & get() = 0;

protected:
  using PointXYZIF = autoware::perception::filters::voxel_grid::PointXYZIF;

  /// \brief Initializes a given PointCloud2
  /// \param[in] msg The point cloud to initialize
  /// \param[in] frame_id frame_id of the PointCloud2 message
  /// \param[in] size Desired size of the PointCloud2 message
  void init_pcl_msg(
    sensor_msgs::msg::PointCloud2 & msg,
    const std::string & frame_id,
    const std::size_t size);

  /// \brief Adds a point to a given PointCloud2
  /// \param[in] cloud The point cloud to add the point into
  /// \param[in] pt point to add
  /// \return Boolean showing the success of the adding operation
  bool add_point_to_cloud(
    sensor_msgs::msg::PointCloud2 & cloud,
    const PointXYZIF & pt);

  /// \brief Resets the offset of the PointCloud2 iterators
  void reset_cloud_idx();

private:
  /// \brief The offset to be used with the PointCloud2 iterators
  int32_t m_point_cloud_idx{0};
};  // VoxelCloudBase
}  // namespace algorithm
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // VOXEL_GRID_NODES__ALGORITHM__VOXEL_CLOUD_BASE_HPP_