// Copyright 2019 Apex.AI, Inc.
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

#ifndef NDT__NDT_MAP_HPP_
#define NDT__NDT_MAP_HPP_

#include <ndt/ndt_representations.hpp>
#include <voxel_grid/voxels.hpp>
#include <voxel_grid/voxel_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry/spatial_hash.hpp>
#include <vector>

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Voxel implementation for the NDT map. This class fuses the voxel and NDTNormal
/// APIs to fully represent an NDT grid cell
class NDT_PUBLIC NDTVoxel : public perception::filters::voxel_grid::CentroidVoxel<Eigen::Vector3d>,
  public NDTNormal<NDTVoxel>
{
public:
  using PointT = Eigen::Vector3d;

  // TODO(yunus.caliskan): make this configurable.
  // Set to value proposed in "Scan Registration for Autonomous Mining Vehicles Using 3D-NDT"
  static constexpr uint32_t NUM_POINT_THRESHOLD = 5U;

  /// This function hides the parent implementation and replaces it with the augmented logic
  /// for the ndt grid cell.
  /// \param pt Point to add to the voxel.
  void add_observation(const PointT & pt)
  {
    if (m_cov_computed) {
      m_covariance.setZero();
      m_cov_point_count = 0U;
      m_cov_computed = false;
    }
    // TODO(yunus.caliskan): Resolve the overload resolution issue!
    // compute the centroid
    // This section is identical to the CentroidVoxel. However calling
    // `CentroidVoxel<Eigen::Vector3d>::add_observation(pt);` overloeads eigen's arithmetic
    // operators due to overload resolution.
    const auto last = static_cast<float>(Voxel<PointT>::count());
    Voxel<PointT>::set_count(Voxel<PointT>::count() + 1U);
    const float count_inv = 1.0F / static_cast<float>(Voxel<PointT>::count());
    // Incremental update: u' = ((u * n) + x) / (n + 1), u = mean, x = obs, n = count
    const PointT centroid = ((Voxel<PointT>::get() * last) + pt) * count_inv;
    Voxel<PointT>::set_centroid(centroid);
  }

  /// Adds the effect of one point to the covariance of the voxel. All points residing in the
  /// voxel should be included in the covariance computation for a correct result.
  /// \param pt point to add for covariance calculation.
  void add_point_for_covariance(const PointT & pt)
  {
    const float inv_count = 1.0 / (count() - 1);
    const auto & centroid = centroid_();
    m_covariance += inv_count * ((pt - centroid) * ((pt - centroid).transpose()));
    ++m_cov_point_count;
    if (m_cov_point_count == count()) {
      m_cov_computed = true;
    }
  }

  // Hiding the original function with a modified version that uses a custom threshold for
  // interfacing with NDTMap
  bool occupied() const
  {
    return count() >= NUM_POINT_THRESHOLD;
  }

  /// Returns the covariance of the points in the voxel.
  /// \return covariance of the cell
  const Eigen::Matrix3d & covariance_() const
  {
    if (!occupied()) {
      throw std::out_of_range("NDTVoxel: Cannot get covariance from an unoccupied voxel");
    }
    if (count() != m_cov_point_count) {
      throw std::length_error("NDTVoxel: Not all points are used in the covariance computation. "
              "Make sure to call `add_point_for_covariance() for each point in the voxel.`");
    }
    return m_covariance;
  }
  /// Returns the mean of the points in the cell
  /// \return centroid of the cell
  const Eigen::Vector3d & centroid_() const
  {
    // Using the overloaded function as the parent function will use the hidden occupancy check
    if (!occupied()) {
      throw std::out_of_range("NDTVoxel: Cannot get centroid from an unoccupied voxel");
    }
    return get();
  }

  /// Returns true if enough points are used in the covariance computation.
  bool cov_computed() const
  {
    return m_cov_computed;
  }

private:
  size_t m_cov_point_count{0U};
  bool m_cov_computed{false};
  Eigen::Matrix3d m_covariance;
};
constexpr uint32_t NDTVoxel::NUM_POINT_THRESHOLD;
/////////////////////////////////////////////

class NDT_PUBLIC NDTVoxelMapOutput;

/// Class representing an NDT map. It utilizes a voxel grid for organizing the cells and a spatial
/// hash for managing the beighboring cells lookup. `NDTVoxelMapOutput` is used to wrap the output.
class NDT_PUBLIC NDTVoxelMap : public perception::filters::voxel_grid::VoxelGrid<NDTVoxel>,
  public NDTMapBase<NDTVoxelMap, NDTVoxel, NDTVoxelMapOutput>
{
public:
  using GridT = perception::filters::voxel_grid::VoxelGrid<NDTVoxel>;
  using HashT = common::geometry::spatial_hash::SpatialHash3d<GridT::IT>;

  /// Constructor
  /// \param voxel_grid_config config instance for the voxel grid
  /// \param hash_config config instance for the spatial hash
  NDTVoxelMap(
    const perception::filters::voxel_grid::Config & voxel_grid_config,
    const common::geometry::spatial_hash::Config3d & hash_config)
  : VoxelGrid(voxel_grid_config),
    m_hash(hash_config) {}

  /// Insert the point cloud to the map. It works by first adding the points to the cells
  /// they correspond to and then Adding the pointers to these cells to a spatial hash to allow
  /// for fast lookup later.
  /// \param msg PointCloud2 message to add.
  void insert(const sensor_msgs::msg::PointCloud2 & msg)
  {
    insert_to_grid(msg);
    update_hash();
  }
  /// Return the nearest neighbouring cells given coordinates. Search radius is set in the spatial
  /// hash config.
  /// \param x x coordinate
  /// \param y y coordinate
  /// \param z z coordinate
  /// \return A vector containing NDT voxels. Each element conforms to the API of NDTUnit
  const std::vector<NDTVoxelMapOutput> & cells_(float_t x, float_t y, float_t z)
  {
    return m_hash.near(x, y, z);
  }

  /// Clear the data from the map.
  void clear()
  {
    VoxelGrid::clear();
    m_hash.clear();
  }

private:
  void insert_to_grid(const sensor_msgs::msg::PointCloud2 & msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity_it(msg, "intensity");

    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end() &&
      intensity_it != intensity_it.end())
    {
      Eigen::Vector3d pt({*x_it, *y_it, *z_it});
      VoxelGrid::insert(pt);

      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
    }

    // TODO(yunus.caliskan): this pointcloud iteration mess will be refactored in #102
    x_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
    y_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
    z_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");
    intensity_it = sensor_msgs::PointCloud2ConstIterator<float>(msg, "intensity");

    // TODO(yunus.caliskan) explore incremental covariance calculation methods instead
    // Covariance has to be computed after the mean is computed. The pointcloud is iterated
    // for a second time instead of having and managing a history of points in the voxel.
    while (x_it != x_it.end() &&
      y_it != y_it.end() &&
      z_it != z_it.end() &&
      intensity_it != intensity_it.end())
    {
      Eigen::Vector3d pt({*x_it, *y_it, *z_it});
      auto & vx = get_voxel(pt);
      // Only compute covariance if there are enough points in a voxel.
      if (vx.occupied()) {
        vx.add_point_for_covariance(pt);
      }

      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
    }
  }

  void update_hash()
  {
    for (auto it = begin(); it != end(); ++it) {
      m_hash.insert(it);
    }
  }


// Hash holding the iterators instead of directly the voxels to avoid copying data
// Point adapters for the iterator are implemented for centroid lookup in spatial hash
  common::geometry::spatial_hash::SpatialHash3d<GridT::IT, NDTVoxelMapOutput> m_hash;
};

/// Wrapper for the Spatial Hash output to give direct access to the centroid and covariance.
class NDTVoxelMapOutput
  : private common::geometry::spatial_hash::HashOutput<NDTVoxelMap::GridT::IT>,
  public NDTNormal<NDTVoxelMapOutput>
{
public:
  // Using the parent's output
  using HashOutput::HashOutput;

  const NDTNormal<NDTVoxelMapOutput>::Point & centroid_() const
  {
    return get_point()->second.centroid();
  }

  const NDTNormal<NDTVoxelMapOutput>::Covariance & covariance_() const
  {
    return get_point()->second.covariance();
  }
};

}  // namespace ndt
}  // namespace localization

namespace common
{
namespace geometry
{
namespace point_adapter
{
/// Point adapters for eigen vector
template<>
float32_t x_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(0));
}

template<>
float32_t y_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(1));
}

template<>
float32_t z_(const Eigen::Vector3d & pt)
{
  return static_cast<float32_t>(pt(2));
}

template<typename ScalarT>
void set_point(Eigen::Vector3d & pt, ScalarT x, ScalarT y, ScalarT z)
{
// ScalarT is left unspecialized to allow implicit numeric casting/promotion
// instead of re-writing different implementations
  pt(0) = x;
  pt(1) = y;
  pt(2) = z;
}

/// Point adapters for NDTVoxel grid iterator.

template<>
float32_t x_(const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(0));
}
template<>
float32_t y_(const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(1));
}
template<>
float32_t z_(const perception::filters::voxel_grid::VoxelGrid<localization::ndt::NDTVoxel>::IT & pt)
{
  return static_cast<float32_t>(pt->second.get()(2));
}

}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // NDT__NDT_MAP_HPP_
