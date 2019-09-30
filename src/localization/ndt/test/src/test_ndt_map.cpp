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

#include <gtest/gtest.h>
#include <test_ndt_map.hpp>
#include <vector>
#include <set>
namespace autoware
{
namespace localization
{
namespace ndt
{


TEST(NDTVoxelTest, ndt_voxel_basic_io) {
  constexpr auto eps = 1e-6;

  NDTVoxel voxel;

  EXPECT_THROW(voxel.centroid(), std::out_of_range);
// Empty state corresponds to multiple failure states for the covariance function so `any_throw`
// is used to reflect that.
  EXPECT_ANY_THROW(voxel.covariance());
  EXPECT_EQ(voxel.count(), 0U);
  Eigen::Vector3d point({5, 5, 5});

  voxel.add_observation(point);

// voxel is considered unoccupied when it has less point than its point threshold
  if (NDTVoxel::NUM_POINT_THRESHOLD > 1U) {
    EXPECT_THROW(voxel.centroid(), std::out_of_range);
    EXPECT_ANY_THROW(voxel.covariance());
  }

// Add the same point until the voxel has sufficient number of points
  for (uint64_t i = 1U; i < NDTVoxel::NUM_POINT_THRESHOLD; i++) {
    voxel.add_observation(point);
  }

  EXPECT_EQ(voxel.count(), NDTVoxel::NUM_POINT_THRESHOLD);
// Centroid should equal to the point as we added the same point multiple times.
  EXPECT_TRUE(point.isApprox(voxel.centroid(), eps));
// Covariance cannot be computed until all points in the voxel are fed back to the voxel
// for covariance update
  EXPECT_THROW(voxel.covariance(), std::length_error);

// Update the covariance using all the points in the voxel but one.
  for (uint64_t i = 0U; i < NDTVoxel::NUM_POINT_THRESHOLD - 1; i++) {
    voxel.add_point_for_covariance(point);
  }

// covariance can't be accessed because one point is not included in the covariance yet.
  EXPECT_FALSE(voxel.cov_computed());
  EXPECT_THROW(voxel.covariance(), std::length_error);

// add the last point to the covariance calculation
  voxel.add_point_for_covariance(point);

// Covariance can now be computed. Its values are zero since all points are the same
// and there's no variance.
  EXPECT_TRUE(voxel.cov_computed());
  EXPECT_NO_THROW(
    EXPECT_LT(voxel.covariance().norm(), eps);
  );

// adding another point after the covariance computation is done which resets the covariance
// computation.
  voxel.add_observation(point);
  voxel.add_point_for_covariance(point);
// Covariance computation needs all the other points to compute the covariance from scratch
// since the computation is not incremental
  EXPECT_FALSE(voxel.cov_computed());
  EXPECT_THROW(voxel.covariance(), std::length_error);
}

///////////////////////////////////

TEST(NDTVoxelTest, ndt_voxel_basic) {
  constexpr auto eps = 1e-6;
  NDTVoxel voxel;
  auto num_points = 5U;
  EXPECT_GE(num_points, NDTVoxel::NUM_POINT_THRESHOLD);

  std::vector<Eigen::Vector3d> points;
  // Add points to the voxel ([0,0,0]... to [4,4,4])
  for (auto i = 0U; i < num_points; i++) {
    auto point =
      Eigen::Vector3d{static_cast<double>(i), static_cast<double>(i), static_cast<double>(i)};
    points.push_back(point);
    voxel.add_observation(point);
  }

  // Update voxel covariance with all the points
  for (const auto & point : points) {
    voxel.add_point_for_covariance(point);
  }

  // validate the mean in numpy
  // np.mean(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]),1)
  Eigen::Vector3d expected_centroid{2.0, 2.0, 2.0};
  // Validate covariance in numpy: np.cov(np.array([[0,1,2,3,4],[0,1,2,3,4],[0,1,2,3,4]]))
  Eigen::Matrix3d expected_covariance;
  expected_covariance.setConstant(2.5);

  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.centroid().isApprox(expected_centroid, eps));
  );
  EXPECT_NO_THROW(
    EXPECT_TRUE(voxel.covariance().isApprox(expected_covariance, eps));
  );
}


TEST_F(NDTMapTest, map_lookup) {
  constexpr auto eps = 1e-5;
  // The idea is to have a 5x5x5 grid with cell edge length of 1
  auto grid_config = perception::filters::voxel_grid::Config(m_min_point, m_max_point, m_voxel_size,
      m_capacity);

  // Spatial hash is set to have a radius of sqrt(3) + 0.1. This way the radius(~1.8) is big enough to cover its
  // diagonal neighbour but still less than 2.0, meaning it will cover all the adjacent cells including diagonal ones
  // but exclude non-=adjacent cells
  NDTVoxelMap ndt_map(grid_config,
    common::geometry::spatial_hash::Config3d(m_min_point.x, m_max_point.x, m_min_point.y,
    m_max_point.y,
    m_min_point.z, m_max_point.z,
    1.83, 1024U)
  );

  EXPECT_EQ(ndt_map.cells(1, 1, 1).size(), 0U);

  // build a pointcloud map.
  // It contains 5*5*5*7 points where each cell would have a center and 6 surrounding points with a
  // 0.3 distance from the center
  build_pc(grid_config);

  // The center points are added to a map with their voxel indices for easy lookup
  EXPECT_EQ(m_voxel_centers.size(), 125U);

  // Insert the pointcloud into the ndt map
  EXPECT_NO_THROW(ndt_map.insert(m_pc));
  // ndt map has 125 voxels now: a 5x5x5 grid
  EXPECT_EQ(ndt_map.size(), 125U);

  // Al cells have the same variance. The value can be validated via numpy:
  // >>> dev = 0.3
  // >>> np.cov(np.array([ [1, 1+dev, 1-dev,1,1,1,1], [1,1,1,1+dev,1-dev,1,1], [1,1,1,1,1,1+dev,
  // 1-dev]  ]))
  Eigen::Matrix3d expected_cov;
  expected_cov << 0.03, 0.0, 0.0,
    0.0, 0.03, 0.0,
    0.0, 0.0, 0.03;

  for (auto & voxel_it : ndt_map) {
    Eigen::Vector3d center;
    // Each voxel has 7 points
    EXPECT_EQ(voxel_it.second.count(), 7U);
    EXPECT_NO_THROW(center = voxel_it.second.centroid());
    // Check if the voxel centroid is the same as the intended centroid
    auto voxel_idx = grid_config.index(center);
    EXPECT_TRUE(m_voxel_centers[voxel_idx].isApprox(center, eps));
    // Check if covariance matches the pre-computed value
    EXPECT_NO_THROW(
      EXPECT_TRUE(voxel_it.second.covariance().isApprox(expected_cov, eps))
    );
  }

  // Iterate the grid and do lookups:

  for (auto x = 1; x <= POINTS_PER_DIM; x++) {
    for (auto y = 1; y <= POINTS_PER_DIM; y++) {
      for (auto z = 1; z <= POINTS_PER_DIM; z++) {
        // Get a list of adjacent cells in the grid manually.
        auto neighbor_indices = get_neighbours(x, y, z, m_min_point, m_max_point, grid_config);
        // Do a distance based lookup in the map
        auto cells = ndt_map.cells(x, y, z);
        // Convert the lookup result to a list of voxel IDs for comparison
        std::set<uint64_t> cell_indices;
        for (const auto & cell : cells) {
          cell_indices.insert(grid_config.index(cell.centroid()));
        }
        // Map lookup returned a list of adjacent cells.
        EXPECT_EQ(neighbor_indices, cell_indices);
      }
    }
  }
}


}  // namespace ndt
}  // namespace localization
}  // namespace autoware
