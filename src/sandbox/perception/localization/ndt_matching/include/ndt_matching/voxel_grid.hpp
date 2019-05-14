#ifndef NDT_MATCHING__VOXEL_GRID_HPP_
#define NDT_MATCHING__VOXEL_GRID_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ndt_matching/visibility_control.hpp>
#include <apexcpp/apexcpp.hpp>

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointSourceType>
class NDT_MATCHING_PUBLIC VoxelGrid {
public:
	VoxelGrid();

	/* Set input points */
	void setInput(typename pcl::PointCloud<PointSourceType>::Ptr input);

	/* For each input point, search for voxels whose distance between their centroids and
	 * the input point are less than radius.
	 * The output is a list of candidate voxel ids */
	void radius_search(
		const PointSourceType & query_point,
		const real_t radius,
		std::vector<int32_t> & voxel_ids,
		const int32_t max_nn=INT_MAX);

	int32_t getVoxelNum() const;

	real_t getMaxX() const;
	real_t getMaxY() const;
	real_t getMaxZ() const;

	real_t getMinX() const;
	real_t getMinY() const;
	real_t getMinZ() const;

	real_t getVoxelX() const;
	real_t getVoxelY() const;
	real_t getVoxelZ() const;

	int32_t getMaxBX() const;
	int32_t getMaxBY() const;
	int32_t getMaxBZ() const;

	int32_t getMinBX() const;
	int32_t getMinBY() const;
	int32_t getMinBZ() const;

	int32_t getVgridX() const;
	int32_t getVgridY() const;
	int32_t getVgridZ() const;

	void setLeafSize(real_t voxel_x, real_t voxel_y, real_t voxel_z);

	/* Searching for the nearest point of each input query point.
	 * Return the distance between the query point and its nearest neighbor.
	 * If the distance is larger than max_range, then return DBL_MAX. */

	real_t nearestNeighborDistance(PointSourceType query_point, real_t max_range);

	Vector3 getCentroid(int32_t voxel_id) const;
	Matrix3 getCovariance(int32_t voxel_id) const;
	Matrix3 getInverseCovariance(int32_t voxel_id) const;

private:

	/* Construct the voxel grid and the build the octree. */
	void initialize();

	/* Put points into voxels */
	void scatterPointsToVoxelGrid();

	/* Compute centroids and covariances of voxels. */
	void computeCentroidAndCovariance();

	/* Find boundaries of input point cloud and compute
	 * the number of necessary voxels as well as boundaries
	 * measured in number of leaf size */
	void findBoundaries();

	void findBoundaries(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud,
							real_t &max_x, real_t &max_y, real_t &max_z,
							real_t &min_x, real_t &min_y, real_t &min_z);

	int32_t voxelId(PointSourceType p);

	int32_t voxelId(int32_t idx, int32_t idy, int32_t idz,
				int32_t min_b_x, int32_t min_b_y, int32_t min_b_z,
				int32_t size_x, int32_t size_y, int32_t size_z);

	void vid_to_anchor(int32_t vid, PointSourceType & anchor_p);

	int32_t nearestVoxel(PointSourceType query_point, Eigen::Matrix<real_t, 6, 1> boundaries, real_t max_range);

	int32_t roundUp(int32_t input, int32_t factor);

	int32_t roundDown(int32_t input, int32_t factor);

	int32_t div(int32_t input, int32_t divisor);

	//Coordinate of input points
	typename pcl::PointCloud<PointSourceType>::Ptr m_input_points;

	int32_t voxel_num_;						// Number of voxels
	real_t max_x_, max_y_, max_z_;		// Upper bounds of the grid (maximum coordinate)
	real_t min_x_, min_y_, min_z_;		// Lower bounds of the grid (minimum coordinate)
	real_t voxel_x_, voxel_y_, voxel_z_;	// Leaf size, a.k.a, size of each voxel

	int32_t max_b_x_, max_b_y_, max_b_z_;	// Upper bounds of the grid, measured in number of voxels
	int32_t min_b_x_, min_b_y_, min_b_z_;	// Lower bounds of the grid, measured in number of voxels
	int32_t vgrid_x_, vgrid_y_, vgrid_z_;	// Size of the voxel grid, measured in number of voxels
  int32_t min_points_per_voxel_;			// Minimum number of points per voxel. If the number of points
										// per voxel is less than this number, then the voxel is ignored
										// during computation (treated like it contains no point)

	boost::shared_ptr<std::vector<Vector3>> centroid_;			// 3x1 Centroid vectors of voxels
	boost::shared_ptr<std::vector<Matrix3>> icovariance_;		// Inverse covariance matrixes of voxel
	boost::shared_ptr<std::vector<std::vector<int32_t>>> points_id_;		// Indexes of points belong to each voxel
	boost::shared_ptr<std::vector<int32_t> > points_per_voxel_;				// Number of points belong to each voxel
													// (may differ from size of each vector in points_id_
													// because of changes made during computing covariances
	boost::shared_ptr<std::vector<Vector3> > tmp_centroid_;
	boost::shared_ptr<std::vector<Matrix3> > tmp_cov_;

	int32_t real_max_bx_, real_max_by_, real_max_bz_;
	int32_t real_min_bx_, real_min_by_, real_min_bz_;

	static const int32_t MAX_BX_ = 16;
	static const int32_t MAX_BY_ = 16;
	static const int32_t MAX_BZ_ = 8;
};  // class VoxelGrid
}  // namespace ndt_matching
}  // namespace autoware_bridge

#endif  // NDT_MATCHING__VOXEL_GRID_HPP_
