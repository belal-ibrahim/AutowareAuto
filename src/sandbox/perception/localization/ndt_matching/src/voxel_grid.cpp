#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>
#include "ndt_matching/voxel_grid.hpp"
#include "ndt_matching/symmetric_eigen_solver.hpp"

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointSourceType>
VoxelGrid<PointSourceType>::VoxelGrid():
	voxel_num_(0),
	max_x_(std::numeric_limits<real_t>::min()),
	max_y_(std::numeric_limits<real_t>::min()),
	max_z_(std::numeric_limits<real_t>::min()),
	min_x_(std::numeric_limits<real_t>::max()),
	min_y_(std::numeric_limits<real_t>::max()),
	min_z_(std::numeric_limits<real_t>::max()),
	voxel_x_(0),
	voxel_y_(0),
	voxel_z_(0),
	max_b_x_(0),
	max_b_y_(0),
	max_b_z_(0),
	min_b_x_(0),
	min_b_y_(0),
	min_b_z_(0),
	vgrid_x_(0),
	vgrid_y_(0),
	vgrid_z_(0),
	min_points_per_voxel_(6),
	real_max_bx_(INT_MIN),
	real_max_by_(INT_MIN),
	real_max_bz_(INT_MIN),
	real_min_bx_(INT_MAX),
	real_min_by_(INT_MAX),
	real_min_bz_(INT_MAX)
{
	centroid_.reset();
	icovariance_.reset();
	points_id_.reset();
	points_per_voxel_.reset();
	tmp_centroid_.reset();
	tmp_cov_.reset();
};

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::roundUp(int32_t input, int32_t factor)
{
	return (input < 0) ? -((-input) / factor) * factor : ((input + factor - 1) / factor) * factor;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::roundDown(int32_t input, int32_t factor)
{
	return (input < 0) ? -((-input + factor - 1) / factor) * factor : (input / factor) * factor;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::div(int32_t input, int32_t divisor)
{
	return (input < 0) ? -((-input + divisor - 1) / divisor) : input / divisor;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::initialize()
{
	centroid_.reset();
	centroid_ = boost::make_shared<std::vector<Vector3> >(voxel_num_);

	icovariance_.reset();
	icovariance_ = boost::make_shared<std::vector<Matrix3> >(voxel_num_);

	points_id_.reset();
	points_id_ = boost::make_shared<std::vector<std::vector<int32_t> > >(voxel_num_);

	points_per_voxel_.reset();
	points_per_voxel_ = boost::make_shared<std::vector<int32_t> >(voxel_num_, 0);

	tmp_centroid_.reset();
	tmp_centroid_ = boost::make_shared<std::vector<Vector3> >(voxel_num_);

	tmp_cov_.reset();
	tmp_cov_ = boost::make_shared<std::vector<Matrix3> >(voxel_num_);
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getVoxelNum() const
{
	return voxel_num_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMaxX() const
{
	return max_x_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMaxY() const
{
	return max_y_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMaxZ() const
{
	return max_z_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMinX() const
{
	return min_x_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMinY() const
{
	return min_y_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getMinZ() const
{
	return min_z_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getVoxelX() const
{
	return voxel_x_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getVoxelY() const
{
	return voxel_y_;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::getVoxelZ() const
{
	return voxel_z_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMaxBX() const
{
	return max_b_x_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMaxBY() const
{
	return max_b_y_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMaxBZ() const
{
	return max_b_z_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMinBX() const
{
	return min_b_x_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMinBY() const
{
	return min_b_y_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getMinBZ() const
{
	return min_b_z_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getVgridX() const
{
	return vgrid_x_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getVgridY() const
{
	return vgrid_y_;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::getVgridZ() const
{
	return vgrid_z_;
}

template <typename PointSourceType>
Vector3 VoxelGrid<PointSourceType>::getCentroid(int32_t voxel_id) const
{
	return (*centroid_)[voxel_id];
}

template <typename PointSourceType>
Matrix3 VoxelGrid<PointSourceType>::getInverseCovariance(int32_t voxel_id) const
{
	return (*icovariance_)[voxel_id];
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::setLeafSize(real_t voxel_x, real_t voxel_y, real_t voxel_z)
{
	voxel_x_ = voxel_x;
	voxel_y_ = voxel_y;
	voxel_z_ = voxel_z;
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::voxelId(PointSourceType p)
{
	int32_t idx = static_cast<int32_t>(floor(p.x / voxel_x_)) - min_b_x_;
	int32_t idy = static_cast<int32_t>(floor(p.y / voxel_y_)) - min_b_y_;
	int32_t idz = static_cast<int32_t>(floor(p.z / voxel_z_)) - min_b_z_;

	return (idx + idy * vgrid_x_ + idz * vgrid_x_ * vgrid_y_);
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::voxelId(int32_t idx, int32_t idy, int32_t idz,
										int32_t min_b_x, int32_t min_b_y, int32_t min_b_z,
										int32_t size_x, int32_t size_y, int32_t size_z)
{
	return (idx - min_b_x) + (idy - min_b_y) * size_x + (idz - min_b_z) * size_x * size_y;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::computeCentroidAndCovariance()
{
	for (int32_t idx = real_min_bx_; idx <= real_max_bx_; idx++)
		for (int32_t idy = real_min_by_; idy <= real_max_by_; idy++)
			for (int32_t idz = real_min_bz_; idz <= real_max_bz_; idz++) {
				int32_t i = voxelId(idx, idy, idz, min_b_x_, min_b_y_, min_b_z_, vgrid_x_, vgrid_y_, vgrid_z_);
				int32_t ipoint_num = (*points_id_)[i].size();
				real_t point_num = static_cast<real_t>(ipoint_num);
				Vector3 pt_sum = (*tmp_centroid_)[i];

				if (ipoint_num > 0) {
					(*centroid_)[i] = pt_sum / point_num;
				}

				Matrix3 covariance;

				if (ipoint_num >= min_points_per_voxel_) {
					covariance = ((*tmp_cov_)[i] - 2.0 * (pt_sum * (*centroid_)[i].transpose())) / point_num + (*centroid_)[i] * (*centroid_)[i].transpose();
					covariance *= (point_num - 1.0) / point_num;

					PointSourceType anchor_p;
					vid_to_anchor(i, anchor_p);
					Vector3 vec_anchor_p(anchor_p.x, anchor_p.y, anchor_p.z);
					(*centroid_)[i] += vec_anchor_p;

					SymmetricEigensolver3x3 sv(covariance);

					sv.compute();
					Matrix3 evecs = sv.eigenvectors();
					Matrix3 evals = sv.eigenvalues().asDiagonal();

					if (evals(0, 0) < 0 || evals(1, 1) < 0 || evals(2, 2) <= 0) {
						(*points_per_voxel_)[i] = -1;
						continue;
					}

					real_t min_cov_eigvalue = evals(2, 2) * 0.01;

					if (evals(0, 0) < min_cov_eigvalue) {
						evals(0, 0) = min_cov_eigvalue;

						if (evals(1, 1) < min_cov_eigvalue) {
							evals(1, 1) = min_cov_eigvalue;
						}

						covariance = evecs * evals * evecs.inverse();
					}

					(*icovariance_)[i] = covariance.inverse();
				}
			}
}

//Input are supposed to be in device memory
template <typename PointSourceType>
void VoxelGrid<PointSourceType>::setInput(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud)
{
	if (input_cloud->points.size() > 0U) {
		/* If no voxel grid was created, then
		 * build the initial voxel grid and octree
		 */
		m_input_points = input_cloud;

		findBoundaries();

		std::vector<Eigen::Vector3i> voxel_ids(input_cloud->points.size());

		for (int32_t i = 0; i < input_cloud->points.size(); i++) {
			Eigen::Vector3i &vid = voxel_ids[i];
			PointSourceType p = input_cloud->points[i];

			vid(0) = static_cast<int32_t>(floor(p.x / voxel_x_));
			vid(1) = static_cast<int32_t>(floor(p.y / voxel_y_));
			vid(2) = static_cast<int32_t>(floor(p.z / voxel_z_));
		}

		voxel_ids.clear();

		initialize();

		scatterPointsToVoxelGrid();

		computeCentroidAndCovariance();
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::findBoundaries()
{

	findBoundaries(m_input_points, max_x_, max_y_, max_z_, min_x_, min_y_, min_z_);

	APEX_PRINT("Voxel boundaries");
	APEX_PRINT((apex::to_string(max_x_) + " " + apex::to_string(max_y_) + " " + apex::to_string(max_z_) + " " +
              apex::to_string(min_x_) + " " + apex::to_string(min_x_) + " " + apex::to_string(min_z_)).c_str());

	real_max_bx_ = max_b_x_ = static_cast<int32_t> (floor(max_x_ / voxel_x_));
	real_max_by_ = max_b_y_ = static_cast<int32_t> (floor(max_y_ / voxel_y_));
	real_max_bz_ = max_b_z_ = static_cast<int32_t> (floor(max_z_ / voxel_z_));

	real_min_bx_ = min_b_x_ = static_cast<int32_t> (floor(min_x_ / voxel_x_));
	real_min_by_ = min_b_y_ = static_cast<int32_t> (floor(min_y_ / voxel_y_));
	real_min_bz_ = min_b_z_ = static_cast<int32_t> (floor(min_z_ / voxel_z_));

	/* Allocate a poll of memory that is larger than the requested memory so
	 * we do not have to reallocate buffers when the target cloud is set
	 */
	/* Max bounds round toward plus infinity */
	max_b_x_ = roundUp(max_b_x_, MAX_BX_);
	max_b_y_ = roundUp(max_b_y_, MAX_BY_);
	max_b_z_ = roundUp(max_b_z_, MAX_BZ_);

	/* Min bounds round toward minus infinity */
	min_b_x_ = roundDown(min_b_x_, MAX_BX_);
	min_b_y_ = roundDown(min_b_y_, MAX_BY_);
	min_b_z_ = roundDown(min_b_z_, MAX_BZ_);

	vgrid_x_ = max_b_x_ - min_b_x_ + 1;
	vgrid_y_ = max_b_y_ - min_b_y_ + 1;
	vgrid_z_ = max_b_z_ - min_b_z_ + 1;

	if (vgrid_x_ > 0 && vgrid_y_ > 0 && vgrid_z_ > 0) {
		voxel_num_ = vgrid_x_ * vgrid_y_ * vgrid_z_;
	} else {
		voxel_num_ = 0;
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::findBoundaries(typename pcl::PointCloud<PointSourceType>::Ptr input_cloud,
													real_t &max_x, real_t &max_y, real_t &max_z,
													real_t &min_x, real_t &min_y, real_t &min_z)
{

	max_x = max_y = max_z = -std::numeric_limits<real_t>::max();
	min_x = min_y = min_z = std::numeric_limits<real_t>::max();

	for (int32_t i = 0; i < input_cloud->points.size(); i++) {
		real_t x = input_cloud->points[i].x;
		real_t y = input_cloud->points[i].y;
		real_t z = input_cloud->points[i].z;

		max_x = (max_x > x) ? max_x : x;
		max_y = (max_y > y) ? max_y : y;
		max_z = (max_z > z) ? max_z : z;

		min_x = (min_x < x) ? min_x : x;
		min_y = (min_y < y) ? min_y : y;
		min_z = (min_z < z) ? min_z : z;
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::radius_search(
	const PointSourceType & p,
	const real_t radius,
	std::vector<int32_t> & voxel_ids,
	const int32_t max_nn)
{
	real_t t_x = p.x;
	real_t t_y = p.y;
	real_t t_z = p.z;

	// std::cout << "------------------------------------\n";
	// std::cout << "t_x: " << t_x << ", t_y: " << t_y << ", t_z: " << t_z << "\n";
	// std::cout << "voxel_x_: " << voxel_x_ << ", voxel_y_: " << voxel_y_ << ", voxel_z_: " << voxel_z_ << "\n";

	int32_t max_id_x = static_cast<int32_t>(floor((t_x + radius) / voxel_x_));
	int32_t max_id_y = static_cast<int32_t>(floor((t_y + radius) / voxel_y_));
	int32_t max_id_z = static_cast<int32_t>(floor((t_z + radius) / voxel_z_));

	int32_t min_id_x = static_cast<int32_t>(floor((t_x - radius) / voxel_x_));
	int32_t min_id_y = static_cast<int32_t>(floor((t_y - radius) / voxel_y_));
	int32_t min_id_z = static_cast<int32_t>(floor((t_z - radius) / voxel_z_));

	// std::cout << "max_id_x: " << max_id_x << ", max_id_y: " << max_id_y << ", max_id_z: " << max_id_z << "\n";
	// std::cout << "min_id_x: " << min_id_x << ", min_id_y: " << min_id_y << ", min_id_z: " << min_id_z << "\n";
	// std::cout << "real_max_bx: " << real_max_bx_ << ", real_max_by: " << real_max_by_ << ", real_max_bz: " << real_max_bz_ << "\n";
	// std::cout << "real_min_bx: " << real_min_bx_ << ", real_min_by: " << real_min_by_ << ", real_min_bz: " << real_min_bz_ << "\n";

	// Find intersection of the cube containing the NN sphere of the point and the voxel grid
	max_id_x = (max_id_x > real_max_bx_) ? real_max_bx_ : max_id_x;
	max_id_y = (max_id_y > real_max_by_) ? real_max_by_ : max_id_y;
	max_id_z = (max_id_z > real_max_bz_) ? real_max_bz_ : max_id_z;

	min_id_x = (min_id_x < real_min_bx_) ? real_min_bx_ : min_id_x;
	min_id_y = (min_id_y < real_min_by_) ? real_min_by_ : min_id_y;
	min_id_z = (min_id_z < real_min_bz_) ? real_min_bz_ : min_id_z;
	int32_t nn = 0;
	for (int32_t idx = min_id_x; idx <= max_id_x && nn < max_nn; idx++) {
		for (int32_t idy = min_id_y; idy <= max_id_y && nn < max_nn; idy++) {
			for (int32_t idz = min_id_z; idz <= max_id_z && nn < max_nn; idz++) {
				int32_t vid = voxelId(idx, idy, idz,
									min_b_x_, min_b_y_, min_b_z_,
									vgrid_x_, vgrid_y_, vgrid_z_);

				if ((*points_per_voxel_)[vid] >= min_points_per_voxel_) {
					real_t cx = (*centroid_)[vid](0) - static_cast<real_t>(t_x);
					real_t cy = (*centroid_)[vid](1) - static_cast<real_t>(t_y);
					real_t cz = (*centroid_)[vid](2) - static_cast<real_t>(t_z);

					real_t distance = sqrtf(cx * cx + cy * cy + cz * cz);

					if (distance < radius) {
						nn++;
						voxel_ids.push_back(vid);
					}
				}
			}
		}
	}
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::vid_to_anchor(
	int32_t vid, PointSourceType & anchor_p)
{
	const int32_t idx = vid % vgrid_x_;
	const int32_t idy = (vid % (vgrid_x_ * vgrid_y_)) / vgrid_y_;
	const int32_t idz = vid / (vgrid_x_ * vgrid_y_);
	anchor_p.x = (idx + min_b_x_) * voxel_x_;
	anchor_p.y = (idy + min_b_y_) * voxel_y_;
	anchor_p.z = (idz + min_b_z_) * voxel_z_;
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::scatterPointsToVoxelGrid()
{

	for (int32_t pid = 0U; pid < m_input_points->points.size(); pid++) {
		const int32_t vid = voxelId(m_input_points->points[pid]);
		PointSourceType p = m_input_points->points[pid];

		PointSourceType anchor_p;
		vid_to_anchor(vid, anchor_p);
		Vector3 p3f(p.x - anchor_p.x, p.y - anchor_p.y, p.z - anchor_p.z);

		if ((*points_id_)[vid].size() == 0) {
			(*centroid_)[vid].setZero();
			(*points_per_voxel_)[vid] = 0;
			(*tmp_centroid_)[vid].setZero();
			(*tmp_cov_)[vid].setIdentity();
		}

		(*tmp_centroid_)[vid] += p3f;
		(*tmp_cov_)[vid] += p3f * p3f.transpose();
		(*points_id_)[vid].push_back(pid);
		(*points_per_voxel_)[vid]++;
	}
}

template <typename PointSourceType>
int32_t VoxelGrid<PointSourceType>::nearestVoxel(PointSourceType query_point, Eigen::Matrix<real_t, 6, 1> boundaries, real_t max_range)
{
	// Index of the origin of the circle (query point)
	real_t qx = query_point.x;
	real_t qy = query_point.y;
	real_t qz = query_point.z;

	int32_t lower_x = static_cast<int32_t>(floor(boundaries(0) / voxel_x_));
	int32_t lower_y = static_cast<int32_t>(floor(boundaries(1) / voxel_y_));
	int32_t lower_z = static_cast<int32_t>(floor(boundaries(2) / voxel_z_));

	int32_t upper_x = static_cast<int32_t>(floor(boundaries(3) / voxel_x_));
	int32_t upper_y = static_cast<int32_t>(floor(boundaries(4) / voxel_y_));
	int32_t upper_z = static_cast<int32_t>(floor(boundaries(5) / voxel_z_));

	real_t min_dist = std::numeric_limits<real_t>::max();
	int32_t nn_vid = -1;

	for (int32_t i = lower_x; i <= upper_x; i++) {
		for (int32_t j = lower_y; j <= upper_y; j++) {
			for (int32_t k = lower_z; k <= upper_z; k++) {
				int32_t vid = voxelId(i, j, k, min_b_x_, min_b_y_, min_b_z_, vgrid_x_, vgrid_y_, vgrid_z_);
				Vector3 c = (*centroid_)[vid];

				if ((*points_id_)[vid].size() > 0) {
					real_t cur_dist = sqrtf((qx - c(0)) * (qx - c(0)) + (qy - c(1)) * (qy - c(1)) + (qz - c(2)) * (qz - c(2)));

					if (cur_dist < min_dist) {
						min_dist = cur_dist;
						nn_vid = vid;
					}
				}
			}
		}
	}
	return nn_vid;
}

template <typename PointSourceType>
real_t VoxelGrid<PointSourceType>::nearestNeighborDistance(PointSourceType q, real_t max_range)
{
	throw std::runtime_error("nearestNeighborDistance is not implemented\n");

	// Eigen::Matrix<real_t, 6, 1> nn_node_bounds;
	//
	// nn_node_bounds = octree_.nearestOctreeNode(q);
	//
	// int32_t nn_vid = nearestVoxel(q, nn_node_bounds, max_range);
	//
	// Vector3 c = (*centroid_)[nn_vid];
	// real_t min_dist = sqrtf((q.x - c(0)) * (q.x - c(0)) + (q.y - c(1)) * (q.y - c(1)) + (q.z - c(2)) * (q.z - c(2)));
	//
	// if (min_dist >= max_range) {
	// 	return std::numeric_limits<real_t>::max();
	// }
	//
	// return min_dist;
}

template class VoxelGrid<pcl::PointXYZI>;
template class VoxelGrid<pcl::PointXYZ>;

}  // namespace ndt_matching
}  // namespace autoware_bridge
