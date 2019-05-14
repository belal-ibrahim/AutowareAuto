#ifndef NDT_MATCHING__REGISTRATION_HPP_
#define NDT_MATCHING__REGISTRATION_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ndt_matching/visibility_control.hpp>

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointInputType, typename PointMapType>
class NDT_MATCHING_PUBLIC NdtBase
{
public:
	NdtBase(
		const real_t resolution,
		const real_t step_size,
		const real_t translation_eps,
		const uint32_t maximum_iteration);

	virtual ~NdtBase();

	/* Set input Scanned point cloud.
	 * Simply set the point cloud input_ */
	// virtual void set_map(typename pcl::PointCloud<PointInputType>::Ptr input);

protected:

	virtual void compute_transformation(
		const typename pcl::PointCloud<PointInputType> & input_points,
		Eigen::Matrix<real_t, 4U, 4U> & pred_transform);

	real_t m_resolution;
	real_t m_step_size;
	real_t m_translation_eps;
	uint32_t m_maximum_iteration;
	uint32_t m_num_iteration;
};  // class NdtBase
}  // namespace ndt_matching
}  // namespace autoware_bridge

#endif  // NDT_MATCHING__REGISTRATION_HPP_
