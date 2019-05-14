#include <iostream>
#include "ndt_matching/ndt_base.hpp"

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointInputType, typename PointMapType>
NdtBase<PointInputType, PointMapType>::NdtBase(
  const real_t resolution,
  const real_t step_size,
  const real_t translation_eps,
  const uint32_t maximum_iteration)
: m_resolution(resolution),
  m_step_size(step_size),
  m_translation_eps(translation_eps),
  m_maximum_iteration(maximum_iteration),
	m_num_iteration(0U)
{}

template <typename PointInputType, typename PointMapType>
NdtBase<PointInputType, PointMapType>::~NdtBase()
{
	return;
}

template <typename PointInputType, typename PointMapType>
void NdtBase<PointInputType, PointMapType>::compute_transformation(
  const typename pcl::PointCloud<PointInputType> & input_points,
  Eigen::Matrix<real_t, 4U, 4U> & pred_transform)
{
	printf("Unsupported by NdtBase\n");
}

template class NdtBase<pcl::PointXYZI, pcl::PointXYZI>;
template class NdtBase<pcl::PointXYZ, pcl::PointXYZ>;
}  // namespace ndt_matching
}  // namespace autoware_bridge
