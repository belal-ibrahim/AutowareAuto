#include <cmath>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "ndt_matching/ndt_matching.hpp"

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointInputType, typename PointMapType>
NdtMatching<PointInputType, PointMapType>::NdtMatching(
	const Config & cfg)
: NdtBase<PointInputType, PointMapType>(
	  cfg.get_resolution(),
		cfg.get_step_size(),
		cfg.get_translation_eps(),
	  cfg.get_maximum_iteration()),
	m_outlier_ratio(0.55F),
	m_trans_probability(0.0F),
	m_initialized(false),
	m_current_pose(),
	m_prev_pose()
{
  typename pcl::PointCloud<PointMapType>::Ptr map_ptr(new pcl::PointCloud<PointMapType>);
	if (pcl::io::loadPCDFile<PointMapType> (cfg.get_map_file(), *map_ptr) == -1) {
    APEX_PRINT("Failed to load map");
		exit(0);
  }
	APEX_PRINT("Success loading from file");
	APEX_PRINT(("Point size: " + apex::to_string(map_ptr->points.size())).c_str());

	set_map(map_ptr);

	APEX_PRINT("Success initializing the voxels");

	// Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
	const real_t gauss_c1 = 10.0F * (1.0F - m_outlier_ratio);
	const real_t gauss_c2 = m_outlier_ratio / powf(m_resolution, 3.0F);
	const real_t gauss_d3 = -logf(gauss_c2);
	m_gauss_d1 = -logf(gauss_c1 + gauss_c2) - gauss_d3;
	m_gauss_d2 = -2.0F * logf((-logf(gauss_c1 * expf(-0.5F) + gauss_c2) - gauss_d3) / m_gauss_d1);
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::set_map(
	const typename pcl::PointCloud<PointMapType>::Ptr input)
{
	// Build the voxel grid
	if (input->points.size() > 0U) {
		m_voxel_grid.setLeafSize(m_resolution, m_resolution, m_resolution);
		m_voxel_grid.setInput(input);
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::set_initial_pose(
	const real_t x, const real_t y, const real_t z,
	const real_t yaw, const real_t pitch, const real_t roll)
{
	m_prev_pose.set_values(x, y, z, yaw, pitch, roll);
	m_current_pose.set_values(x, y, z, yaw, pitch, roll);
	m_initialized = true;
	APEX_PRINT("Pose is initialized");
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::set_odometry(
	const nav_msgs::msg::Odometry & msg)
{
	throw std::runtime_error("set_odometry: Not implemented");
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
const geometry_msgs::msg::PoseStamped NdtMatching<PointInputType, PointMapType>::update(
	const sensor_msgs::msg::PointCloud2 & msg)
{
	pcl::PointCloud<PointInputType> points;
	APEX_PRINT("Before conversion");
	convert_ros2_to_pcl(msg, points);
	APEX_PRINT("After conversion");

  Eigen::Matrix4f tf_baselink_to_lidar;
	Eigen::Translation3f tl_btol(0.18F, 0.02F, 1.48F);  // x, y, z
  Eigen::AngleAxisf rot_x_btol(0.0F, Vector3::UnitX());  // roll
  Eigen::AngleAxisf rot_y_btol(0.0F, Vector3::UnitY());  // pitch
  Eigen::AngleAxisf rot_z_btol(3.16, Vector3::UnitZ());  // yaw
  tf_baselink_to_lidar = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

	Eigen::Transform<real_t, 3U, 1U> baselink_transform;
	m_current_pose.create_matrix_from_values(baselink_transform);
	Eigen::Matrix4f lidar_transform;
	lidar_transform = baselink_transform * tf_baselink_to_lidar;

	APEX_PRINT("Before align");
	std::cout << "Before align\n";
	std::cout << lidar_transform << "\n";

	compute_transformation(points, lidar_transform);

	APEX_PRINT("After align");
	std::cout << "After align\n";
	std::cout << lidar_transform << "\n";

	const Eigen::Matrix4f current_matrix = lidar_transform * tf_baselink_to_lidar.inverse();
	m_current_pose.set_values_from_matrix(current_matrix);
	m_prev_pose = m_current_pose;

	geometry_msgs::msg::PoseStamped pose_stamped;
	m_current_pose.create_pose_stamped(pose_stamped);
	pose_stamped.header.stamp = msg.header.stamp;
	return pose_stamped;
	// convert_pcl_to_ros2(points, msg);
	// APEX_PRINT("Undo");
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::convert_ros2_to_pcl(
	const sensor_msgs::msg::PointCloud2 & msg,
	pcl::PointCloud<PointInputType> & pcl_points)
{
	pcl::PCLPointCloud2 pcl_pc2;
	// pcl_stamp = stamp.toNSec() / 1000ull;
	// pcl_pc2.header.seq = msg.header.seq; // No seq in ROS2
	pcl_pc2.header.frame_id = msg.header.frame_id;
	pcl_pc2.height = msg.height;
	pcl_pc2.width = msg.width;
	pcl_pc2.fields.resize(msg.fields.size());
	int32_t idx = 0U;
	for (std::vector<sensor_msgs::msg::PointField>::const_iterator it = msg.fields.begin();
	     it != msg.fields.end(); ++it, ++idx) {
		pcl_pc2.fields[idx].name = (*it).name;
		pcl_pc2.fields[idx].offset = (*it).offset;
		pcl_pc2.fields[idx].datatype = (*it).datatype;
		pcl_pc2.fields[idx].count = (*it).count;
	}
	pcl_pc2.is_bigendian = msg.is_bigendian;
	pcl_pc2.point_step = msg.point_step;
	pcl_pc2.row_step = msg.row_step;
	pcl_pc2.is_dense = msg.is_dense;
	pcl_pc2.data = msg.data;
	pcl::fromPCLPointCloud2(pcl_pc2, pcl_points);
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::convert_pcl_to_ros2(
	const pcl::PointCloud<PointInputType> & pcl_points,
	sensor_msgs::msg::PointCloud2 & msg)
{
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::toPCLPointCloud2(pcl_points, pcl_pc2);
	// pcl_stamp = stamp.toNSec() / 1000ull;
	// pcl_pc2.header.seq = msg.header.seq; // No seq in ROS2
	msg.header.frame_id = pcl_pc2.header.frame_id;
	msg.height = pcl_pc2.height;
	msg.width = pcl_pc2.width;
	msg.fields.resize(pcl_pc2.fields.size());
	int32_t idx = 0U;
	for (std::vector<pcl::PCLPointField>::const_iterator it = pcl_pc2.fields.begin();
	     it != pcl_pc2.fields.end(); ++it, ++idx) {
		msg.fields[idx].name = (*it).name;
		msg.fields[idx].offset = (*it).offset;
		msg.fields[idx].datatype = (*it).datatype;
		msg.fields[idx].count = (*it).count;
	}
	msg.is_bigendian = pcl_pc2.is_bigendian;
	msg.point_step = pcl_pc2.point_step;
	msg.row_step = pcl_pc2.row_step;
	msg.is_dense = pcl_pc2.is_dense;
	msg.data = pcl_pc2.data;
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::auxilaryFunction_PsiMT(
	const real_t a, const real_t f_a, const real_t f_0,
	const real_t g_0, const real_t mu)
{
  return (f_a - f_0 - mu * g_0 * a);
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::auxilaryFunction_dPsiMT(
	const real_t g_a, const real_t g_0, const real_t mu)
{
  return (g_a - mu * g_0);
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::trialValueSelectionMT(
	const real_t a_l, const real_t f_l, const real_t g_l,
	const real_t a_u, const real_t f_u, const real_t g_u,
	const real_t a_t, const real_t f_t, const real_t g_t)
{
	// Case 1 in Trial Value Selection [More, Thuente 1994]
	if (f_t > f_l) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		const real_t z = 3.0F * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		const real_t w = sqrtf(z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		const real_t a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2.0F * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
		// Equation 2.4.2 [Sun, Yuan 2006]
		const real_t a_q = a_l - 0.5F * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

		if (fabsf(a_c - a_l) < fabsf(a_q - a_l)) {
		  return (a_c);
		} else {
		  return (0.5F * (a_q + a_c));
		}
	} else if (g_t * g_l < 0.0F) { 	// Case 2 in Trial Value Selection [More, Thuente 1994]
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		const real_t z = 3.0F * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		const real_t w = sqrtf(z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		const real_t a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2.0F * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		const real_t a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		if (fabsf(a_c - a_t) >= fabsf(a_s - a_t)) {
		  return (a_c);
		} else {
		  return (a_s);
		}
	} else if (fabsf(g_t) <= fabsf(g_l)) {  // Case 3 in Trial Value Selection [More, Thuente 1994]
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		const real_t z = 3.0F * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		const real_t w = sqrtf(z * z - g_t * g_l);
		const real_t a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2.0F * w);

		// Calculate the minimizer of the quadratic that interpolates g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		const real_t a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		real_t a_t_next;
		if (fabsf(a_c - a_t) < fabsf(a_s - a_t)) {
		  a_t_next = a_c;
		} else {
		  a_t_next = a_s;
		}

		if (a_t > a_l) {
		  return (std::min(a_t + 0.66F * (a_u - a_t), a_t_next));
		} else {
		  return (std::max(a_t + 0.66F * (a_u - a_t), a_t_next));
		}
	}
	// Case 4 in Trial Value Selection [More, Thuente 1994]
	else {
		// Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		const real_t z = 3.0F * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
		const real_t w = sqrtf(z * z - g_t * g_u);
		// Equation 2.4.56 [Sun, Yuan 2006]
		return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2.0F * w));
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::updateIntervalMT (
	real_t &a_l, real_t &f_l, real_t &g_l,
	real_t &a_u, real_t &f_u, real_t &g_u,
	const real_t a_t, const real_t f_t, const real_t g_t)
{
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
	if (f_t > f_l) {
		a_u = a_t;
		f_u = f_t;
		g_u = g_t;
		return (false);
	}
	// Case U2 in Update Algorithm and Case b in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) > 0.0F) {
		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Case U3 in Update Algorithm and Case c in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) < 0.0F) {
		a_u = a_l;
		f_u = f_l;
		g_u = g_l;

		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Interval Converged
	else {
		return (true);
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::compute_transformation(
	const typename pcl::PointCloud<PointInputType> & input_points,
  Eigen::Matrix<real_t, 4U, 4U> & pred_transform)
{
	typename pcl::PointCloud<PointInputType> transformed_points;
	transformed_points.points.resize(input_points.points.size());
	pcl::transformPointCloud(input_points, transformed_points, pred_transform);

	Eigen::Transform<real_t, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
	eig_transformation.matrix() = pred_transform;

	Eigen::Matrix<real_t, 6U, 1U> p, delta_p, score_gradient;
	const Vector3 init_translation = eig_transformation.translation();
	const Vector3 init_rotation = eig_transformation.rotation().eulerAngles(0U, 1U, 2U);

	p << init_translation(0U), init_translation(1U), init_translation(2U),
	     init_rotation(0U), init_rotation(1U), init_rotation(2U);

	Eigen::Matrix<real_t, 6U, 6U> hessian;
	real_t score = computeDerivatives(
		score_gradient, hessian, input_points, transformed_points, p, true);

	const real_t points_size = static_cast<real_t>(input_points.points.size());
	std::cout << "before p: " << p << "\n";
	std::cout << "Before score: " << score << "\n";
	m_num_iteration = 0U;
	for (; m_num_iteration < m_maximum_iteration; ++m_num_iteration) {
		Eigen::JacobiSVD<Eigen::Matrix<real_t, 6U, 6>> sv(
			hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);

		delta_p = sv.solve(-score_gradient);

		real_t delta_p_norm = delta_p.norm();

		if (delta_p_norm == 0.0F) {
			m_trans_probability = score / points_size;
			throw std::runtime_error("In NDT matching: The updated transformation equals to zero matrix");
		}

		delta_p.normalize();
		delta_p_norm = computeStepLengthMT(
			pred_transform, p, delta_p, delta_p_norm, m_step_size, m_translation_eps / 2.0F,
			score, score_gradient, hessian, input_points, transformed_points);
		delta_p *= delta_p_norm;

		p = p + delta_p;

		if ((m_num_iteration > m_maximum_iteration) ||
			  (m_num_iteration && (fabsf(delta_p_norm) < m_translation_eps))) {
			break;
		}
	}

	std::cout << "After p: " << p << "\n";
	std::cout << "Iteration: " << m_num_iteration << ", score: " << score << "\n";

	if (points_size > 0.0F) {
		m_trans_probability = score / points_size;
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::computeDerivatives(
	Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
	Eigen::Matrix<real_t, 6U, 6> & hessian,
	const typename pcl::PointCloud<PointInputType> & input_points,
	const typename pcl::PointCloud<PointInputType> & transformed_points,
	const Eigen::Matrix<real_t, 6U, 1U> & pose,
	const bool8_t compute_hessian)
{
	score_gradient.setZero();
	hessian.setZero();

	//Compute Angle Derivatives
	computeAngleDerivatives(pose);

	std::vector<int32_t> neighbor_ids;
	Eigen::Matrix<real_t, 3U, 6U> point_gradient;
	Eigen::Matrix<real_t, 18U, 6U> point_hessian;
	real_t score = 0.0F;

	point_gradient.setZero();
	point_gradient.block<3U, 3U>(0U, 0U).setIdentity();
	point_hessian.setZero();

	const int32_t points_size = transformed_points.points.size();
	for (int32_t idx = 0U; idx < points_size; ++idx) {
		neighbor_ids.clear();
		const PointInputType & x_trans_pt = transformed_points.points[idx];

		m_voxel_grid.radius_search(x_trans_pt, m_resolution, neighbor_ids);
		const int32_t near_size = neighbor_ids.size();
		for (int32_t i = 0U; i < near_size; ++i) {
			const int32_t vid = neighbor_ids[i];
			const PointInputType & x_pt = input_points.points[idx];
			const Vector3 x = Vector3(x_pt.x, x_pt.y, x_pt.z);
			computePointDerivatives(x, point_gradient, point_hessian, compute_hessian);

			Vector3 x_trans = Vector3(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);
			x_trans -= m_voxel_grid.getCentroid(vid);
			const Matrix3 & c_inv = m_voxel_grid.getInverseCovariance(vid);
			score += updateDerivatives(score_gradient, hessian, point_gradient,
				                         point_hessian, x_trans, c_inv, compute_hessian);
			// Vector3 x_trans = Vector3(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);
			// Eigen::Vector3d hoge1 = m_voxel_grid.getCentroid(vid);
			// x_trans -= hoge1.cast<real_t>();
			// Eigen::Matrix3d hoge2 = m_voxel_grid.getInverseCovariance(vid);
			// score += updateDerivatives(score_gradient, hessian, point_gradient,
			// 	                         point_hessian, x_trans, hoge2.cast<real_t>(), compute_hessian);
		}
	}

	return score;
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::computePointDerivatives(
	const Vector3 & x,
	Eigen::Matrix<real_t, 3U, 6U> & point_gradient,
	Eigen::Matrix<real_t, 18U, 6U> & point_hessian,
	const bool8_t compute_hessian)
{
	point_gradient(1U, 3U) = x.dot(m_j_ang_a);
	point_gradient(2U, 3U) = x.dot(m_j_ang_b);
	point_gradient(0U, 4U) = x.dot(m_j_ang_c);
	point_gradient(1U, 4U) = x.dot(m_j_ang_d);
	point_gradient(2U, 4U) = x.dot(m_j_ang_e);
	point_gradient(0U, 5U) = x.dot(m_j_ang_f);
	point_gradient(1U, 5U) = x.dot(m_j_ang_g);
	point_gradient(2U, 5U) = x.dot(m_j_ang_h);

	if (compute_hessian) {
		Vector3 a, b, c, d, e, f;
    a << 0.0F, x.dot(m_h_ang_a2), x.dot(m_h_ang_a3);
    b << 0.0F, x.dot(m_h_ang_b2), x.dot(m_h_ang_b3);
    c << 0.0F, x.dot(m_h_ang_c2), x.dot(m_h_ang_c3);
    d << x.dot(m_h_ang_d1), x.dot(m_h_ang_d2), x.dot(m_h_ang_d3);
    e << x.dot(m_h_ang_e1), x.dot(m_h_ang_e2), x.dot(m_h_ang_e3);
    f << x.dot(m_h_ang_f1), x.dot(m_h_ang_f2), x.dot(m_h_ang_f3);

		point_hessian.block<3U, 1U>(9U, 3U) = a;
		point_hessian.block<3U, 1U>(12U, 3U) = b;
		point_hessian.block<3U, 1U>(15U, 3U) = c;
		point_hessian.block<3U, 1U>(9U, 4U) = b;
		point_hessian.block<3U, 1U>(12U, 4U) = d;
		point_hessian.block<3U, 1U>(15U, 4U) = e;
		point_hessian.block<3U, 1U>(9U, 5U) = c;
		point_hessian.block<3U, 1U>(12U, 5U) = e;
		point_hessian.block<3U, 1U>(15U, 5U) = f;
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::updateDerivatives(
	Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
	Eigen::Matrix<real_t, 6U, 6U> & hessian,
	const Eigen::Matrix<real_t, 3U, 6U> & point_gradient,
	const Eigen::Matrix<real_t, 18U, 6U> & point_hessian,
	const Vector3 & x_trans,
	const Matrix3 & c_inv,
	const bool8_t compute_hessian)
{
	real_t e_x_cov_x = expf(-m_gauss_d2 * x_trans.dot(c_inv * x_trans) / 2.0F);
	// Compute the current score
	const real_t score_inc = -m_gauss_d1 * e_x_cov_x;

	// Compute the gradient
	e_x_cov_x = m_gauss_d2 * e_x_cov_x;

	if (e_x_cov_x > 1.0F || e_x_cov_x < 0.0F || e_x_cov_x != e_x_cov_x) {
		return 0.0F;
	}

	e_x_cov_x *= m_gauss_d1;
	for (int32_t i = 0U; i < 6U; ++i) {
		const Vector3 cov_dxd_pi = c_inv * point_gradient.col(i);

		score_gradient(i) += x_trans.dot(cov_dxd_pi) * e_x_cov_x;

		if (compute_hessian) {
			const int32_t col_size = hessian.cols();
			for (int32_t j = 0U; j < col_size; ++j) {
				hessian(i, j) += e_x_cov_x * (-m_gauss_d2 * x_trans.dot(cov_dxd_pi) * x_trans.dot(c_inv * point_gradient.col(j)) +
									x_trans.dot(c_inv * point_hessian.block<3U, 1U>(3U * i, j)) +
									point_gradient.col(j).dot(cov_dxd_pi));
			}
		}
	}
	return score_inc;
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::updateHessian(
	Eigen::Matrix<real_t, 6U, 6U> & hessian,
	const Eigen::Matrix<real_t, 3U, 6U> & point_gradient,
	const Eigen::Matrix<real_t, 18U, 6U> & point_hessian,
	const Vector3 & x_trans,
	const Matrix3 & c_inv)
{
	real_t e_x_cov_x = m_gauss_d2 * expf(-(m_gauss_d2 / 2.0F) * x_trans.dot(c_inv * x_trans));

	if (e_x_cov_x > 1.0F || e_x_cov_x < 0.0F || e_x_cov_x != e_x_cov_x) {
		return;
	}

	e_x_cov_x *= m_gauss_d1;

	for (int32_t i = 0U; i < 6U; ++i) {
		const Vector3 cov_dxd_pi = c_inv * point_gradient.col(i);
		const int32_t col_size = hessian.cols();
		for (int32_t j = 0U; j < col_size; ++j) {
			hessian(i, j) += e_x_cov_x * (-m_gauss_d2 * x_trans.dot(cov_dxd_pi) * x_trans.dot(c_inv * point_gradient.col(j)) +
								x_trans.dot(c_inv * point_hessian.block<3U, 1U>(3U * i, j)) +
								point_gradient.col(j).dot(cov_dxd_pi));
		}
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::computeAngleDerivatives(
	const Eigen::Matrix<real_t, 6U, 1U> pose,
	const bool8_t compute_hessian)
{
	real_t cx, cy, cz, sx, sy, sz;

	if (fabs(pose(3U)) < 10e-5) {
		cx = 1.0F;
		sx = 0.0F;
	} else {
		cx = cosf(pose(3U));
		sx = sinf(pose(3U));
	}

	if (fabs(pose(4U)) < 10e-5) {
		cy = 1.0F;
		sy = 0.0F;
	} else {
		cy = cosf(pose(4U));
		sy = sinf(pose(4U));
	}

	if (fabs(pose(5U)) < 10e-5) {
		cz = 1.0F;
		sz = 0.0F;
	} else {
		cz = cosf(pose(5U));
		sz = sinf(pose(5U));
	}

	m_j_ang_a(0U) = -sx * sz + cx * sy * cz;
	m_j_ang_a(1U) = -sx * cz - cx * sy * sz;
	m_j_ang_a(2U) = -cx * cy;

	m_j_ang_b(0U) = cx * sz + sx * sy * cz;
	m_j_ang_b(1U) = cx * cz - sx * sy * sz;
	m_j_ang_b(2U) = -sx * cy;

	m_j_ang_c(0U) = -sy * cz;
	m_j_ang_c(1U) = sy * sz;
	m_j_ang_c(2U) = cy;

	m_j_ang_d(0U) = sx * cy * cz;
	m_j_ang_d(1U) = -sx * cy * sz;
	m_j_ang_d(2U) = sx * sy;

	m_j_ang_e(0U) = -cx * cy * cz;
	m_j_ang_e(1U) = cx * cy * sz;
	m_j_ang_e(2U) = -cx * sy;

	m_j_ang_f(0U) = -cy * sz;
	m_j_ang_f(1U) = -cy * cz;
	m_j_ang_f(2U) = 0.0F;

	m_j_ang_g(0U) = cx * cz - sx * sy * sz;
	m_j_ang_g(1U) = -cx * sz - sx * sy * cz;
	m_j_ang_g(2U) = 0.0F;

	m_j_ang_h(0U) = sx * cz + cx * sy * sz;
	m_j_ang_h(1U) = cx * sy * cz - sx * sz;
	m_j_ang_h(2U) = 0.0F;

	if (compute_hessian) {
		m_h_ang_a2(0U) = -cx * sz - sx * sy * cz;
		m_h_ang_a2(1U) = -cx * cz + sx * sy * sz;
		m_h_ang_a2(2U) = sx * cy;

		m_h_ang_a3(0U) = -sx * sz + cx * sy * cz;
		m_h_ang_a3(1U) = -cx * sy * sz - sx * cz;
		m_h_ang_a3(2U) = -cx * cy;

		m_h_ang_b2(0U) = cx * cy * cz;
		m_h_ang_b2(1U) = -cx * cy * sz;
		m_h_ang_b2(2U) = cx * sy;

		m_h_ang_b3(0U) = sx * cy * cz;
		m_h_ang_b3(1U) = -sx * cy * sz;
		m_h_ang_b3(2U) = sx * sy;

		m_h_ang_c2(0U) = -sx * cz - cx * sy * sz;
		m_h_ang_c2(1U) = sx * sz - cx * sy * cz;
		m_h_ang_c2(2U) = 0.0F;

		m_h_ang_c3(0U) = cx * cz - sx * sy * sz;
		m_h_ang_c3(1U) = -sx * sy * cz - cx * sz;
		m_h_ang_c3(2U) = 0.0F;

		m_h_ang_d1(0U) = -cy * cz;
		m_h_ang_d1(1U) = cy * sz;
		m_h_ang_d1(2U) = sy;

		m_h_ang_d2(0U) = -sx * sy * cz;
		m_h_ang_d2(1U) = sx * sy * sz;
		m_h_ang_d2(2U) = sx * cy;

		m_h_ang_d3(0U) = cx * sy * cz;
		m_h_ang_d3(1U) = -cx * sy * sz;
		m_h_ang_d3(2U) = -cx * cy;

		m_h_ang_e1(0U) = sy * sz;
		m_h_ang_e1(1U) = sy * cz;
		m_h_ang_e1(2U) = 0.0F;

		m_h_ang_e2(0U) = -sx * cy * sz;
		m_h_ang_e2(1U) = -sx * cy * cz;
		m_h_ang_e2(2U) = 0.0F;

		m_h_ang_e3(0U) = cx * cy * sz;
		m_h_ang_e3(1U) = cx * cy * cz;
		m_h_ang_e3(2U) = 0.0F;

		m_h_ang_f1(0U) = -cy * cz;
		m_h_ang_f1(1U) = cy * sz;
		m_h_ang_f1(2U) = 0.0F;

		m_h_ang_f2(0U) = -cx * sz - sx * sy * cz;
		m_h_ang_f2(1U) = -cx * cz + sx * sy * sz;
		m_h_ang_f2(2U) = 0.0F;

		m_h_ang_f3(0U) = -sx * sz + cx * sy * cz;
		m_h_ang_f3(1U) = -cx * sy * sz - sx * cz;
		m_h_ang_f3(2U) = 0.0F;
	}
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
real_t NdtMatching<PointInputType, PointMapType>::computeStepLengthMT(
	Eigen::Matrix<real_t, 4U, 4U> & pred_transform,
	const Eigen::Matrix<real_t, 6U, 1U> & x,
	Eigen::Matrix<real_t, 6U, 1U> & step_dir,
	const real_t step_init,
	const real_t step_max,
	const real_t step_min,
	real_t & score,
	Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
	Eigen::Matrix<real_t, 6U, 6> & hessian,
	const typename pcl::PointCloud<PointInputType> & input_points,
	typename pcl::PointCloud<PointInputType> & transformed_points)
{
	const real_t phi_0 = -score;
	real_t d_phi_0 = -(score_gradient.dot(step_dir));

	if (d_phi_0 >= 0) {
		if (d_phi_0 == 0) {
			return 0;
		} else {
			d_phi_0 *= -1;
			step_dir *= -1;
		}
	}

	const real_t mu = 1.e-4;
	const real_t nu = 0.9;
	real_t a_l = 0, a_u = 0;

	real_t f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
	real_t g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	real_t f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
	real_t g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	bool interval_converged = (step_max - step_min) < 0;
	bool open_interval = true;

	real_t a_t = step_init;
	a_t = std::min(a_t, step_max);
	a_t = std::max(a_t, step_min);

	Eigen::Matrix<real_t, 6, 1> x_t = x + step_dir * a_t;

	pred_transform = (Eigen::Translation<real_t, 3>(static_cast<real_t>(x_t(0)), static_cast<real_t>(x_t(1)), static_cast<real_t>(x_t(2))) *
								Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(3)), Vector3::UnitX()) *
								Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(4)), Vector3::UnitY()) *
								Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(5)), Vector3::UnitZ())).matrix();

	transformPointCloud(input_points, transformed_points, pred_transform);

	score = computeDerivatives(score_gradient, hessian, input_points, transformed_points, x_t, true);
	real_t phi_t = -score;
	real_t d_phi_t = -(score_gradient.dot(step_dir));
	real_t psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
	real_t d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

	constexpr int max_step_iterations = 10;
	int step_idx = 0;
	for (; step_idx < max_step_iterations; ++step_idx) {
	  if (interval_converged || (psi_t <= 0 && d_phi_t <= -nu * d_phi_0)) {
			break;
		}

		if (open_interval) {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}

		a_t = (a_t < step_max) ? a_t : step_max;
		a_t = (a_t > step_min) ? a_t : step_min;
		x_t = x + step_dir * a_t;

		pred_transform =
		  (Eigen::Translation<real_t, 3>(static_cast<real_t>(x_t(0)), static_cast<real_t>(x_t(1)), static_cast<real_t>(x_t(2))) *
			 Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(3)), Vector3::UnitX()) *
			 Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(4)), Vector3::UnitY()) *
			 Eigen::AngleAxis<real_t>(static_cast<real_t>(x_t(5)), Vector3::UnitZ())).matrix();

    transformPointCloud(input_points, transformed_points, pred_transform);

		score = computeDerivatives(score_gradient, hessian, input_points, transformed_points, x_t, false);

		phi_t -= score;
		d_phi_t -= (score_gradient.dot(step_dir));
		psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
		d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

		if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
			open_interval = false;

			f_l += phi_0 - mu * d_phi_0 * a_l;
			g_l += mu * d_phi_0;

			f_u += phi_0 - mu * d_phi_0 * a_u;
			g_u += mu * d_phi_0;
		}

		if (open_interval) {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}
		++step_idx;
	}

	if (step_idx) {
		computeHessian(hessian, input_points, transformed_points);
	}

	return a_t;
}
////////////////////////////////////////////////////////////////////////////////
template <typename PointInputType, typename PointMapType>
void NdtMatching<PointInputType, PointMapType>::computeHessian(
	Eigen::Matrix<real_t, 6U, 6U> & hessian,
	const typename pcl::PointCloud<PointInputType> & input_points,
	const typename pcl::PointCloud<PointInputType> & transformed_points)
{
	hessian.setZero();
	Eigen::Matrix<real_t, 3U, 6U> point_gradient;
	Eigen::Matrix<real_t, 18U, 6U> point_hessian;
	const int32_t points_size = transformed_points.points.size();
	for (int32_t idx = 0U; idx < points_size; ++idx) {
		const PointInputType & x_trans_pt = transformed_points.points[idx];

		std::vector<int32_t> neighbor_ids;

		m_voxel_grid.radius_search(x_trans_pt, m_resolution, neighbor_ids);
		const int32_t near_size = neighbor_ids.size();
		for (int32_t i = 0U; i < near_size; ++i) {
			const int32_t vid = neighbor_ids[i];
			const PointInputType & x_pt = input_points.points[idx];
			const Vector3 x = Vector3(x_pt.x, x_pt.y, x_pt.z);
			computePointDerivatives(x, point_gradient, point_hessian, true);

			Vector3 x_trans = Vector3(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);
			x_trans -= m_voxel_grid.getCentroid(vid);
			const Matrix3 & c_inv = m_voxel_grid.getInverseCovariance(vid);
			updateHessian(hessian, point_gradient, point_hessian, x_trans, c_inv);
			// Vector3 x_trans = Vector3(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);
			// Eigen::Vector3d hoge1 = m_voxel_grid.getCentroid(vid);
			// x_trans -= hoge1.cast<real_t>();
			// Eigen::Matrix3d hoge2 = m_voxel_grid.getInverseCovariance(vid);
			// updateHessian(hessian, point_gradient, point_hessian, x_trans, hoge2.cast<real_t>());
		}
	}
}

template class NdtMatching<pcl::PointXYZI, pcl::PointXYZI>;
template class NdtMatching<pcl::PointXYZ, pcl::PointXYZ>;
}
}
