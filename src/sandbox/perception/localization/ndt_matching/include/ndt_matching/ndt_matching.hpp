#ifndef NDT_MATCHING__ndt_matching_HPP_
#define NDT_MATCHING__ndt_matching_HPP_

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Transform.h>
#include "ndt_matching/visibility_control.hpp"
#include "ndt_matching/ndt_base.hpp"
#include "ndt_matching/voxel_grid.hpp"
#include "ndt_matching/config.hpp"

namespace autoware_bridge
{
namespace ndt_matching
{

template <typename PointInputType, typename PointMapType>
class NDT_MATCHING_PUBLIC NdtMatching
: public NdtBase<PointInputType, PointMapType>
{
public:
	NdtMatching(const Config & cfg);

	// Set the input map points
	void set_map(const typename pcl::PointCloud<PointMapType>::Ptr input);

	// Set the initial pose
	void set_initial_pose(
		const real_t x, const real_t y, const real_t z,
	  const real_t yaw, const real_t pitch, const real_t roll);

	// Set the odometry from the state estimator
	void set_odometry(const nav_msgs::msg::Odometry & msg);

	// Main function. Take points and predict the position in a map
  const geometry_msgs::msg::PoseStamped update(
    const sensor_msgs::msg::PointCloud2 & inputs);

protected:
	class Pose
	{
	public:
		Pose()
		: m_x(0.0F),
			m_y(0.0F),
			m_z(0.0F),
			m_yaw(0.0F),
			m_pitch(0.0F),
			m_roll(0.0F)
		{};

		Pose(const real_t x, const real_t y, const real_t z,
	     	 const real_t yaw, const real_t pitch, const real_t roll)
		: m_x(x),
		  m_y(y),
			m_z(z),
			m_yaw(yaw),
			m_pitch(pitch),
			m_roll(roll)
		{};

		void set_values(
			const real_t x, const real_t y, const real_t z,
	    const real_t yaw, const real_t pitch, const real_t roll)
		{
			m_x = x;
			m_y = y;
			m_z = z;
			m_yaw = yaw;
			m_pitch = pitch;
			m_roll = roll;
		};

		void set_values_from_matrix(const Eigen::Matrix4f & matrix)
		{
			tf2::Matrix3x3 rotate_mat;
			rotate_mat.setValue(
				static_cast<real_t>(matrix(0, 0)), static_cast<real_t>(matrix(0, 1)), static_cast<real_t>(matrix(0, 2)),
				static_cast<real_t>(matrix(1, 0)), static_cast<real_t>(matrix(1, 1)), static_cast<real_t>(matrix(1, 2)),
				static_cast<real_t>(matrix(2, 0)), static_cast<real_t>(matrix(2, 1)), static_cast<real_t>(matrix(2, 2)));
			m_x = matrix(0U, 3U);
			m_y = matrix(1U, 3U);
			m_z = matrix(2U, 3U);
			double roll;
			double pitch;
			double yaw;
			rotate_mat.getRPY(roll, pitch, yaw);
			roll = static_cast<real_t>(roll);
			pitch = static_cast<real_t>(pitch);
			yaw = static_cast<real_t>(yaw);			
		}

		void create_matrix_from_values(Eigen::Transform<real_t, 3, 1> & transform)
		{
			Eigen::Translation3f init_translation(m_x, m_y, m_z);
			Eigen::AngleAxisf init_rotation_x(m_roll, Vector3::UnitX());
			Eigen::AngleAxisf init_rotation_y(m_pitch, Vector3::UnitY());
			Eigen::AngleAxisf init_rotation_z(m_yaw, Vector3::UnitZ());
			transform = init_translation * init_rotation_z * init_rotation_y * init_rotation_x;
		}

		void create_pose_stamped(geometry_msgs::msg::PoseStamped & pose_stamped)
		{
				tf2::Quaternion current_pose_q;
				current_pose_q.setRPY(m_roll, m_pitch, m_yaw);
				pose_stamped.header.frame_id = "map";
		    pose_stamped.pose.position.x = m_x;
		    pose_stamped.pose.position.y = m_y;
		    pose_stamped.pose.position.z = m_z;
		    pose_stamped.pose.orientation.x = current_pose_q.x();
		    pose_stamped.pose.orientation.y = current_pose_q.y();
		    pose_stamped.pose.orientation.z = current_pose_q.z();
		    pose_stamped.pose.orientation.w = current_pose_q.w();
		}

		real_t x() {return m_x;};
		real_t y() {return m_y;};
		real_t z() {return m_z;};
		real_t yaw() {return m_yaw;};
		real_t pitch() {return m_pitch;};
		real_t roll() {return m_roll;};

	private:
		real_t m_x;
		real_t m_y;
		real_t m_z;
		real_t m_yaw;
		real_t m_pitch;
		real_t m_roll;
	};

	void compute_transformation(
		const typename pcl::PointCloud<PointInputType> & input_points,
		Eigen::Matrix<real_t, 4U, 4U> &guess) override;

private:
  // using ControllerDiagnostic = apex_auto_msgs::msg::ControllerDiagnostic;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  // using Transform = geometry_msgs::msg::Transform;
  // using PointsMessage = sensor_msgs::msg::PointCloud2;

	static void convert_ros2_to_pcl(
		const sensor_msgs::msg::PointCloud2 & msg,
		pcl::PointCloud<PointInputType> & pcl_points);

	static void convert_pcl_to_ros2(
		const pcl::PointCloud<PointInputType> & pcl_points,
		sensor_msgs::msg::PointCloud2 & msg);
	// Copied from ndt.h
  static real_t auxilaryFunction_PsiMT(
		const real_t a, const real_t f_a, const real_t f_0,
		const real_t g_0, const real_t mu=1.e-4);
  // Copied from ndt.h
  static real_t auxilaryFunction_dPsiMT(
		const real_t g_a, const real_t g_0, const real_t mu=1.e-4);

  static real_t updateIntervalMT (
		real_t &a_l, real_t &f_l, real_t &g_l,
		real_t &a_u, real_t &f_u, real_t &g_u,
		const real_t a_t, const real_t f_t, const real_t g_t);

  static real_t trialValueSelectionMT(
		const real_t a_l, const real_t f_l, const real_t g_l,
		const real_t a_u, const real_t f_u, const real_t g_u,
		const real_t a_t, const real_t f_t, const real_t g_t);

	void computeAngleDerivatives(
		const Eigen::Matrix<real_t, 6U, 1U> pose,
		const bool8_t compute_hessian=true);

	real_t computeStepLengthMT(
		Eigen::Matrix<real_t, 4U, 4U> & pred_transform,
		const Eigen::Matrix<real_t, 6U, 1U> & x,
		Eigen::Matrix<real_t, 6U, 1U> & step_dir,
		const real_t step_init, const real_t step_max, const real_t step_min,
		real_t & score,
		Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
		Eigen::Matrix<real_t, 6U, 6U> & hessian,
		const typename pcl::PointCloud<PointInputType> & input_points,
		typename pcl::PointCloud<PointInputType> & transformed_points);

	void computeHessian(
		Eigen::Matrix<real_t, 6U, 6U> & hessian,
		const typename pcl::PointCloud<PointInputType> & input_points,
		const typename pcl::PointCloud<PointInputType> & transformed_points);

	real_t computeDerivatives(
		Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
		Eigen::Matrix<real_t, 6U, 6U> & hessian,
		const typename pcl::PointCloud<PointInputType> & input_points,
		const typename pcl::PointCloud<PointInputType> & transformed_points,
		const Eigen::Matrix<real_t, 6U, 1U> & pose,
		const bool8_t compute_hessian=true);

	void computePointDerivatives(
		const Vector3 & x,
		Eigen::Matrix<real_t, 3U, 6U> & point_gradient,
		Eigen::Matrix<real_t, 18U, 6U> & point_hessian,
		const bool8_t computeHessian=true);

	real_t updateDerivatives(
		Eigen::Matrix<real_t, 6U, 1U> & score_gradient,
		Eigen::Matrix<real_t, 6U, 6U> & hessian,
		const Eigen::Matrix<real_t, 3, 6> & point_gradient,
		const Eigen::Matrix<real_t, 18, 6> & point_hessian,
		const Vector3 & x_trans,
		const Matrix3 & c_inv,
		const bool8_t compute_hessian=true);

	void updateHessian(
		Eigen::Matrix<real_t, 6U, 6U> & hessian,
		const Eigen::Matrix<real_t, 3U, 6U> & point_gradient,
		const Eigen::Matrix<real_t, 18U, 6U> & point_hessian,
		const Vector3 & x_trans,
		const Matrix3 & c_inv);

	using NdtBase<PointInputType, PointMapType>::m_translation_eps;
	using NdtBase<PointInputType, PointMapType>::m_maximum_iteration;
	using NdtBase<PointInputType, PointMapType>::m_resolution;
	using NdtBase<PointInputType, PointMapType>::m_step_size;
	using NdtBase<PointInputType, PointMapType>::m_num_iteration;

	real_t m_gauss_d1;
	real_t m_gauss_d2;
	real_t m_outlier_ratio;
	Vector3 m_j_ang_a, m_j_ang_b, m_j_ang_c, m_j_ang_d, m_j_ang_e, m_j_ang_f, m_j_ang_g, m_j_ang_h;

	Vector3 m_h_ang_a2, m_h_ang_a3, m_h_ang_b2, m_h_ang_b3, m_h_ang_c2, m_h_ang_c3,
		m_h_ang_d1, m_h_ang_d2, m_h_ang_d3, m_h_ang_e1, m_h_ang_e2, m_h_ang_e3,
		m_h_ang_f1, m_h_ang_f2, m_h_ang_f3;

	real_t m_trans_probability;
	bool8_t m_initialized;
	VoxelGrid<PointMapType> m_voxel_grid;
	Pose m_current_pose;
	Pose m_prev_pose;
};  // class NdtMatching
}  // namespace ndt_matching
}  // namespace autoware_bridge
#endif  // NDT_MATCHING__ndt_matching_HPP_
