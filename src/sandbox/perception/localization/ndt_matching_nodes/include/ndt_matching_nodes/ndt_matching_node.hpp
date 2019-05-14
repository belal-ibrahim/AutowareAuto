/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the pure pursuit node

#ifndef NDT_MATCHING_NODES__ndt_matching_NODE_HPP_
#define NDT_MATCHING_NODES__ndt_matching_NODE_HPP_

#include <apexcpp/apexcpp.hpp>
#include <apexcpp/apex_node.hpp>
#include <apex_auto_msgs/msg/trajectory.hpp>
#include <apex_auto_msgs/msg/trajectory_point_stamped.hpp>
#include <apex_auto_msgs/msg/controller_diagnostic.hpp>
#include <apex_auto_msgs/msg/vehicle_control_command.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <apex_tf/apex_tf.hpp>
#include <time_policies/time_policy_subscription.hpp>
#include <time_policies/priority_wrapper.hpp>
#include <ndt_matching/ndt_matching.hpp>
#include <string>
#include <vector>
#include "ndt_matching_nodes/visibility_control.hpp"

namespace autoware_bridge
{
namespace ndt_matching_nodes
{

/// \brief Boilerplate node that subscribes to the current pose and
/// publishes a vehicle control command
class NDT_MATCHING_NODES_PUBLIC NdtMatchingNode : public apex::node::ApexNode
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] max_cycle_time Maximum cycle time of this node, e.g. the time it takes to receive
  ///                           handle, and publish a packet should be no more than this
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  NdtMatchingNode(
    const apex::string_strict256_t & node_name,
    const std::chrono::nanoseconds max_cycle_time,
    const apex::string_strict256_t & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] pose_topic Name of input pose topic
  /// \param[in] trajectory_topic Name of input trajectory topic
  /// \param[in] command_topic Name of output control command topic
  /// \param[in] diagnosis_topic Name of output diagnosis topic
  /// \param[in] static_frame_ids Name list of the static frame id
  /// \param[in] dynamic_frame_ids Name list of the dynamic frame id
  /// \param[in] timeout Timeout of waiting for data, will result in an internal error
  /// \param[in] tf_timeout Timeout of waiting for the tf data, will result in a runtime error
  /// \param[in] traj_minimum_rate The policy to select a sample up to the specified time older than
  ///                              the pose topics timestamp.
  /// \param[in] qos_depth Depth value for the qos setting
  /// \param[in] num_max_traj_history Maximum number of the history that
  ///                                 this node can store trajectory topics
  /// \param[in] cfg Configuration object for NdtMatching
  /// \param[in] max_cycle_time Maximum cycle time of this node, e.g. the time it takes to receive
  ///                           handle, and publish a packet should be no more than this
  /// \param[in] node_namespace Namespace of this node
  // NdtMatchingNode(
  //   const apex::string_strict256_t & node_name,
  //   const apex::string_strict256_t & pose_topic,
  //   const apex::string_strict256_t & trajectory_topic,
  //   const apex::string_strict256_t & command_topic,
  //   const apex::string_strict256_t & diagnosis_topic,
  //   const std::vector<std::string> & static_frame_ids,
  //   const std::vector<std::string> & dynamic_frame_ids,
  //   const std::chrono::nanoseconds & timeout,
  //   const std::chrono::nanoseconds & tf_timeout,
  //   const std::chrono::nanoseconds & traj_minimum_rate,
  //   const uint32_t qos_depth,
  //   const uint32_t num_max_traj_history,
  //   const ndt_matching::Config & cfg,
  //   const std::chrono::nanoseconds max_cycle_time,
  //   const apex::string_strict256_t & node_namespace = "");

protected:
  /// \brief Core run loop
  void function() override;

private:
  using TrajectoryPointStamped = apex_auto_msgs::msg::TrajectoryPointStamped;
  using ControllerDiagnostic = apex_auto_msgs::msg::ControllerDiagnostic;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using Transform = geometry_msgs::msg::Transform;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointsMessage = sensor_msgs::msg::PointCloud2;
  /// \brief Transform the current pose by the given transformation
  /// \param[inout] pose The current vehicle pose
  /// \param[in] transform The transformation from the source frame to the target frame
  ndt_matching_NODES_LOCAL
  void transform_pose(
    TrajectoryPointStamped & pose, const Transform & transform) const;

  ndt_matching::NdtMatching<pcl::PointXYZ, pcl::PointXYZ> m_lidar_localizer;
  const rmw_qos_profile_t m_qos;
  apex_auto::common::apex_tf::Tf<> m_tf;
  apex_auto::common::apex_tf::TimeoutPolicy m_tf_policy;

  const rclcpp::PollingSubscription<PointsMessage>::SharedPtr m_points_sub_ptr;
  apex::time_sample_policies::TimePolicySubscription<
    rclcpp::PollingSubscription<PoseWithCovarianceStamped>::SharedPtr,
    apex::time_sample_policies::OldestNewerThanTimestamp> m_odom_policy_sub;
  const typename rclcpp::Publisher<PoseStamped>::SharedPtr m_pose_pub_ptr;
  const typename rclcpp::Publisher<ControllerDiagnostic>::SharedPtr m_diag_pub_ptr;
  const typename rclcpp::Publisher<PointsMessage>::SharedPtr m_points_pub_ptr;
  const typename rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf2_pub_ptr;

  bool8_t m_initial_pose_initialized;
  const std::chrono::nanoseconds m_timeout;
  rclcpp::Waitset<1> m_waitset;
};  // class NdtMatchingNode
}  // namespace ndt_matching_nodes
}  // namespace autoware_bridge

#endif  // NDT_MATCHING_NODES__ndt_matching_NODE_HPP_
