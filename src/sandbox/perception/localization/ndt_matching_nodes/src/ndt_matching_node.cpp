// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

//lint -e537 cpplint vs pclint NOLINT
#include <chrono>
#include <utility>
#include <string>
#include <vector>
#include "ndt_matching_nodes/ndt_matching_node.hpp"

namespace autoware_bridge
{
namespace ndt_matching_nodes
{

using sub_set_t = std::array<std::shared_ptr<rclcpp::PollingSubscriptionBase>, 1>;  // NOLINT
////////////////////////////////////////////////////////////////////////////////
NdtMatchingNode::NdtMatchingNode(
  const apex::string_strict256_t & node_name,
  const std::chrono::nanoseconds max_cycle_time,
  const apex::string_strict256_t & node_namespace)
: ApexNode{node_name, max_cycle_time, node_namespace},
  m_lidar_localizer(ndt_matching::Config{
          static_cast<float32_t>(get_parameter(
            "localizer.resolution").as_double()),
          static_cast<float32_t>(get_parameter(
            "localizer.step_size").as_double()),
          static_cast<float32_t>(get_parameter("localizer.translation_eps").as_double()),
          static_cast<uint32_t>(get_parameter("localizer.maximum_iteration").as_int()),
          get_parameter("map_file").as_string()}),
  m_qos{
    rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    static_cast<uint32_t>(get_parameter("qos_depth").as_int()),
    rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    false},
  m_tf(create_polling_subscription<tf2_msgs::msg::TFMessage>("tf"),
    get_parameter("static_frame_ids").as_string_array(),
    get_parameter("dynamic_frame_ids").as_string_array()),
  m_tf_policy(std::chrono::milliseconds(get_parameter("tf_timeout").as_int())),
  m_points_sub_ptr(create_polling_subscription<PointsMessage>(
      get_parameter("points_topic").as_string(), m_qos)),
  m_odom_policy_sub(
    create_polling_subscription<PoseWithCovarianceStamped>(
      get_parameter("odom_topic").as_string(), m_qos),
    static_cast<uint32_t>(get_parameter("num_max_odom_history").as_int()),
    apex::time_sample_policies::OldestNewerThanTimestamp(
      std::chrono::milliseconds(get_parameter("odom_minimum_rate").as_int()))),
  m_pose_pub_ptr(ApexNode::create_publisher<PoseStamped>(
      get_parameter("pose_topic").as_string())),
  m_diag_pub_ptr(ApexNode::create_publisher<ControllerDiagnostic>(
      get_parameter("diagnosis_topic").as_string())),
  m_points_pub_ptr(ApexNode::create_publisher<PointsMessage>(
      "filfil")),
  m_tf2_pub_ptr(ApexNode::create_publisher<tf2_msgs::msg::TFMessage>("tf")),
  m_initial_pose_initialized(false),
  m_timeout(std::chrono::milliseconds(get_parameter("timeout_ms").as_int())),
  m_waitset(sub_set_t {{m_points_sub_ptr}})
{
  const bool8_t use_initial_pose = get_parameter("localizer.use_initial_pose").as_bool();
  if (use_initial_pose) {
    const float32_t x = static_cast<float32_t>(get_parameter("localizer.initial_pose.x").as_double());
    const float32_t y = static_cast<float32_t>(get_parameter("localizer.initial_pose.y").as_double());
    const float32_t z = static_cast<float32_t>(get_parameter("localizer.initial_pose.z").as_double());
    const float32_t yaw = static_cast<float32_t>(get_parameter("localizer.initial_pose.yaw").as_double());
    const float32_t pitch = static_cast<float32_t>(get_parameter("localizer.initial_pose.pitch").as_double());
    const float32_t roll = static_cast<float32_t>(get_parameter("localizer.initial_pose.roll").as_double());
    m_lidar_localizer.set_initial_pose(x, y, z, yaw, pitch, roll);
    m_initial_pose_initialized = true;
  }
}
////////////////////////////////////////////////////////////////////////////////
// NdtMatchingNode::NdtMatchingNode(
//   const apex::string_strict256_t & node_name,
//   const apex::string_strict256_t & pose_topic,
//   const apex::string_strict256_t & odom_topic,
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
//   const apex::string_strict256_t & node_namespace)
// : ApexNode(node_name, max_cycle_time, node_namespace),
//   m_lidar_localizer(cfg),
//   m_qos{
//     rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL,
//     qos_depth,
//     rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
//     RMW_QOS_POLICY_DURABILITY_VOLATILE,
//     false},
//   m_tf(create_polling_subscription<tf2_msgs::msg::TFMessage>("tf"),
//     static_frame_ids, dynamic_frame_ids),
//   m_tf_policy(tf_timeout),
//   m_points_sub_ptr(create_polling_subscription<TrajectoryPointStamped>(
//       pose_topic.c_str(), m_qos)),
//   m_odom_policy_sub(
//     create_polling_subscription<Trajectory>(odom_topic.c_str(), m_qos),
//     num_max_traj_history,
//     apex::time_sample_policies::OldestNewerThanTimestamp(traj_minimum_rate)),
//   m_cmd_pub_ptr(ApexNode::create_publisher<VehicleControlCommand>(
//       command_topic.c_str())),
//   m_diag_pub_ptr(ApexNode::create_publisher<ControllerDiagnostic>(
//       diagnosis_topic.c_str())),
//   m_initial_pose_initialized(false),
//   m_timeout(timeout),
//   m_waitset(sub_set_t {{m_points_sub_ptr}})
// {
// }
////////////////////////////////////////////////////////////////////////////////
void NdtMatchingNode::function()
{
  APEX_PRINT("Success running");
  try {
    (void)m_waitset.wait(m_timeout);
    const auto loan_points = m_points_sub_ptr->take();
    if (loan_points) {
      // Take a odom
      try {
        // const auto timestamp =
        //   apex::time_sample_policies::HeaderTimestampTrait<decltype(loan_points)>::timestamp(loan_points);
        // const auto odom_data = m_odom_policy_sub.read(timestamp);
        // const PoseWithCovarianceStamped & new_odom_data = (*odom_data.first).data();
        // m_tf.update(apex::from_msg_time(new_odom_data.header.stamp));
        // std::cout << new_odom_data.pose.pose.orientation.x << ", " << new_odom_data.pose.pose.orientation.y << ", " <<
				//   new_odom_data.pose.pose.orientation.z << "\n";
        // exit(0);
        // const bool8_t can_transform = m_tf.can_transform(
        //   "/map",
        //   "base_link",
        //   apex::from_msg_time(new_odom_data.header.stamp));
        // if (can_transform) {
        //   const geometry_msgs::msg::Transform & m_to_b_transform =
        //     m_tf.lookup_transform(
        //       "/map",
        //       "base_link",
        //       apex::from_msg_time(new_odom_data.header.stamp),
        //       &m_tf_policy);
        //   m_lidar_localizer.set_odometry(new_odom_data, m_to_b_transform);
        // }
      } catch (const apex::time_sample_policies::NoSampleAvailable & e) {
        APEX_PRINT("No odom topic: only processed by the lidar points");
      }
      m_initial_pose_initialized = true;
      // Take a points
      auto points = loan_points.data();
      // m_tf.update(apex::from_msg_time((m_lidar_localizer.get_odom().header.stamp)));
      // const bool8_t can_transform = m_tf.can_transform(
      //   points.header.frame_id.c_str(),
      //   m_lidar_localizer.get_odom().header.frame_id.c_str(),
      //   apex::from_msg_time(m_lidar_localizer.get_odom().header.stamp));
      // if (can_transform) {
      //   const geometry_msgs::msg::Transform points_to_traj = m_tf.lookup_transform(
      //     points.header.frame_id.c_str(),
      //     m_lidar_localizer.get_odom().header.frame_id.c_str(),
      //     apex::from_msg_time(m_lidar_localizer.get_odom().header.stamp),
      //     &m_tf_policy);
      // transform_points(points, points_to_traj);
      const PoseStamped pose = m_lidar_localizer.update(points);
      m_pose_pub_ptr->publish(pose);

      tf2_msgs::msg::TFMessage tf2_msg;
      tf2_msg.transforms.assign(1, geometry_msgs::msg::TransformStamped{});
      tf2_msg.transforms[0].transform.translation.x = pose.pose.position.x;
      tf2_msg.transforms[0].transform.translation.y = pose.pose.position.y;
      tf2_msg.transforms[0].transform.translation.z = pose.pose.position.z;
      tf2_msg.transforms[0].transform.rotation.x = pose.pose.orientation.x;
      tf2_msg.transforms[0].transform.rotation.y = pose.pose.orientation.y;
      tf2_msg.transforms[0].transform.rotation.z = pose.pose.orientation.z;
      tf2_msg.transforms[0].transform.rotation.w = pose.pose.orientation.w;
      tf2_msg.transforms[0].header.frame_id = "/map";
      tf2_msg.transforms[0].header.stamp = points.header.stamp;
      tf2_msg.transforms[0].child_frame_id = "/base_link";
      m_tf2_pub_ptr->publish(tf2_msg);
      // m_lidar_localizer.update(points);
      // m_points_pub_ptr->publish(points);
      // m_cmd_pub_ptr->publish(cmd);
      // const auto & diagnostic = m_lidar_localizer.get_diagnostic();
      // m_diag_pub_ptr->publish(diagnostic);
      // } else {
      //   // Since the odom is taken successfully, TF graph or the odom's timestamp
      //   // has problems. Fatal error.
      //   throw std::runtime_error("NdtMatchingNode: could not subscribe the tf topic");
      // }
    }
  } catch (const rclcpp::TimeoutError & e) {
    APEX_PRINT("Waitset rclcpp::TimeoutError: Waiting for the points topic");
  } catch (const apex::time_sample_policies::NoSampleAvailable & e) {
    if (m_initial_pose_initialized) {
      // throw std::runtime_error("Could not take the odom within the specified time duration");
    } else {
      APEX_PRINT(
        "Could not take the odom since this node has not subscribed odom topics");
    }
  } catch (const std::runtime_error & e) {
    APEX_PRINT(e.what());
  } catch (...) {
    APEX_PRINT("Error");
  }
}
////////////////////////////////////////////////////////////////////////////////
void NdtMatchingNode::transform_pose(
  TrajectoryPointStamped & pose, const Transform & transform) const
{
  // Compute transform TODO(y.tsuji) Ported from the KinematicTracker.
  // Current ApexOS does not support tf based transformation.
  const float32_t a2 = static_cast<float32_t>(transform.rotation.w * transform.rotation.w);
  const float32_t b2 = static_cast<float32_t>(transform.rotation.x * transform.rotation.x);
  const float32_t c2 = static_cast<float32_t>(transform.rotation.y * transform.rotation.y);
  const float32_t d2 = static_cast<float32_t>(transform.rotation.z * transform.rotation.z);
  const float32_t ad = static_cast<float32_t>(transform.rotation.w * transform.rotation.z);
  const float32_t bc = static_cast<float32_t>(transform.rotation.x * transform.rotation.y);
  const float32_t rot_xx = (a2 + b2) - (c2 + d2);
  const float32_t rot_xy = 2.0F * (bc - ad);
  const float32_t rot_x = static_cast<float32_t>(transform.translation.x);
  const float32_t rot_yx = 2.0F * (bc + ad);
  const float32_t rot_yy = (a2 + c2) - (b2 + d2);
  const float32_t rot_y = static_cast<float32_t>(transform.translation.y);
  const float32_t pos_x = pose.state.x;
  pose.state.x = (rot_xx * pos_x) + (rot_xy * pose.state.y) + rot_x;
  pose.state.y = (rot_yx * pos_x) + (rot_yy * pose.state.y) + rot_y;
  const float32_t siny = 2.0F * (ad + bc);
  const float32_t cosy = 1.0F - (2.0F * (c2 + d2));
  pose.state.heading = apex_auto::common::helper_functions::angle_distance_rad(
    pose.state.heading + atan2f(siny, cosy), 0.0F);
}
}  // namespace ndt_matching_nodes
}  // namespace autoware_bridge
