/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <autoware_config_msgs/ConfigNDT.h>

#include <autoware_msgs/NDTStat.h>

struct pose
{
  float32_t x;
  float32_t y;
  float32_t z;
  float32_t roll;
  float32_t pitch;
  float32_t yaw;
};

static pose initial_pose, predict_pose, previous_pose,
    ndt_pose, current_pose, localizer_pose;

static float32_t offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pose
// Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int init_pos_set = 0;

static cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;

// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static float32_t step_size = 0.1;   // Step size
static float32_t trans_eps = 0.01;  // Transformation epsilon

static ros::Publisher ndt_pose_pub;
static geometry_msgs::PoseStamped ndt_pose_msg;

static ros::Publisher localizer_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg;

static geometry_msgs::TwistStamped estimate_twist_msg;

static ros::Duration scan_duration;

static float32_t exe_time = 0.0;
static bool has_converged;
static int iteration = 0;
static float32_t fitness_score = 0.0;
static float32_t trans_probability = 0.0;

static float32_t diff = 0.0;
static float32_t diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

static float32_t current_velocity = 0.0, previous_velocity = 0.0;  // [m/s]
static float32_t current_velocity_x = 0.0, previous_velocity_x = 0.0;
static float32_t current_velocity_y = 0.0, previous_velocity_y = 0.0;
static float32_t current_velocity_z = 0.0, previous_velocity_z = 0.0;
// static float32_t current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;

static float32_t current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
static float32_t current_accel_x = 0.0;
static float32_t current_accel_y = 0.0;
static float32_t current_accel_z = 0.0;
// static float32_t current_accel_yaw = 0.0;

static float32_t angular_velocity = 0.0;

static int use_predict_pose = 0;

static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

static std_msgs::Float32 time_float_ndt_matching;

static float32_t predict_pose_error = 0.0;

static float32_t _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol;

static std::string _localizer = "velodyne";
static std::string _offset = "linear";  // linear, zero, quadratic

static bool _get_height = false;

static std::ofstream ofs;
static std::string filename;

// static tf::TransformListener local_transform_listener;
static tf::StampedTransform local_transform;

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform("/map", input->header.frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // TODO(y.tsuji) Need to check the transformation
  current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
  current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
  current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

  if (_get_height == true && map_loaded == 1)
  {
    float32_t min_distance = DBL_MAX;
    float32_t nearest_z = current_pose.z;
    for (const auto& p : map)
    {
      float32_t distance = hypot(current_pose.x - p.x, current_pose.y - p.y);
      if (distance < min_distance)
      {
        min_distance = distance;
        nearest_z = p.z;
      }
    }
    current_pose.z = nearest_z;
  }

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  current_velocity = 0.0;
  current_velocity_x = 0.0;
  current_velocity_y = 0.0;
  current_velocity_z = 0.0;
  angular_velocity = 0.0;

  current_accel = 0.0;
  current_accel_x = 0.0;
  current_accel_y = 0.0;
  current_accel_z = 0.0;

  offset_x = 0.0;
  offset_y = 0.0;
  offset_z = 0.0;
  offset_yaw = 0.0;

  init_pos_set = 1;
}

static float32_t calcDiffForRadian(const float32_t lhs_rad, const float32_t rhs_rad)
{
  float32_t diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (map_loaded == 1 && init_pos_set == 1)
  {
    matching_start = std::chrono::system_clock::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    ros::Time current_scan_time = input->header.stamp;
    static ros::Time previous_scan_time = current_scan_time;

    pcl::fromROSMsg(*input, filtered_scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size();

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
    Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
        getFitnessScore_end;
    static float32_t align_time, getFitnessScore_time = 0.0;

    ndt.setInputSource(filtered_scan_ptr);

    // Guess the initial gross estimation of the transformation
    float32_t diff_time = (current_scan_time - previous_scan_time).toSec();

    if (_offset == "linear")
    {
      offset_x = current_velocity_x * diff_time;
      offset_y = current_velocity_y * diff_time;
      offset_z = current_velocity_z * diff_time;
      offset_yaw = angular_velocity * diff_time;
    }
    else if (_offset == "quadratic")
    {
      offset_x = (current_velocity_x + current_accel_x * diff_time) * diff_time;
      offset_y = (current_velocity_y + current_accel_y * diff_time) * diff_time;
      offset_z = current_velocity_z * diff_time;
      offset_yaw = angular_velocity * diff_time;
    }

    predict_pose.x = previous_pose.x + offset_x;
    predict_pose.y = previous_pose.y + offset_y;
    predict_pose.z = previous_pose.z + offset_z;
    predict_pose.roll = previous_pose.roll;
    predict_pose.pitch = previous_pose.pitch;
    predict_pose.yaw = previous_pose.yaw + offset_yaw;

    pode predict_pose_for_ndt = predict_pose;

    Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    align_start = std::chrono::system_clock::now();
    anh_ndt.align(init_guess);
    align_end = std::chrono::system_clock::now();

    has_converged = anh_ndt.hasConverged();

    t = anh_ndt.getFinalTransformation();
    iteration = anh_ndt.getFinalNumIteration();

    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score = anh_ndt.getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();

    trans_probability = anh_ndt.getTransformationProbability();

    align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    t2 = t * tf_btol.inverse();

    getFitnessScore_time =
        std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
        1000.0;

    pthread_mutex_unlock(&mutex);

    tf::Matrix3x3 mat_l;  // localizer
    mat_l.setValue(static_cast<float32_t>(t(0, 0)), static_cast<float32_t>(t(0, 1)), static_cast<float32_t>(t(0, 2)),
                   static_cast<float32_t>(t(1, 0)), static_cast<float32_t>(t(1, 1)), static_cast<float32_t>(t(1, 2)),
                   static_cast<float32_t>(t(2, 0)), static_cast<float32_t>(t(2, 1)), static_cast<float32_t>(t(2, 2)));

    // Update localizer_pose
    localizer_pose.x = t(0, 3);
    localizer_pose.y = t(1, 3);
    localizer_pose.z = t(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    tf::Matrix3x3 mat_b;  // base_link
    mat_b.setValue(static_cast<float32_t>(t2(0, 0)), static_cast<float32_t>(t2(0, 1)), static_cast<float32_t>(t2(0, 2)),
                   static_cast<float32_t>(t2(1, 0)), static_cast<float32_t>(t2(1, 1)), static_cast<float32_t>(t2(1, 2)),
                   static_cast<float32_t>(t2(2, 0)), static_cast<float32_t>(t2(2, 1)), static_cast<float32_t>(t2(2, 2)));

    // Update ndt_pose
    ndt_pose.x = t2(0, 3);
    ndt_pose.y = t2(1, 3);
    ndt_pose.z = t2(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    // Calculate the difference between ndt_pose and predict_pose
    predict_pose_error = sqrt((ndt_pose.x - predict_pose_for_ndt.x) * (ndt_pose.x - predict_pose_for_ndt.x) +
                              (ndt_pose.y - predict_pose_for_ndt.y) * (ndt_pose.y - predict_pose_for_ndt.y) +
                              (ndt_pose.z - predict_pose_for_ndt.z) * (ndt_pose.z - predict_pose_for_ndt.z));

    constexpr float32_t PREDICT_POSE_THRESHOLD = 0.5
    if (predict_pose_error <= PREDICT_POSE_THRESHOLD) {
      use_predict_pose = 0;
    } else {
      use_predict_pose = 1;
    }

    if (use_predict_pose == 0) {
      current_pose.x = ndt_pose.x;
      current_pose.y = ndt_pose.y;
      current_pose.z = ndt_pose.z;
      current_pose.roll = ndt_pose.roll;
      current_pose.pitch = ndt_pose.pitch;
      current_pose.yaw = ndt_pose.yaw;
    } else {
      current_pose.x = predict_pose_for_ndt.x;
      current_pose.y = predict_pose_for_ndt.y;
      current_pose.z = predict_pose_for_ndt.z;
      current_pose.roll = predict_pose_for_ndt.roll;
      current_pose.pitch = predict_pose_for_ndt.pitch;
      current_pose.yaw = predict_pose_for_ndt.yaw;
    }

    // Compute the velocity and acceleration
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    // TODO(y.tsuji)
    // const pose trans_current_pose = convertPoseIntoRelativeCoordinate(current_pose, previous_pose);
    bool is_forehand = true
    current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
    current_velocity =  (is_forehand) ? current_velocity : -current_velocity;
    current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
    current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
    current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
    angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

    current_accel = (diff_time > 0) ? ((current_velocity - previous_velocity) / diff_time) : 0;
    current_accel_x = (diff_time > 0) ? ((current_velocity_x - previous_velocity_x) / diff_time) : 0;
    current_accel_y = (diff_time > 0) ? ((current_velocity_y - previous_velocity_y) / diff_time) : 0;
    current_accel_z = (diff_time > 0) ? ((current_velocity_z - previous_velocity_z) / diff_time) : 0;

    estimated_vel_mps.data = current_velocity;
    estimated_vel_kmph.data = current_velocity * 3.6;

    // Set values for publishing pose
    predict_q.setRPY(predict_pose.roll, predict_pose.pitch, predict_pose.yaw);

    predict_pose_msg.header.frame_id = "/map";
    predict_pose_msg.header.stamp = current_scan_time;
    predict_pose_msg.pose.position.x = predict_pose.x;
    predict_pose_msg.pose.position.y = predict_pose.y;
    predict_pose_msg.pose.position.z = predict_pose.z;
    predict_pose_msg.pose.orientation.x = predict_q.x();
    predict_pose_msg.pose.orientation.y = predict_q.y();
    predict_pose_msg.pose.orientation.z = predict_q.z();
    predict_pose_msg.pose.orientation.w = predict_q.w();

    ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);

    ndt_pose_msg.header.frame_id = "/map";
    ndt_pose_msg.header.stamp = current_scan_time;
    ndt_pose_msg.pose.position.x = ndt_pose.x;
    ndt_pose_msg.pose.position.y = ndt_pose.y;
    ndt_pose_msg.pose.position.z = ndt_pose.z;
    ndt_pose_msg.pose.orientation.x = ndt_q.x();
    ndt_pose_msg.pose.orientation.y = ndt_q.y();
    ndt_pose_msg.pose.orientation.z = ndt_q.z();
    ndt_pose_msg.pose.orientation.w = ndt_q.w();

    current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

    localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);

    localizer_pose_msg.header.frame_id = "/map";
    localizer_pose_msg.header.stamp = current_scan_time;
    localizer_pose_msg.pose.position.x = localizer_pose.x;
    localizer_pose_msg.pose.position.y = localizer_pose.y;
    localizer_pose_msg.pose.position.z = localizer_pose.z;
    localizer_pose_msg.pose.orientation.x = localizer_q.x();
    localizer_pose_msg.pose.orientation.y = localizer_q.y();
    localizer_pose_msg.pose.orientation.z = localizer_q.z();
    localizer_pose_msg.pose.orientation.w = localizer_q.w();

    ndt_pose_pub.publish(ndt_pose_msg);
    localizer_pose_pub.publish(localizer_pose_msg);

    // Send TF "/base_link" to "/map"
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    transform.setRotation(current_q);

    // TODO(y.tsuji): TF check
    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));

    matching_end = std::chrono::system_clock::now();
    exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

    // Set values for /estimate_twist
    estimate_twist_msg.header.stamp = current_scan_time;
    estimate_twist_msg.header.frame_id = "/base_link";
    estimate_twist_msg.twist.linear.x = current_velocity;
    estimate_twist_msg.twist.linear.y = 0.0;
    estimate_twist_msg.twist.linear.z = 0.0;
    estimate_twist_msg.twist.angular.x = 0.0;
    estimate_twist_msg.twist.angular.y = 0.0;
    estimate_twist_msg.twist.angular.z = angular_velocity;

    geometry_msgs::Vector3Stamped estimate_vel_msg;
    estimate_vel_msg.header.stamp = current_scan_time;
    estimate_vel_msg.vector.x = current_velocity;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence: " << input->header.seq << std::endl;
    std::cout << "Timestamp: " << input->header.stamp << std::endl;
    std::cout << "Frame ID: " << input->header.frame_id << std::endl;
    //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness Score: " << fitness_score << std::endl;
    std::cout << "Transformation Probability: " << trans_probability << std::endl;
    std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
    std::cout << "Number of Iterations: " << iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
              << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << t << std::endl;
    std::cout << "Align time: " << align_time << std::endl;
    std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Update previous_***
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time = current_scan_time;

    previous_previous_velocity = previous_velocity;
    previous_velocity = current_velocity;
    previous_velocity_x = current_velocity_x;
    previous_velocity_y = current_velocity_y;
    previous_velocity_z = current_velocity_z;
    previous_accel = current_accel;

    previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "float_ndt_matching");
  pthread_mutex_init(&mutex, NULL);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Geting parameters
  int method_type_tmp = 0;
  private_nh.getParam("method_type", method_type_tmp);
  private_nh.getParam("offset", _offset);
  private_nh.getParam("get_height", _get_height);

  if (nh.getParam("localizer", _localizer) == false)
  {
    std::cout << "localizer is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Log file: " << filename << std::endl;
  std::cout << "offset: " << _offset << std::endl;
  std::cout << "get_height: " << _get_height << std::endl;
  std::cout << "localizer: " << _localizer << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  // Updated in initialpose_callback or gnss_callback
  initial_pose.x = 0.0;
  initial_pose.y = 0.0;
  initial_pose.z = 0.0;
  initial_pose.roll = 0.0;
  initial_pose.pitch = 0.0;
  initial_pose.yaw = 0.0;

  // Publishers
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);

  // Subscribers
  ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, param_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 10, initialpose_callback);
  ros::Subscriber points_sub = nh.subscribe("filtered_points", 1, points_callback);

  ros::spin();

  return 0;
}
