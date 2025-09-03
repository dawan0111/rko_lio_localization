/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "node.hpp"
// other
#include <Eigen/Geometry>
#include <pb_utils/profiler.hpp>
#include <pb_utils/timer.hpp>
// ros
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/message_instance.h>

namespace {
using rko_lio::ImuControl;
using rko_lio_ros_utils::LidarFrame;
using rko_lio_ros_utils::Secondsd;
using rko_lio_ros_utils::TimestampVector;
using rko_lio_ros_utils::Vector3dVector;
using namespace std::literals;

ImuControl imu_msg_to_imu_data(const sensor_msgs::Imu& imu_msg) {
  using EigenRowMajor3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

  ImuControl imu_data;
  imu_data.time = rko_lio_ros_utils::ros_time_to_seconds(imu_msg.header.stamp);
  imu_data.angular_velocity = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.angular_velocity);
  imu_data.angular_velocity_covariance = Eigen::Map<const EigenRowMajor3d>(imu_msg.angular_velocity_covariance.data());
  imu_data.acceleration = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.linear_acceleration);
  imu_data.acceleration_covariance = Eigen::Map<const EigenRowMajor3d>(imu_msg.linear_acceleration_covariance.data());
  return imu_data;
}
} // namespace

namespace rko_lio_ros {
Node::Node() : nh("~") {
  nh.param("results_dir", results_dir, "results"s);
  nh.param("run_name", run_name, "lio_run"s);
  nh.param("debug", debug, debug);
  nh.param("odom_frame", odom_frame, ""s);
  nh.param("odom_topic", odom_topic, "/rko_lio/odom"s);
  nh.param("base_frame", base_frame, ""s);
  nh.param("imu_frame", imu_frame, ""s);
  nh.param("imu_topic", imu_topic, ""s);
  nh.param("lidar_frame", lidar_frame, ""s);
  nh.param("lidar_topic", lidar_topic, ""s);
  nh.param("force_absolute_lidar_timestamps", force_absolute_lidar_timestamps, force_absolute_lidar_timestamps);

  // tf
  nh.param("invert_odom_tf", invert_odom_tf, invert_odom_tf);
  tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

  // publishing
  odom_publisher = nh.advertise<nav_msgs::Odometry>("/rko_lio/odometry", 1);
  accel_publisher = nh.advertise<geometry_msgs::AccelStamped>("/rko_lio/accel", 1);
  nh.param("publish_deskewed_cloud", publish_deskewed_cloud, publish_deskewed_cloud);
  if (publish_deskewed_cloud) {
    frame_publisher = nh.advertise<sensor_msgs::PointCloud2>("/rko_lio/frame", 1);
  }
  nh.param("publish_local_map", publish_local_map, publish_local_map);
  if (publish_local_map) {
    nh.param("map_topic", map_topic, "/rko_lio/local_map"s);
    double publish_map_after_double = 1.0;
    nh.param("publish_map_after", publish_map_after_double, publish_map_after_double);
    publish_map_after = Secondsd(publish_map_after_double);
    map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/rko_lio/local_map", 1);
    map_publish_thread = std::thread([this]() { publish_map_loop(); });
  }

  // lio params
  rko_lio::LIO::Config lio_config{};
  nh.param("deskew", lio_config.deskew, lio_config.deskew);
  nh.param("voxel_size", lio_config.voxel_size, lio_config.voxel_size);
  nh.param("max_points_per_voxel", lio_config.max_points_per_voxel, lio_config.max_points_per_voxel);
  nh.param("max_range", lio_config.max_range, lio_config.max_range);
  nh.param("min_range", lio_config.min_range, lio_config.min_range);
  nh.param("max_correspondance_distance", lio_config.max_correspondance_distance,
           lio_config.max_correspondance_distance);
  nh.param("convergence_criterion", lio_config.convergence_criterion, lio_config.convergence_criterion);
  nh.param("max_num_threads", lio_config.max_num_threads, lio_config.max_num_threads);
  nh.param("initialization_phase", lio_config.initialization_phase, lio_config.initialization_phase);
  nh.param("min_beta", lio_config.min_beta, lio_config.min_beta);
  nh.param("max_expected_jerk", lio_config.max_expected_jerk, lio_config.max_expected_jerk);
  nh.param("double_downsample", lio_config.double_downsample, lio_config.double_downsample);
  lio = std::make_unique<rko_lio::LIO>(lio_config);

  // manually, if, define extrinsics
  parse_cli_extrinsics();

  if (debug) {
    body_accel_publisher = nh.advertise<geometry_msgs::AccelStamped>("/rko_lio/body_accel", 1);
  }

  ROS_INFO_STREAM(
      "Subscribed to IMU: " << imu_topic << (!imu_frame.empty() ? " (frame " + imu_frame + ")" : "") << " and LiDAR: "
                            << lidar_topic << (!lidar_frame.empty() ? " (frame " + lidar_frame + ")" : "")
                            << ". Publishing odometry to "
                               "/rko_lio/odometry and acceleration estimates to /rko_lio/accel. Deskewing is "
                            << (lio->config.deskew ? "enabled" : "disabled") << "."
                            << (publish_deskewed_cloud ? " Publishing deskewed_cloud to /rko_lio/frame." : ""));

  // most of the variables have been set in the main thread now
  registration_thread = std::thread([this]() { registration_loop(); });
  ROS_INFO("LIO Node is up!");
}

// quaternion first (scalar last), then xyz
void Node::parse_cli_extrinsics() {
  // i have a far cleaner version of this on the ros2 side. but im not touching this anymore
  auto parse_cli_extrinsic_to_sophus = [this](const std::string& name) {
    std::vector<double> param_extrinsic_name2base;
    nh.getParam("extrinsic_" + name + "2base", param_extrinsic_name2base);
    if (param_extrinsic_name2base.size() == 7) {
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> mapped_vector(param_extrinsic_name2base.data());
      Eigen::Quaterniond q;
      q.x() = mapped_vector[0];
      q.y() = mapped_vector[1];
      q.z() = mapped_vector[2];
      q.w() = mapped_vector[3];
      if (q.norm() < 1e-6) {
        extrinsics_set = false;
        throw std::runtime_error(name + " extrinsic quaternion has zero norm");
      }
      Sophus::SE3d extrinsic(q, mapped_vector.tail<3>());
      ROS_INFO_STREAM("Parsed " << name << " extrinsic as : " << extrinsic.log().transpose());
      extrinsics_set = true;
      return extrinsic;
    } else if (param_extrinsic_name2base.size() > 0) {
      ROS_WARN_STREAM("Provided a wrong "
                      << name << " extrinsic probably. check the value: "
                      << Eigen::Map<Eigen::VectorXd>(param_extrinsic_name2base.data(), param_extrinsic_name2base.size())
                             .transpose());
    }
    // will be true only if both get parsed successfully
    extrinsics_set = false;
    return Sophus::SE3d{};
  };
  extrinsic_lidar2base = parse_cli_extrinsic_to_sophus("lidar");
  extrinsic_imu2base = parse_cli_extrinsic_to_sophus("imu");
}

bool Node::check_and_query_extrinsics() {
  if (extrinsics_set) {
    return true;
  }
  const std::optional<Sophus::SE3d> imu_transform =
      rko_lio_ros_utils::get_transform(tf_buffer, imu_frame, base_frame, 0s);
  if (!imu_transform) {
    return false;
  }
  const std::optional<Sophus::SE3d> lidar_transform =
      rko_lio_ros_utils::get_transform(tf_buffer, lidar_frame, base_frame, 0s);
  if (!lidar_transform) {
    return false;
  }
  extrinsic_imu2base = imu_transform.value();
  extrinsic_lidar2base = lidar_transform.value();
  extrinsics_set = true;
  return true;
}

void Node::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (imu_frame.empty()) {
    imu_frame = imu_msg->header.frame_id;
  }
  if (!check_and_query_extrinsics()) {
    // we assume that extrinsics are static. if they change, its better to querey the tf directly in the registration
    // loop for each message being processed asynchronously.
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    imu_buffer.emplace(imu_msg_to_imu_data(*imu_msg));
    atomic_can_process = !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
  }
  if (atomic_can_process) {
    sync_condition_variable.notify_one();
  }
}

void Node::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
  if (lidar_frame.empty()) {
    lidar_frame = lidar_msg->header.frame_id;
  }
  if (!check_and_query_extrinsics()) {
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    if (lidar_buffer.size() >= max_lidar_buffer_size) {
      ROS_WARN_STREAM("Registration lidar buffer limit reached. Dropping frame.");
      sync_condition_variable.notify_one();
      return;
    }
  }
  try {
    const auto& [start_stamp, end_stamp, timestamps] =
        rko_lio_ros_utils::process_timestamps(lidar_msg, force_absolute_lidar_timestamps);
    const Vector3dVector scan = rko_lio_ros_utils::point_cloud2_to_eigen(lidar_msg);
    {
      std::lock_guard lock(buffer_mutex);
      lidar_buffer.emplace(start_stamp, end_stamp, timestamps, scan);
      atomic_can_process = !imu_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
    }
    if (atomic_can_process) {
      sync_condition_variable.notify_one();
    }
  } catch (const std::runtime_error& ex) {
    ROS_ERROR("Encountered error, dropping frame: Error: %s", ex.what());
    throw ex;
  }
}

void Node::registration_loop() {
  while (atomic_node_running) {
    SCOPED_PROFILER("ROS Registration Loop");
    std::unique_lock lock(buffer_mutex);
    // sync_condition_variable.wait(lock, [this]() { return !atomic_node_running || atomic_can_process; });
    // once i got stuck on this wait, so im hacking a fix. just because this is ros1 and i do not care
    sync_condition_variable.wait_for(lock, std::chrono::seconds(10),
                                     [this]() { return !atomic_node_running || atomic_can_process; });
    if (!atomic_node_running) {
      // node could have been killed after waiting on the cv
      break;
    }
    if (!atomic_can_process) {
      // maybe we hit the wait limit because we were stuck
      // hopefully something happened in the 5 seconds in between like a kill
      ROS_WARN("woke up the reg thread because we waited too long. something's gone wrong. breaking out.");
      atomic_node_running = false;
      break;
    }
    LidarFrame frame = std::move(lidar_buffer.front());
    lidar_buffer.pop();
    const auto& [start_stamp, end_stamp, timestamps, scan] = frame;
    for (; !imu_buffer.empty() && imu_buffer.front().time < end_stamp; imu_buffer.pop()) {
      lio->update_imu_motion(extrinsic_imu2base, imu_buffer.front());
    }
    // check if there are more messages buffered already
    atomic_can_process =
        !imu_buffer.empty() && !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
    lock.unlock(); // we dont touch the buffers or shared state anymore

    const Vector3dVector deskewed_frame = std::invoke([&]() {
      if (publish_local_map) {
        std::lock_guard lock(lio_mutex);
        return lio->register_scan(extrinsic_lidar2base, scan, timestamps);
      } else {
        return lio->register_scan(extrinsic_lidar2base, scan, timestamps);
      }
    });

    if (!deskewed_frame.empty()) {
      // TODO: first frame is skipped and an empty frame is returned. there should be a better way to handle this
      if (publish_deskewed_cloud) {
        std_msgs::Header header;
        header.frame_id = lidar_frame;
        header.stamp = ros::Time(end_stamp.count());
        frame_publisher.publish(rko_lio_ros_utils::eigen_to_point_cloud2(deskewed_frame, header));
      }
      publish_odometry(lio->lidar_state, end_stamp);
      publish_accel(end_stamp);
    }
  }
}

void Node::publish_odometry(const rko_lio::State& state, const Secondsd& stamp) const {
  const std::string_view from_frame = base_frame;
  const std::string_view to_frame = odom_frame;
  // tf message
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = ros::Time(stamp.count());
  if (invert_odom_tf) {
    transform_msg.header.frame_id = from_frame;
    transform_msg.child_frame_id = to_frame;
    transform_msg.transform = rko_lio_ros_utils::sophus_to_transform(state.pose.inverse());
  } else {
    transform_msg.header.frame_id = to_frame;
    transform_msg.child_frame_id = from_frame;
    transform_msg.transform = rko_lio_ros_utils::sophus_to_transform(state.pose);
  }
  tf_broadcaster->sendTransform(transform_msg);
  // odometry msg
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(stamp.count());
  odom_msg.header.frame_id = to_frame;
  odom_msg.child_frame_id = from_frame;
  odom_msg.pose.pose = rko_lio_ros_utils::sophus_to_pose(state.pose);
  rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(state.velocity, odom_msg.twist.twist.linear);
  rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(state.angular_velocity, odom_msg.twist.twist.angular);
  odom_publisher.publish(odom_msg);
}

void Node::publish_accel(const Secondsd& stamp) const {
  auto accel_msg = geometry_msgs::AccelStamped();
  accel_msg.header.stamp = ros::Time(stamp.count());
  accel_msg.header.frame_id = imu_frame;
  rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(lio->lidar_state.linear_acceleration, accel_msg.accel.linear);
  accel_publisher.publish(accel_msg);

  // auto body_accel_msg = geometry_msgs::AccelStamped();
  // body_accel_msg.header.stamp = ros::Time(stamp.count());
  // body_accel_msg.header.frame_id = imu_frame;
  // rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(lio->trajectory.average_body_acceleration,
  // body_accel_msg.accel.linear);
  // rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(lio->trajectory.average_imu_acceleration,
  // body_accel_msg.accel.angular); body_accel_publisher.publish(body_accel_msg);
}

void Node::publish_map_loop() {
  while (atomic_node_running) {
    std::this_thread::sleep_for(publish_map_after);
    std::unique_lock lock(lio_mutex);
    if (lio->map.Empty()) {
      ROS_WARN_ONCE("Local map publish thread: Local map is empty.");
      continue;
    }
    const Vector3dVector map_points = lio->map.Pointcloud();
    lock.unlock(); // we don't access the map from lio anymore
    std_msgs::Header map_header;
    map_header.stamp = ros::Time::now();
    map_header.frame_id = odom_frame;
    map_publisher.publish(rko_lio_ros_utils::eigen_to_point_cloud2(map_points, map_header));
  }
}

Node::~Node() {
  atomic_node_running = false;
  sync_condition_variable.notify_all();
  registration_thread.join();
  map_publish_thread.join();
}
} // namespace rko_lio_ros
