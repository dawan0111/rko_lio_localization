/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
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

#include "rko_lio_ros_utils/rko_lio_ros_utils.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

namespace {
using namespace std::literals;
constexpr double EPSILON = 1e-8;
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);
using rko_lio_ros_utils::Secondsd;

struct ImuData {
  Secondsd time{0};
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

ImuData imu_msg_to_imu_data(const sensor_msgs::msg::Imu& imu_msg) {
  ImuData imu_data;
  imu_data.time = rko_lio_ros_utils::ros_time_to_seconds(imu_msg.header.stamp);
  imu_data.angular_velocity = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.angular_velocity);
  imu_data.acceleration = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.linear_acceleration);
  return imu_data;
}
} // namespace

class ImuComplementaryFilter : public rclcpp::Node {
public:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  double alpha;
  std::string imu_topic;
  std::string imu_frame;
  std::string base_frame;
  std::string attitude_topic;
  std::string odom_frame;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  double roll{};
  double pitch{};
  Secondsd last_imu_time{0.0};
  Eigen::Vector3d last_base_frame_angular_velocity;

  Sophus::SE3d extrinsic_imu2base;
  bool extrinsic_set = false;

  ImuComplementaryFilter()
      : Node("rko_lio_attitude_filter"),
        alpha(declare_parameter("alpha", 0.98)),
        imu_topic(declare_parameter("imu_topic", "/imu")),
        imu_frame(declare_parameter("imu_frame", "imu")),
        base_frame(declare_parameter("base_frame", "base")),
        odom_frame(declare_parameter("odom_frame", "odom_attitude")),
        attitude_topic(declare_parameter("attitude_topic", "/attitude_filter/pose")) {

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), 10s);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 100, [this](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) { imu_callback(imu_msg); });

    odom_pub = create_publisher<nav_msgs::msg::Odometry>(attitude_topic, 10);

    RCLCPP_INFO_STREAM(get_logger(),
                       "Attitude filter node started. Alpha: " << alpha << " Subscribed to : " << imu_topic
                                                               << ", publishing pose to: " << attitude_topic);
  }

  bool check_and_set_extrinsic() {
    if (extrinsic_set) {
      return true;
    }
    const std::optional<Sophus::SE3d> imu_transform =
        rko_lio_ros_utils::get_transform(tf_buffer, imu_frame, base_frame, 0s);
    if (!imu_transform) {
      return false;
    }
    extrinsic_imu2base = imu_transform.value();
    extrinsic_set = true;
    return true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    if (imu_frame.empty()) {
      imu_frame = imu_msg->header.frame_id;
    }
    if (!check_and_set_extrinsic()) {
      return;
    }
    const ImuData imu_data = imu_msg_to_imu_data(*imu_msg);
    if (last_imu_time < EPSILON_TIME) {
      RCLCPP_INFO(this->get_logger(), "Skipping first IMU message.");
      last_imu_time = imu_data.time;
      return;
    }
    auto compute_base_frame_imu = [&](const ImuData& imu, const double dt) {
      ImuData base_frame_motion = imu;
      const Sophus::SO3d& extrinsic_rotation = extrinsic_imu2base.so3();
      base_frame_motion.angular_velocity = extrinsic_rotation * imu.angular_velocity;
      const Eigen::Vector3d& lever_arm = -1 * extrinsic_imu2base.translation();
      const Eigen::Vector3d angular_acceleration =
          (base_frame_motion.angular_velocity - last_base_frame_angular_velocity) / dt;

      base_frame_motion.acceleration =
          extrinsic_rotation * imu.acceleration + angular_acceleration.cross(lever_arm) +
          base_frame_motion.angular_velocity.cross(base_frame_motion.angular_velocity.cross(lever_arm));
      last_base_frame_angular_velocity = base_frame_motion.angular_velocity;
      return base_frame_motion;
    };
    const double dt = (imu_data.time - last_imu_time).count();
    const ImuData base_frame_imu = compute_base_frame_imu(imu_data, dt);
    last_imu_time = imu_data.time;

    // Gyro integration
    const Eigen::Vector3d& omega = imu_data.angular_velocity;
    double roll_gyro = roll + omega.x() * dt;
    double pitch_gyro = pitch + omega.y() * dt;

    // Get the roll pitch from accelerometer
    const Eigen::Vector3d& accel = imu_data.acceleration;
    // from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
    double roll_acc = std::atan2(accel.y(), accel.z());
    double pitch_acc = std::atan2(-accel.x(), std::sqrt(accel.y() * accel.y() + accel.z() * accel.z()));
    // from MEMS Based IMU for Tilting Measurement: Comparison of Complementary and Kalman Filter Based Data Fusion
    // double pitch_acc = std::atan2(accel.z(), accel.x());
    // double roll_acc = std::atan2(accel.z(), accel.y());

    // Complementary filter
    roll = alpha * roll_gyro + (1.0 - alpha) * roll_acc;
    pitch = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc;

    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

    // Publish pose
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = imu_msg->header.stamp;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_pub->publish(odom_msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuComplementaryFilter>());
  rclcpp::shutdown();
  return 0;
}
