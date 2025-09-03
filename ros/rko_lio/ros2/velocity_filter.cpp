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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rko_lio_ros_utils/ros_vectors.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
}
namespace {
using namespace std::literals;
constexpr double EPSILON = 1e-8;
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);
using rko_lio_ros_utils::Secondsd;

inline Eigen::Vector3d gravity() { return {0, 0, -9.8107}; }

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

namespace rko_lio_ros {

class VelocityFilter : public rclcpp::Node {
public:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  std::string odom_topic;
  std::string odom_frame;
  std::string base_frame;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::string imu_topic;
  std::string imu_frame;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr low_pass_velocity_pub;
  std::string low_pass_velocity_topic;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr complementary_filter_velocity_pub;
  std::string complementary_filter_velocity_topic;

  // Parameters
  // should be set based on the required time constant, tau, and sample rate (10Hz for LiDAR).
  // time constant is the time scale of the signal the filter should work on. see the complementary filter reference
  // given later
  // alpha = tau / (tau + sample rate)
  double low_pass_alpha = 0.66;
  double complementary_filter_alpha = 0.66;

  // filters start from zero velocity
  Eigen::Vector6d last_low_pass_velocity = Eigen::Vector6d::Zero();
  Eigen::Vector6d last_complementary_filter_velocity = Eigen::Vector6d::Zero();
  Secondsd last_imu_time{0.0};
  Secondsd last_odom_time{0.0};
  Eigen::Vector3d last_base_frame_angular_velocity;

  Sophus::SE3d extrinsic_imu2base;
  bool extrinsic_set = false;
  using time_compare = decltype([](const ImuData& left, const ImuData& right) { return left.time > right.time; });
  std::priority_queue<ImuData, std::vector<ImuData>, time_compare> imu_buffer;

  explicit VelocityFilter(const rclcpp::NodeOptions& options) : Node("rko_lio_velocity_filter_component", options) {
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), 10s);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    low_pass_alpha = declare_parameter("odom_low_pass_alpha", low_pass_alpha);
    complementary_filter_alpha = declare_parameter("comp_filter_alpha", complementary_filter_alpha);

    base_frame = declare_parameter("base_frame", "base");
    odom_topic = declare_parameter("odom_topic", "/odom");
    odom_frame = declare_parameter("odom_frame", "odom");
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) { odom_callback(odom_msg); });

    imu_topic = declare_parameter("imu_topic", "/imu");
    imu_frame = declare_parameter("imu_frame", "imu");
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 100, [this](const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) { imu_callback(imu_msg); });

    low_pass_velocity_topic = declare_parameter("low_pass_velocity_topic", "/velocity_filter/low_pass_velocity");
    low_pass_velocity_pub = create_publisher<geometry_msgs::msg::TwistStamped>(low_pass_velocity_topic, 10);

    complementary_filter_velocity_topic =
        declare_parameter("complementary_filter_velocity_topic", "/velocity_filter/complementary_filter_velocity");
    complementary_filter_velocity_pub =
        create_publisher<geometry_msgs::msg::TwistStamped>(complementary_filter_velocity_topic, 10);

    RCLCPP_INFO_STREAM(get_logger(),
                       "VelocityFilter subscribed to IMU: "
                           << imu_topic << (!imu_frame.empty() ? " (frame " + imu_frame + ")" : "")
                           << ", Odom: " << odom_topic << (!odom_frame.empty() ? " (frame " + odom_frame + ")" : "")
                           << (!base_frame.empty() ? (". Base frame: " + base_frame) : "")
                           << ". Low-pass velocity filter alpha: " << low_pass_alpha
                           << ", complementary velocity filter alpha: " << complementary_filter_alpha
                           << ". Publishing low-pass velocity to: " << low_pass_velocity_topic
                           << ", complementary filter velocity to: " << complementary_filter_velocity_topic << ".");
    RCLCPP_INFO(this->get_logger(), "VelocityFilter node is up!");
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
    ImuData base_frame_motion = imu_data;
    const Sophus::SO3d& extrinsic_rotation = extrinsic_imu2base.so3();
    base_frame_motion.angular_velocity = extrinsic_rotation * imu_data.angular_velocity;
    // TODO: ensure this matches the final transform equation in rko_lio imu transform
    const Eigen::Vector3d& lever_arm = -1 * extrinsic_imu2base.translation();
    const double dt = (imu_data.time - last_imu_time).count();
    const Eigen::Vector3d angular_acceleration =
        (base_frame_motion.angular_velocity - last_base_frame_angular_velocity) / dt;

    base_frame_motion.acceleration =
        extrinsic_rotation * imu_data.acceleration + angular_acceleration.cross(lever_arm) +
        base_frame_motion.angular_velocity.cross(base_frame_motion.angular_velocity.cross(lever_arm));
    last_base_frame_angular_velocity = base_frame_motion.angular_velocity;
    imu_buffer.emplace(base_frame_motion);
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    if (base_frame.empty()) {
      base_frame = odom_msg->child_frame_id;
    }
    if (odom_frame.empty()) {
      odom_frame = odom_msg->header.frame_id;
    }
    if (!check_and_set_extrinsic()) {
      // need the extrinsic between imu and base
      return;
    }

    const auto& base_twist = odom_msg->twist.twist;
    Eigen::Vector6d base_velocity;
    base_velocity.head<3>() = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(base_twist.linear);
    base_velocity.tail<3>() = rko_lio_ros_utils::ros_xyz_to_eigen_vector3d(base_twist.angular);

    apply_low_pass_filter_and_publish(base_velocity, odom_msg->header.stamp);

    const Secondsd odom_time = rko_lio_ros_utils::ros_time_to_seconds(odom_msg->header.stamp);
    if (last_odom_time < EPSILON_TIME) {
      last_odom_time = odom_time;
      return;
    }

    std::optional<Sophus::SE3d> tf_opt =
        rko_lio_ros_utils::get_transform(tf_buffer, imu_frame, base_frame, last_odom_time);
    if (!tf_opt) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Failed to get transform imu->base at last odom_time: " << last_odom_time.count());
      return;
    }
    const Sophus::SO3d& rotation = tf_opt->so3();

    // average the imu measurements
    Eigen::Vector3d sum_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_angular_velocity = Eigen::Vector3d::Zero();
    int count = 0;
    for (; !imu_buffer.empty() && imu_buffer.top().time <= odom_time; imu_buffer.pop()) {
      const ImuData& imu_data = imu_buffer.top();
      if (imu_data.time < last_odom_time) {
        continue;
      }
      sum_acceleration += imu_data.acceleration;
      sum_angular_velocity += imu_data.angular_velocity;
      ++count;
    }
    if (!count) {
      RCLCPP_WARN(this->get_logger(), "No IMU data available between last odom and current odom msg");
      last_odom_time = odom_time;
      return;
    }

    // NOTE: i'm using the rotation at the start of the integration period to compensate for gravity since its simpler.
    // High frequency motion would require the usual individual acceleration compensation.
    const Eigen::Vector3d gravity_local = rotation.inverse() * gravity();
    const Eigen::Vector3d avg_acceleration = (sum_acceleration / count) + gravity_local;
    const Eigen::Vector3d avg_angular_velocity = sum_angular_velocity / count;

    apply_complementary_filter_and_publish(odom_time, avg_acceleration, avg_angular_velocity, base_velocity);
  }

  void apply_low_pass_filter_and_publish(const Eigen::Vector6d& unfiltered_velocity,
                                         const builtin_interfaces::msg::Time& stamp) {
    const Eigen::Vector6d low_pass_velocity =
        low_pass_alpha * last_low_pass_velocity + (1 - low_pass_alpha) * unfiltered_velocity;
    // publish
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame;
    rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(low_pass_velocity.head<3>(), msg.twist.linear);
    rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(low_pass_velocity.tail<3>(), msg.twist.angular);
    low_pass_velocity_pub->publish(msg);
    // update
    last_low_pass_velocity = low_pass_velocity;
  }

  // https://d1.amobbs.com/bbs_upload782111/files_35/ourdev_608933JR01FI.pdf (The Balance Filter - Shane Colton)
  void apply_complementary_filter_and_publish(const Secondsd& odom_time,
                                              const Eigen::Vector3d& avg_acceleration,
                                              const Eigen::Vector3d& avg_angular_velocity,
                                              const Eigen::Vector6d& low_freq_velocity) {
    const double dt = (odom_time - last_odom_time).count();
    // the "high frequency" (biased) measurement for the complimentary filter
    Eigen::Vector6d imu_velocity;
    imu_velocity.head<3>() = last_complementary_filter_velocity.head<3>() + avg_acceleration * dt;
    imu_velocity.tail<3>() = avg_angular_velocity;

    const Eigen::Vector6d complementary_filter_velocity =
        complementary_filter_alpha * imu_velocity + (1.0 - complementary_filter_alpha) * low_freq_velocity;
    // publish
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(odom_time).count());
    msg.header.frame_id = base_frame;
    rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(complementary_filter_velocity.head<3>(), msg.twist.linear);
    rko_lio_ros_utils::eigen_vector3d_to_ros_xyz(complementary_filter_velocity.tail<3>(), msg.twist.angular);
    complementary_filter_velocity_pub->publish(msg);
    // update state
    last_complementary_filter_velocity = complementary_filter_velocity;
    last_odom_time = odom_time;
  }
};

} // namespace rko_lio_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rko_lio_ros::VelocityFilter)
