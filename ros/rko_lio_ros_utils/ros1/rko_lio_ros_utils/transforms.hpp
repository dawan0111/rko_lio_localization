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

#pragma once
#include "aliases.hpp"
#include "ros_vectors.hpp"
// system
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <string>
// ros
#include <geometry_msgs/Pose.h>
#include <tf2_ros/buffer.h>

namespace rko_lio_ros_utils {
template <typename Scalar = double>
Sophus::SE3<Scalar> transform_to_sophus(const geometry_msgs::TransformStamped& transform) {
  const auto& t = transform.transform;
  return {typename Sophus::SE3<Scalar>::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
          typename Sophus::SE3<Scalar>::Point(t.translation.x, t.translation.y, t.translation.z)};
}

template <typename Scalar = double>
std::optional<Sophus::SE3<Scalar>> get_transform(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                 const std::string& from_frame,
                                                 const std::string& to_frame,
                                                 const Secondsd& time,
                                                 const Secondsd& timeout = Secondsd(0)) {
  geometry_msgs::TransformStamped from2to_transform;
  try {
    tf_buffer->_validateFrameId("from_frame", from_frame);
    tf_buffer->_validateFrameId("to frame", to_frame);
    std::unique_ptr<std::string> error_str = std::make_unique<std::string>();
    if (!tf_buffer->canTransform(to_frame, from_frame, ros::Time(time.count()), ros::Duration(timeout.count()),
                                 error_str.get())) {
      ROS_WARN_STREAM("Cannot transfrom from: " << from_frame << " -> to: " << to_frame << " at time: " << time.count()
                                                << " with timeout: " << timeout.count()
                                                << " because of: " << *error_str);
      return std::nullopt;
    }
    from2to_transform = tf_buffer->lookupTransform(to_frame, from_frame, ros::Time(time.count()));
    return transform_to_sophus<Scalar>(from2to_transform);
  } catch (const tf2::InvalidArgumentException& e) {
    ROS_WARN_STREAM("TF lookup error (InvalidArgumentException): " << e.what());
    ROS_WARN_STREAM("Arguments are, to_frame: " << to_frame << ", from_frame: " << from_frame
                                                << ", time: " << time.count());
  } catch (const tf2::LookupException& e) {
    ROS_WARN_STREAM("TF lookup error (LookupException): " << e.what());
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR_STREAM("Could not get the transform from " << from_frame << " to " << to_frame << ": " << ex.what());
  }
  return std::nullopt;
}

inline geometry_msgs::Pose sophus_to_pose(const Sophus::SE3d& se3) {
  geometry_msgs::Pose pose;
  eigen_vector3d_to_ros_xyz(se3.translation(), pose.position);
  Eigen::Quaterniond q(se3.so3().unit_quaternion());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

inline geometry_msgs::Transform sophus_to_transform(const Sophus::SE3d& se3) {
  geometry_msgs::Transform transform;
  eigen_vector3d_to_ros_xyz(se3.translation(), transform.translation);
  Eigen::Quaterniond q(se3.so3().unit_quaternion());
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();
  return transform;
}

} // namespace rko_lio_ros_utils
