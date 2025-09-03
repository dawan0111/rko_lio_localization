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

#pragma once
#include "aliases.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace rko_lio_ros_utils {
// force_absolute treats point stamps as absolute values
std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const sensor_msgs::PointCloud2ConstPtr& msg,
                                                                   const bool force_absolute = false);

Vector3dVector point_cloud2_to_eigen(const sensor_msgs::PointCloud2ConstPtr& msg);

template <typename Scalar>
std::pair<std::vector<Eigen::Matrix<Scalar, 3, 1>>, IntensityVector>
point_cloud2_to_eigen_with_intensities(const sensor_msgs::PointCloud2ConstPtr& msg) {
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  std::vector<Eigen::Matrix<Scalar, 3, 1>> points;
  IntensityVector intensities;
  points.reserve(point_count);
  intensities.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> msg_intensity(*msg, "intensity");
  for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_intensity) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
    intensities.emplace_back(*msg_intensity);
  }
  return {points, intensities};
}
} // namespace rko_lio_ros_utils
