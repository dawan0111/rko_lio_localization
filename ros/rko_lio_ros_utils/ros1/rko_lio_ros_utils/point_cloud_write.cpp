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

#include "point_cloud_write.hpp"
#include <regex>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
inline std::string FixFrameId(const std::string& frame_id) {
  return std::regex_replace(frame_id, std::regex("^/"), "");
}
} // namespace

namespace rko_lio_ros_utils {
sensor_msgs::PointCloud2Ptr eigen_to_point_cloud2(const Vector3dVector& points, const std_msgs::Header& header) {
  auto cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();

  // create the msg
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
  cloud_msg->fields.clear();
  int offset = 0;
  offset = addPointField(*cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);

  constexpr bool timestamp = false;
  if constexpr (timestamp) {
    // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
    offset = addPointField(*cloud_msg, "time", 1, sensor_msgs::PointField::FLOAT64, offset);
    offset += sizeOfPointField(sensor_msgs::PointField::FLOAT64);
  }

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(static_cast<size_t>(cloud_msg->height) * cloud_msg->row_step);
  modifier.resize(points.size());

  // fill the msg
  sensor_msgs::PointCloud2Iterator<float> msg_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(*cloud_msg, "z");
  for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
    const Eigen::Vector3d& point = points[i];
    *msg_x = static_cast<float>(point.x());
    *msg_y = static_cast<float>(point.y());
    *msg_z = static_cast<float>(point.z());
  }
  return cloud_msg;
}

sensor_msgs::PointCloud2Ptr eigen_to_point_cloud2_with_intensities(const Vector3dVector& points,
                                                                   const IntensityVector& intensities,
                                                                   const std_msgs::Header& header) {
  assert(points.size() == intensities.size());

  // Create PointCloud2 message with intensity field
  auto cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
  cloud_msg->fields.clear();

  int offset = 0;
  offset = addPointField(*cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);

  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(static_cast<size_t>(cloud_msg->height) * cloud_msg->row_step);

  modifier.resize(points.size());

  // Fill the data
  sensor_msgs::PointCloud2Iterator<float> msg_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> msg_intensity(*cloud_msg, "intensity");

  for (size_t i = 0; i < points.size(); ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_intensity) {
    const Eigen::Vector3d& point = points[i];
    *msg_x = static_cast<float>(point.x());
    *msg_y = static_cast<float>(point.y());
    *msg_z = static_cast<float>(point.z());
    *msg_intensity = static_cast<float>(intensities[i]);
  }

  return cloud_msg;
}
} // namespace rko_lio_ros_utils
