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
#include "util.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <teaser/registration.h> // libteaser_registration

namespace rko_lio::core {

class GlobalMapMatcher {
public:
  GlobalMapMatcher(std::string& global_map_path, const std::string& lidar_tf_name);
  // Sophus::SE3d solve(const LidarFrame& lidar_frame);

private:
  Sophus::SE3d getWorldPose_();
  Vector3dVector transformPoint_(const Sophus::SE3d& pose, const Vector3dVector& points);
  std::tuple<Vector3dVector, Vector3dVector> getMapCorrespondences(const Vector3dVector& points, int max_size = 4000);

  std::unique_ptr<teaser::RobustRegistrationSolver> solver_ptr_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  std::string global_map_path_;
  std::string lidar_tf_name_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_{new pcl::PointCloud<pcl::PointXYZ>};
};
} // namespace rko_lio::core