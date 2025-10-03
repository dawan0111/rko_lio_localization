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
#include "sparse_voxel_grid.hpp"
#include "util.hpp"
#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace rko_lio::core {

using OneCorrespondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = tbb::concurrent_vector<OneCorrespondence>;

class GlobalMapMatcher {
public:
  SparseVoxelGrid global_map;

  GlobalMapMatcher(const std::string& global_map_path,
                   const double voxel_size,
                   const double clipping_distance,
                   const unsigned int max_points_per_voxel)
      : global_map_path_(global_map_path), global_map(voxel_size, clipping_distance, max_points_per_voxel) {
    initialize();
  };
  Sophus::SE3d solve(const Correspondences& correspondences);

private:
  void initialize();
  Vector3dVector transformPoint_(const Sophus::SE3d& pose, const Vector3dVector& points);

  std::unique_ptr<teaser::RobustRegistrationSolver> solver_ptr_;
  std::string global_map_path_;
  std::string lidar_tf_name_;
};
} // namespace rko_lio::core