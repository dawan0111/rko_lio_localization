// global_map_matcher.cpp — constructor-only + stubbed privates
// Builds with the provided header; only the constructor is functional.
// - Loads global map PLY
// - Cleans NaNs
// - (Optional) voxel downsample (disabled by default; toggle kVoxelLeafSize)
// - Initializes KDTree and TEASER++ solver
//
// Private helpers are stubbed to return empty/default values so this compiles.

#include "global_map_matcher.hpp" // your header from the message

namespace rko_lio::core {

namespace {
// Toggle downsampling if desired
constexpr float kVoxelLeafSize = 0.0f; // set >0 (e.g., 0.10f) to enable
} // namespace

// ------------------------------ Constructor ------------------------------
GlobalMapMatcher::GlobalMapMatcher(std::string& global_map_path, const std::string& lidar_tf_name)
    : global_map_path_(global_map_path), lidar_tf_name_(lidar_tf_name) {
  // 1) Load global map from PLY
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(global_map_path_, *global_map_) != 0) {
    throw std::runtime_error("GlobalMapMatcher: failed to load PLY: " + global_map_path_);
  }

  // 2) Remove NaNs
  {
    std::vector<int> idx; // unused indices output
    pcl::removeNaNFromPointCloud(*global_map_, *global_map_, idx);
  }

  // 3) Optional voxel downsample (disabled by default)
  if (kVoxelLeafSize > 0.f) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(kVoxelLeafSize, kVoxelLeafSize, kVoxelLeafSize);
    vg.setInputCloud(global_map_);
    auto map_ds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*map_ds);
    global_map_.swap(map_ds);
  }

  if (global_map_->empty()) {
    throw std::runtime_error("GlobalMapMatcher: global map is empty after preprocessing");
  }

  // 4) KDTree over the map
  kdtree_.setInputCloud(global_map_);

  // 5) Initialize TEASER++ solver with reasonable defaults
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.03; // meters; tune for your LiDAR noise
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  params.rotation_cost_threshold = 0.005;

  solver_ptr_ = std::make_unique<teaser::RobustRegistrationSolver>(params);

  std::cout << "[GlobalMapMatcher] Loaded map (" << global_map_->size() << " pts), "
            << "lidar_tf='" << lidar_tf_name_ << "'\n";
}

// --------------------------- Stubbed privates ----------------------------
// These are placeholders to satisfy the linker; wire them up later.

Sophus::SE3d GlobalMapMatcher::getWorldPose_() {
  return Sophus::SE3d(); // identity
}

Vector3dVector GlobalMapMatcher::transformPoint_(const Sophus::SE3d& /*pose*/, const Vector3dVector& /*points*/) {
  return {}; // empty
}

std::tuple<Vector3dVector, Vector3dVector> GlobalMapMatcher::getMapCorrespondences(const Vector3dVector& /*points*/,
                                                                                   int /*max_size*/) {
  return std::make_tuple(Vector3dVector{}, Vector3dVector{});
}

} // namespace rko_lio::core
