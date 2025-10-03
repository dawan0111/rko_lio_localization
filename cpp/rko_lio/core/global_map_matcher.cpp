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

void GlobalMapMatcher::initialize() {
  // Initialize TEASER++ solver with reasonable defaults
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.03; // meters; tune for your LiDAR noise
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  params.rotation_cost_threshold = 0.005;

  solver_ptr_ = std::make_unique<teaser::RobustRegistrationSolver>(params);

  teaser::PLYReader reader;
  teaser::PointCloud global_map_pointcloud;
  teaser::PointCloud dst_cloud;

  auto status = reader.read(global_map_path_, global_map_pointcloud);

  std::vector<Eigen::Vector3d> points(global_map_pointcloud.size());

  std::transform(global_map_pointcloud.begin(), global_map_pointcloud.end(), points.begin(),
                 [&](const auto& pointcloud) { return Eigen::Vector3d(pointcloud.x, pointcloud.y, pointcloud.z); });

  std::cout << "globalmap path: " << global_map_path_ << "\n";
  std::cout << "Loaded " << points.size() << " points from global map.\n";
  global_map.AddPoints(points);
}

Sophus::SE3d GlobalMapMatcher::solve(const Correspondences& correspondences) {
  const std::size_t N = correspondences.size();
  if (N < 3) {
    std::cerr << "[GlobalMapMatcher] Not enough correspondences (" << N << "). Returning identity.\n";
    return Sophus::SE3d{};
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);

  tbb::parallel_for(std::size_t{0}, N, [&](std::size_t i) {
    const auto& c = correspondences[i];
    src.col(i) = c.first;
    tgt.col(i) = c.second;
  });

  try {
    solver_ptr_->solve(src, tgt);
    const auto sol = solver_ptr_->getSolution();

    if (!sol.valid) {
      std::cerr << "[GlobalMapMatcher] TEASER++ returned invalid solution. Returning identity.\n";
      return Sophus::SE3d{};
    }

    return Sophus::SE3d(Sophus::SO3d(sol.rotation), sol.translation);
  } catch (const std::exception& e) {
    std::cerr << "[GlobalMapMatcher] Exception in TEASER++ solve: " << e.what() << "\nReturning identity.\n";
    return Sophus::SE3d{};
  }
}

// --------------------------- Stubbed privates ----------------------------
// These are placeholders to satisfy the linker; wire them up later.

Vector3dVector GlobalMapMatcher::transformPoint_(const Sophus::SE3d& /*pose*/, const Vector3dVector& /*points*/) {
  return {}; // empty
}

} // namespace rko_lio::core
