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

#include "lio.hpp"
#include "rko_lio/util.hpp"
#include "voxel_down_sample.hpp"
// other
#include <nlohmann/json.hpp>
#include <pb_utils/profiler.hpp>
#include <sophus/se3.hpp>
// tbb
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
// stl
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>

namespace {
// correspondence - original point and corresponding map point, in that order
using OneCorrespondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Correspondences = tbb::concurrent_vector<OneCorrespondence>;
Correspondences data_association(const Sophus::SE3d& pose,
                                 const rko_lio::Vector3dVector& points,
                                 const rko_lio::SparseVoxelGrid& voxel_map,
                                 const rko_lio::LIO::Config& config) {
  const int max_threads = config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  static const auto tbb_control_settings =
      tbb::global_control(tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_threads));
  using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
  Correspondences correspondences;
  correspondences.reserve(points.size());
  tbb::parallel_for(
      // Range
      tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
      [&](const tbb::blocked_range<points_iterator>& r) {
        std::for_each(r.begin(), r.end(), [&](const auto& point) {
          // transform the source point here and get the corresponding map point
          const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(pose * point);
          if (distance < config.max_correspondance_distance) {
            correspondences.emplace_back(point, closest_neighbor);
          }
        });
      });
  return correspondences;
}
} // namespace

namespace {
constexpr double EPSILON = 1e-8;
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);
using rko_lio::gravity;
using rko_lio::GRAVITY_MAG;
using rko_lio::Secondsd;
using rko_lio::square;
using rko_lio::TimestampVector;
using rko_lio::Vector3dVector;

void transform_points(const Sophus::SE3d& T, Vector3dVector& points) {
  std::transform(points.begin(), points.end(), points.begin(), [&](const auto& point) { return T * point; });
}

// can't use concepts because that'll break the ros1 version
template <class Functor>
Vector3dVector preprocess_scan(const Vector3dVector& frame,
                               const TimestampVector& timestamps,
                               const Secondsd end_time,
                               const Functor& relative_pose_at_time,
                               const rko_lio::LIO::Config config) {
  const std::vector<Eigen::Vector3d>& deskewed_frame = std::invoke([&]() {
    if (!config.deskew) {
      return frame;
    }
    const Sophus::SE3d& scan_to_scan_motion_inverse = relative_pose_at_time(end_time).inverse();
    Vector3dVector deskewed_frame(frame.size());
    std::transform(frame.cbegin(), frame.cend(), timestamps.cbegin(), deskewed_frame.begin(),
                   [&](const Eigen::Vector3d& point, const Secondsd timestamp) {
                     const auto pose = scan_to_scan_motion_inverse * relative_pose_at_time(timestamp);
                     return pose * point;
                   });
    return deskewed_frame;
  });
  std::vector<Eigen::Vector3d> preprocessed_frame;
  preprocessed_frame.reserve(deskewed_frame.size());
  std::for_each(deskewed_frame.cbegin(), deskewed_frame.cend(), [&](const auto& point) {
    const double point_range = point.norm();
    if (point_range > config.min_range && point_range < config.max_range) {
      preprocessed_frame.emplace_back(point);
    }
  });
  preprocessed_frame.shrink_to_fit();
  return preprocessed_frame;
}

inline Eigen::Vector3d compute_point_to_point_residual(const Sophus::SE3d& pose,
                                                       const OneCorrespondence& correspondence) {
  const auto& [source, target] = correspondence;
  const Eigen::Vector3d residual = (pose * source) - target;
  return residual;
}

// inline Eigen::Vector3d compute_complementary_orientation_residual(const Sophus::SO3d& rotation_complementary,
//                                                                   const Sophus::SO3d& current_rotation) {
//   const Eigen::Vector3d z_world = {0, 0, 1};
//   const Eigen::Vector3d error = (rotation_complementary * z_world).cross(current_rotation * z_world);
//   return error;
// }

inline Eigen::Vector3d compute_acceleration_cost_residual(const Eigen::Vector3d& local_gravity_estimate,
                                                          const Sophus::SO3d& current_rotation) {
  const Eigen::Vector3d predicted_gravity =
      current_rotation.inverse() * (-1 * gravity()); // points upwards, same as local_gravity_estimate
  const Eigen::Vector3d error = predicted_gravity - local_gravity_estimate;
  return error;
}
using LinearSystem = std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double>;
LinearSystem build_linear_system(const Sophus::SE3d& current_pose,
                                 const Correspondences& correspondences,
                                 const Eigen::Vector3d& local_gravity_estimate,
                                 double& beta_ori) {
  auto linear_system_reduce = [](LinearSystem lhs, const LinearSystem& rhs) {
    auto& [lhs_H, lhs_b, lhs_chi] = lhs;
    const auto& [rhs_H, rhs_b, rhs_chi] = rhs;
    lhs_H += rhs_H;
    lhs_b += rhs_b;
    lhs_chi += rhs_chi;
    return lhs;
  };

  auto calculate_left_icp_jacobian = [](const Sophus::SE3d& current_pose, const OneCorrespondence& correspondence) {
    const auto& [source, _] = correspondence;
    Eigen::Matrix3_6d J_icp_l = Eigen::Matrix3_6d::Zero();
    J_icp_l.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_icp_l.block<3, 3>(0, 3) = -1 * Sophus::SO3d::hat(current_pose * source);
    return J_icp_l;
  };

  const auto& [H_icp, b_icp, chi_icp] =
      std::transform_reduce(correspondences.cbegin(), correspondences.cend(),
                            LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0), linear_system_reduce,
                            // transform
                            [&](const auto& correspondence) {
                              const Eigen::Vector3d residual =
                                  compute_point_to_point_residual(current_pose, correspondence);
                              // const auto J = calculate_right_icp_jacobian(current_pose, correspondence);
                              const auto J = calculate_left_icp_jacobian(current_pose, correspondence);
                              return LinearSystem(J.transpose() * J,        // JT * R.inv() * J
                                                  J.transpose() * residual, // JT * R.inv() * r
                                                  residual.squaredNorm());  // chi
                            });

  if (beta_ori < 0) {
    return {H_icp / correspondences.size(), b_icp / correspondences.size(), 0.5 * chi_icp};
  }

  auto calculate_left_acceleration_jacobian = [](const Eigen::Vector3d& local_gravity_estimate,
                                                 const Sophus::SO3d& current_rotation) {
    Eigen::Matrix3_6d J_ori = Eigen::Matrix3_6d::Zero();
    J_ori.block<3, 3>(0, 3) = current_rotation.inverse().matrix() * Sophus::SO3d::hat(-1 * gravity()).matrix();
    return J_ori;
  };
  const auto& [H_ori, b_ori, chi_ori] = std::invoke([&]() {
    const Eigen::Vector3d residual = compute_acceleration_cost_residual(local_gravity_estimate, current_pose.so3());
    const Eigen::Matrix3_6d J_ori = calculate_left_acceleration_jacobian(local_gravity_estimate, current_pose.so3());
    return LinearSystem{J_ori.transpose() * J_ori, J_ori.transpose() * residual, residual.squaredNorm()};
  });
  // constexpr double min_beta = 250;
  // compute beta_ori like before later and clip it with min (which is the config param as initial beta_ori)
  // beta_ori = std::min(beta_ori, min_beta);
  return {H_icp / correspondences.size() + H_ori / beta_ori, b_icp / correspondences.size() + b_ori / beta_ori,
          0.5 * (chi_icp + chi_ori)};
}

Sophus::SE3d icp(const Vector3dVector& frame,
                 const rko_lio::SparseVoxelGrid& voxel_map,
                 const Sophus::SE3d& initial_guess,
                 const Eigen::Vector3d& local_gravity_estimate,
                 const double accel_mag_variance,
                 const rko_lio::LIO::Config& config) {
  Sophus::SE3d current_pose = initial_guess;
  // gets adaptively computed in the first iteration of build_linear_system
  double beta_ori = config.min_beta * (1 + accel_mag_variance);
  // double beta_ori = config.min_beta + accel_mag_variance;

  for (size_t i = 0; i < config.max_iterations; ++i) {
    const Correspondences& correspondences = data_association(current_pose, frame, voxel_map, config);
    if (correspondences.empty()) {
      throw std::runtime_error("Number of correspondences are 0.");
    }
    const auto& [H, b, chi] = build_linear_system(current_pose, correspondences, local_gravity_estimate, beta_ori);
    const Eigen::Vector6d dx = H.ldlt().solve(-b);
    current_pose = Sophus::SE3d::exp(dx) * current_pose; // left jacobian version
    if (dx.norm() < config.convergence_criterion || i == (config.max_iterations - 1)) {
      std::cout << "iter " << i << ", beta_ori: " << beta_ori << ", chi: " << chi
                << ", assoc: " << correspondences.size() << "\n";
      // std::cout << "===========================================================\n";
      break;
    }
  }
  return current_pose;
}

// align accel (IMU/base frame) to +z (world)
Sophus::SO3d align_accel_to_z_world(const Eigen::Vector3d& accel) {
  //  partially unobservable in the gravity direction, and the z in R_accel.log() will actually always be 0
  const Eigen::Vector3d z_world = {0, 0, 1};
  const Eigen::Quaterniond quat_accel = Eigen::Quaterniond::FromTwoVectors(accel, z_world);
  return Sophus::SO3d(quat_accel);
}
} // namespace

// ==========================
//   actual LIO class stuff
// ==========================//
namespace rko_lio {

void LIO::update_imu_motion(const ImuControl& base_imu) {
  if (lidar_state.time < EPSILON_TIME) {
    std::cout << "Skipping IMU, waiting for first LiDAR message.\n";
    _last_real_imu_time = base_imu.time;
    _last_real_base_imu_ang_vel = base_imu.angular_velocity;
    return;
  }
  if (_imu_local_rotation_time < EPSILON_TIME) {
    _imu_local_rotation_time = lidar_state.time;
  }
  const double dt = (base_imu.time - _imu_local_rotation_time).count();
  if (dt < 0) {
    // should not happen if the ros node is bufferering messages correctly. if it is, messages are out of sync.
    // this will mess up the acceleration compensation, since we integrate gyro from last lidar time onwards
    std::cout << "WARNING: Imu message at time " << base_imu.time.count()
              << " is behind the last local imu rotation time: " << _imu_local_rotation_time.count()
              << ", last lidar time is: " << lidar_state.time.count() << "\n";
    // skip this imu reading?
  }
  ++_interval_imu_count;
  const Eigen::Vector3d unbiased_ang_vel = base_imu.angular_velocity - imu_bias.gyroscope;
  const Eigen::Vector3d unbiased_accel = base_imu.acceleration - imu_bias.accelerometer;
  _interval_angular_velocity_sum += unbiased_ang_vel;
  _interval_imu_acceleration_sum += unbiased_accel;
  const double previous_imu_accel_mag_mean = _interval_imu_accel_mag_mean;
  _interval_imu_accel_mag_mean += (unbiased_accel.norm() - previous_imu_accel_mag_mean) / _interval_imu_count;
  _interval_welford_sum_of_squares +=
      (unbiased_accel.norm() - previous_imu_accel_mag_mean) * (unbiased_accel.norm() - _interval_imu_accel_mag_mean);

  _imu_local_rotation = _imu_local_rotation * Sophus::SO3d::exp(unbiased_ang_vel * dt);
  _imu_local_rotation_time = base_imu.time;

  const Eigen::Vector3d local_gravity = _imu_local_rotation.inverse() * gravity();
  _interval_body_acceleration_sum += (unbiased_accel + local_gravity);

  // TODO: Maintain an imu state here, and then update it to publish later. should be easy enough
  _last_real_imu_time = base_imu.time;
  _last_real_base_imu_ang_vel = base_imu.angular_velocity;
}

void LIO::update_imu_motion(const Sophus::SE3d& extrinsic_imu2base, const ImuControl& raw_imu) {
  if (extrinsic_imu2base.log().norm() < EPSILON) {
    update_imu_motion(raw_imu);
    return;
  }
  if (_last_real_imu_time < EPSILON_TIME) {
    std::cout << "Skipping IMU message as we need a previous imu time for extrinsic compensation.\n";
    _last_real_imu_time = raw_imu.time;
    return;
  }
  ImuControl base_imu = raw_imu;
  const Sophus::SO3d& extrinsic_rotation = extrinsic_imu2base.so3();
  base_imu.angular_velocity = extrinsic_rotation * raw_imu.angular_velocity;
  base_imu.angular_velocity_covariance =
      extrinsic_rotation.matrix() * raw_imu.angular_velocity_covariance * extrinsic_rotation.inverse().matrix();
  const Eigen::Vector3d& lever_arm = -1 * extrinsic_imu2base.translation();
  const Secondsd dt = raw_imu.time - _last_real_imu_time;
  const Eigen::Vector3d angular_acceleration = std::invoke([&]() -> Eigen::Vector3d {
    if (std::chrono::abs(dt) < Secondsd(1.0 / 5000.0)) {
      // there's cases where imu messages appear very close to each other (DigiForest data), but still with a +ve dt
      // if the dt is less than the equivalent of a 5000 Hz imu, im assuming zero ang accel, because otherwise the accel
      // is messed up due to the dt divide
      std::cout << "WARNING: IMU message too close to the last received IMU message. dt is: " << dt.count()
                << ". Ignoring angular acceleration for this integration.\n";
      return Eigen::Vector3d::Zero();
    } else {
      const Eigen::Vector3d angular_acceleration =
          (base_imu.angular_velocity - _last_real_base_imu_ang_vel) / dt.count();
      return angular_acceleration;
    }
  });
  base_imu.acceleration = extrinsic_rotation * raw_imu.acceleration + angular_acceleration.cross(lever_arm) +
                          base_imu.angular_velocity.cross(base_imu.angular_velocity.cross(lever_arm));
  base_imu.acceleration_covariance =
      extrinsic_rotation.matrix() * raw_imu.acceleration_covariance * extrinsic_rotation.inverse().matrix();
  this->update_imu_motion(base_imu);
}

void LIO::initialize(const Secondsd lidar_time) {
  std::cout << "Initializing LIO.\n";
  if (_interval_imu_count == 0) {
    throw std::runtime_error("No imu measurements to use for initialization.");
  }
  const Eigen::Vector3d avg_accel = _interval_imu_acceleration_sum / _interval_imu_count;
  const Eigen::Vector3d avg_gyro = _interval_angular_velocity_sum / _interval_imu_count;
  _imu_local_rotation = align_accel_to_z_world(avg_accel);
  _imu_local_rotation_time = lidar_time;
  lidar_state.pose.so3() = _imu_local_rotation;
  lidar_state.time = lidar_time;

  const Eigen::Vector3d local_gravity = _imu_local_rotation.inverse() * gravity();
  imu_bias.accelerometer = avg_accel + local_gravity;
  imu_bias.gyroscope = avg_gyro;
  _initialized = true;
  std::cout << "LIO initialized using " << _interval_imu_count << " IMU measurements. Estimated starting rotation is "
            << _imu_local_rotation.log().transpose() << ". Estimated accel bias: " << imu_bias.accelerometer.transpose()
            << ", gyro bias: " << imu_bias.gyroscope.transpose() << "\n";
}

void LIO::update_body_acceleration(const Eigen::Vector3d& imu_accel,
                                   const double accel_mag_variance,
                                   const Sophus::SO3d& rotation_estimate,
                                   const Secondsd& time) {
  const double dt = (time - lidar_state.time).count();
  const Eigen::Vector3d& body_accel_measurement = imu_accel + rotation_estimate.inverse() * gravity();

  const double max_acceleration_change = config.max_expected_jerk * dt;
  // assume [j, -j] range for uniform dist. on jerk. variance is (2j)^2 / 12 = j^2/3. multiply by dt^2 for accel
  const Eigen::Matrix3d process_noise = square(max_acceleration_change) / 3 * Eigen::Matrix3d::Identity();
  body_acceleration_covariance += process_noise;

  // isotropic accel mag variance
  const Eigen::Matrix3d measurement_noise = accel_mag_variance / 3 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = body_acceleration_covariance + measurement_noise;
  const Eigen::Matrix3d kalman_gain = body_acceleration_covariance * S.inverse();

  const Eigen::Vector3d innovation = kalman_gain * (body_accel_measurement - mean_body_acceleration);
  mean_body_acceleration += innovation;
  body_acceleration_covariance -= kalman_gain * body_acceleration_covariance;
}

Vector3dVector LIO::register_scan(const Vector3dVector& scan, const TimestampVector& timestamps) {
  const auto max = std::max_element(timestamps.cbegin(), timestamps.cend());
  const Secondsd current_lidar_time = *max;

  if (lidar_state.time < EPSILON_TIME) {
    lidar_state.time = current_lidar_time;
    std::cout << "First LiDAR received, using as global frame.";
    return {};
  }

  const auto& [avg_body_accel, avg_ang_vel] = std::invoke([&]() -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
    if (config.initialization_phase && !_initialized) {
      // assume static and
      initialize(current_lidar_time);
      return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }
    if (_interval_imu_count == 0) {
      std::cout << "WARNING: No Imu measurements in interval to average. Assuming constant velocity motion.\n";
      return {Eigen::Vector3d::Zero(), lidar_state.angular_velocity};
    }
    const Eigen::Vector3d avg_body_accel = _interval_body_acceleration_sum / _interval_imu_count;
    const Eigen::Vector3d avg_ang_vel = _interval_angular_velocity_sum / _interval_imu_count;
    // log_in_gray("interval count ", _interval_imu_count);
    // log_in_gray("avg body acceleration ", avg_body_accel.transpose());
    // log_in_gray("interval imu accel mag mean ", _interval_imu_accel_mag_mean);
    // log_in_gray("interval imu accel mag welford variance ",
    //             _interval_welford_sum_of_squares / (_interval_imu_count - 1));
    // log_in_gray("variance inverse ", 1 / (_interval_welford_sum_of_squares / (_interval_imu_count - 1)));

    // log_in_gray("interval imu accel sum", _interval_imu_acceleration_sum.transpose());
    // log_in_gray("interval body accel sum", _interval_body_acceleration_sum.transpose());
    if (avg_body_accel.norm() > 50) {
      throw std::runtime_error("shit acceleration. check");
    }
    return {avg_body_accel, avg_ang_vel};
  });

  auto relative_pose_at_time = [&](const Secondsd time) -> Sophus::SE3d {
    const double dt = (time - lidar_state.time).count();
    Eigen::Matrix<double, 6, 1> tau;
    tau.head<3>() = lidar_state.velocity * dt + (avg_body_accel * square(dt) / 2);
    tau.tail<3>() = avg_ang_vel * dt;
    return Sophus::SE3d::exp(tau);
  };
  const Sophus::SE3d initial_guess = lidar_state.pose * relative_pose_at_time(current_lidar_time);

  // acceleration filter
  const Eigen::Vector3d avg_imu_accel = _interval_imu_acceleration_sum / _interval_imu_count;
  const double accel_mag_variance = _interval_welford_sum_of_squares / (_interval_imu_count - 1);
  update_body_acceleration(avg_imu_accel, accel_mag_variance, initial_guess.so3(), current_lidar_time);
  const Eigen::Vector3d local_gravity_estimate = avg_imu_accel - mean_body_acceleration; // points upwards
  // const Eigen::Vector3d local_gravity_estimate = avg_imu_accel; // points upwards

  const Vector3dVector preprocessed_scan =
      preprocess_scan(scan, timestamps, current_lidar_time, relative_pose_at_time, config);
  const auto& [frame_downsample, keypoints] = std::invoke([&]() -> std::pair<Vector3dVector, Vector3dVector> {
    if (config.double_downsample) {
      const Vector3dVector frame_downsample = voxel_down_sample(preprocessed_scan, config.voxel_size * 0.5);
      const Vector3dVector keypoints = voxel_down_sample(frame_downsample, config.voxel_size * 1.5);
      return {frame_downsample, keypoints};
    } else {
      const Vector3dVector frame_downsample = voxel_down_sample(preprocessed_scan, config.voxel_size);
      return {frame_downsample, frame_downsample};
    }
  });

  if (!map.Empty()) {
    SCOPED_PROFILER("ICP");
    const Sophus::SE3d optimized_pose =
        icp(keypoints, map, initial_guess, local_gravity_estimate, accel_mag_variance, config);
    const double dt = (current_lidar_time - lidar_state.time).count();
    // estimate velocities and accelerations from the new pose
    const Sophus::SE3d motion = lidar_state.pose.inverse() * optimized_pose;
    const Eigen::Vector6d local_velocity = motion.log() / dt;
    const Eigen::Vector3d local_linear_acceleration =
        (local_velocity.head<3>() - motion.so3().inverse() * lidar_state.velocity) / dt;

    // update
    lidar_state.time = current_lidar_time;
    lidar_state.pose = optimized_pose;
    lidar_state.velocity = local_velocity.head<3>();
    lidar_state.angular_velocity = local_velocity.tail<3>();
    lidar_state.linear_acceleration = local_linear_acceleration;

    _imu_local_rotation = optimized_pose.so3(); // correct only the drift in imu integration
    _imu_local_rotation_time = current_lidar_time;
  }

  // reset imu averages
  _interval_imu_count = 0;
  _interval_angular_velocity_sum.setZero();
  _interval_imu_acceleration_sum.setZero();
  _interval_body_acceleration_sum.setZero();
  _interval_imu_accel_mag_mean = 0;
  _interval_welford_sum_of_squares = 0;

  map.Update(frame_downsample, lidar_state.pose);
  _poses_with_timestamps.emplace_back(lidar_state.time, lidar_state.pose);
  return preprocessed_scan;
}

Vector3dVector LIO::register_scan(const Sophus::SE3d& extrinsic_lidar2base,
                                  const Vector3dVector& scan,
                                  const TimestampVector& timestamps) {
  if (extrinsic_lidar2base.log().norm() < EPSILON) {
    return register_scan(scan, timestamps);
  }
  Vector3dVector transformed_scan = scan;
  transform_points(extrinsic_lidar2base, transformed_scan);
  Vector3dVector frame = register_scan(transformed_scan, timestamps);
  transform_points(extrinsic_lidar2base.inverse(), frame);
  return frame;
}

// ============================ logs ===============================
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LIO::Config,
                                   deskew,
                                   max_iterations,
                                   voxel_size,
                                   max_points_per_voxel,
                                   max_range,
                                   min_range,
                                   convergence_criterion,
                                   max_correspondance_distance,
                                   max_num_threads,
                                   initialization_phase,
                                   max_expected_jerk,
                                   double_downsample,
                                   min_beta)

void LIO::dump_results_to_disk(const std::filesystem::path& results_dir, const std::string& run_name) const {
  try {
    std::filesystem::create_directories(results_dir); // no error if exists
    int index = 0;
    std::filesystem::path output_dir = results_dir / (run_name + "_" + std::to_string(index));
    while (std::filesystem::exists(output_dir)) {
      ++index;
      output_dir = results_dir / (run_name + "_" + std::to_string(index));
    }
    std::filesystem::create_directory(output_dir);
    const std::filesystem::path output_file = output_dir / (run_name + "_tum_" + std::to_string(index) + ".txt");
    // dump poses
    if (std::ofstream file(output_file); file.is_open()) {
      for (const auto& [timestamp, pose] : _poses_with_timestamps) {
        const Eigen::Vector3d& translation = pose.translation();
        const Eigen::Quaterniond& quaternion = pose.so3().unit_quaternion();
        file << std::fixed << std::setprecision(6) << timestamp.count() << " " << translation.x() << " "
             << translation.y() << " " << translation.z() << " " << quaternion.x() << " " << quaternion.y() << " "
             << quaternion.z() << " " << quaternion.w() << "\n";
      }
      std::cout << "Poses written to " << std::filesystem::absolute(output_file) << "\n";
    }
    // dump config
    const nlohmann::json json_config = {{"config", config}};
    const std::filesystem::path config_file = output_dir / "config.json";
    if (std::ofstream file(config_file); file.is_open()) {
      file << json_config.dump(4);
      std::cout << "Configuration written to " << config_file << "\n";
    }
  } catch (const std::filesystem::filesystem_error& ex) {
    std::cout << "Cannot write files to disk, encountered filesystem error: " << ex.what() << "\n";
  }
}
} // namespace rko_lio
