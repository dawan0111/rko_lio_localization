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

#include "read_timestamps.hpp"
#include "rko_lio/lio.hpp"
#include "stl_vector_eigen.hpp"
#include <cmath>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace {
std::vector<Eigen::Vector3d> transform_points(const Sophus::SE3d& T, const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector3d> transformed_points = points;
  std::transform(transformed_points.begin(), transformed_points.end(), transformed_points.begin(),
                 [&](const auto& point) { return T * point; });
  return transformed_points;
}

} // namespace
namespace py = pybind11;
using namespace rko_lio;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

PYBIND11_MODULE(rko_lio_pybind, m) {
  auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
      m, "_Vector3dVector", "std::vector<Eigen::Vector3d>", py::py_array_to_vectors_double<Eigen::Vector3d>);
  py::bind_vector<std::vector<double>>(m, "_VectorDouble");

  py::class_<LIO::Config>(m, "_Config")
      .def(py::init<>())
      .def_readwrite("deskew", &LIO::Config::deskew)
      .def_readwrite("max_iterations", &LIO::Config::max_iterations)
      .def_readwrite("voxel_size", &LIO::Config::voxel_size)
      .def_readwrite("max_points_per_voxel", &LIO::Config::max_points_per_voxel)
      .def_readwrite("max_range", &LIO::Config::max_range)
      .def_readwrite("min_range", &LIO::Config::min_range)
      .def_readwrite("convergence_criterion", &LIO::Config::convergence_criterion)
      .def_readwrite("max_correspondance_distance", &LIO::Config::max_correspondance_distance)
      .def_readwrite("max_num_threads", &LIO::Config::max_num_threads)
      .def_readwrite("initialization_phase", &LIO::Config::initialization_phase)
      .def_readwrite("max_expected_jerk", &LIO::Config::max_expected_jerk)
      .def_readwrite("double_downsample", &LIO::Config::double_downsample)
      .def_readwrite("min_beta", &LIO::Config::min_beta);

  py::class_<LIO>(m, "_LIO")
      .def(py::init<const LIO::Config&>(), py::arg("config"))
      .def(
          "update_imu_motion",
          [](LIO& self, const Eigen::Vector3d& accel, const Eigen::Matrix3d& accel_cov, const Eigen::Vector3d& gyro,
             const Eigen::Matrix3d& gyro_cov, double time) {
            self.update_imu_motion(ImuControl{
                .time = Secondsd(time),
                .acceleration = accel,
                .acceleration_covariance = accel_cov,
                .angular_velocity = gyro,
                .angular_velocity_covariance = gyro_cov,
            });
          },
          py::arg("acceleration"), py::arg("acceleration_cov"), py::arg("angular_velocity"),
          py::arg("angular_velocity_cov"), py::arg("time"))
      .def(
          "update_imu_motion",
          [](LIO& self, const Eigen::Matrix4d& extrinsic_imu2base, const Eigen::Vector3d& accel,
             const Eigen::Matrix3d& accel_cov, const Eigen::Vector3d& gyro, const Eigen::Matrix3d& gyro_cov,
             double time) {
            self.update_imu_motion(Sophus::SE3d(extrinsic_imu2base), ImuControl{
                                                                         .time = Secondsd(time),
                                                                         .acceleration = accel,
                                                                         .acceleration_covariance = accel_cov,
                                                                         .angular_velocity = gyro,
                                                                         .angular_velocity_covariance = gyro_cov,
                                                                     });
          },
          py::arg("extrinsic_imu2base"), py::arg("acceleration"), py::arg("acceleration_cov"),
          py::arg("angular_velocity"), py::arg("angular_velocity_cov"), py::arg("time"))
      .def(
          "register_scan",
          [](LIO& self, const std::vector<Eigen::Vector3d>& scan, const std::vector<double>& timestamps) {
            std::vector<Secondsd> tsd;
            tsd.reserve(timestamps.size());
            std::transform(timestamps.cbegin(), timestamps.cend(), std::back_inserter(tsd),
                           [](const double t) { return Secondsd(t); });
            return self.register_scan(scan, tsd);
          },
          py::arg("scan"), py::arg("timestamps"))
      .def(
          "register_scan",
          [](LIO& self, const Eigen::Matrix4d& extrinsic_lidar2base, const std::vector<Eigen::Vector3d>& scan,
             const std::vector<double>& timestamps) {
            std::vector<Secondsd> tsd;
            tsd.reserve(timestamps.size());
            std::transform(timestamps.cbegin(), timestamps.cend(), std::back_inserter(tsd),
                           [](const double t) { return Secondsd(t); });
            return self.register_scan(Sophus::SE3d(extrinsic_lidar2base), scan, tsd);
          },
          py::arg("extrinsic_lidar2base"), py::arg("scan"), py::arg("timestamps"))
      .def("dump_results_to_disk",
           [](LIO& self, const std::string& results_dir, const std::string& run_name) {
             self.dump_results_to_disk(std::filesystem::path(results_dir), run_name);
           })
      .def("local_map_point_cloud", [](LIO& self) { return self.map.Pointcloud(); })
      .def("pose", [](LIO& self) { return self.lidar_state.pose.matrix(); });

  m.def(
      "_process_timestamps",
      [](const std::vector<double>& raw_ts, const double header_stamp, const bool force_absolute = false) {
        const auto& [begin, end, abs_ts] =
            timestamp_util::process_timestamps(raw_ts, timestamp_util::Secondsd(header_stamp), force_absolute);
        std::vector<double> abs_ts_double;
        abs_ts_double.reserve(abs_ts.size());
        for (const auto& t : abs_ts)
          abs_ts_double.push_back(t.count());
        return std::make_tuple(begin.count(), end.count(), std::move(abs_ts_double));
      },
      py::arg("raw_timestamps"), py::arg("header_stamp"), py::arg("force_absolute") = false);
}
