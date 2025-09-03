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

#include "point_cloud_read.hpp"
#include "time.hpp"
// ros
#include <algorithm>
// stl
#include <cstddef>
#include <iostream>
#include <stdexcept>

namespace {
using rko_lio_ros_utils::Secondsd;
using rko_lio_ros_utils::TimestampVector;

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

PointField get_timestamp_field(const PointCloud2::ConstSharedPtr& msg) {
  PointField timestamp_field;
  for (const auto& field : msg->fields) {
    if ((field.name == "t" || field.name == "timestamp" || field.name == "time" || field.name == "stamps")) {
      timestamp_field = field;
    }
  }
  if (timestamp_field.count != 0U) {
    return timestamp_field;
  }
  throw std::logic_error("Point cloud needs timestamps for deskewing");
}

inline TimestampVector extract_timestamps_from_msg(const PointCloud2::ConstSharedPtr msg,
                                                   const PointField& timestamp_field) {
  auto number_of_digits_integer_part = [](const auto& stamp) {
    const auto stampd = static_cast<double>(stamp);
    const auto digits_in_integer_part = static_cast<uint64_t>(std::round(stampd));
    return digits_in_integer_part > 0 ? std::floor(std::log10(digits_in_integer_part) + 1) : 1;
  };
  auto extract_timestamps = [&](auto&& stamp_iterator) -> TimestampVector {
    const size_t n_points = static_cast<size_t>(msg->height) * msg->width;
    TimestampVector timestamps;
    timestamps.reserve(n_points);
    bool is_relative = std::invoke(
        [&](auto iterator) {
          for (size_t i = 0; i < n_points; ++i, ++iterator) {
            // at least one number has to be close to 0
            if (std::abs(static_cast<double>(*iterator)) < 1e-8) {
              return true;
            }
          }
          return false;
        },
        stamp_iterator);
    bool is_relative_and_any_nanoseconds = is_relative && std::invoke(
                                                              [&](auto iterator) {
                                                                for (size_t i = 0; i < n_points; ++i, ++iterator) {
                                                                  // if in nanoseconds, and lidar is at say 10Hz, then
                                                                  // max stamp is around 0.09 sec. in nanoseconds thats
                                                                  // 90000000.0 which is 8 digits
                                                                  if (number_of_digits_integer_part(*iterator) > 7) {
                                                                    return true;
                                                                  }
                                                                }
                                                                return false;
                                                              },
                                                              stamp_iterator);
    for (size_t i = 0; i < n_points; ++i, ++stamp_iterator) {
      auto stampd = static_cast<double>(*stamp_iterator);
      // std::cout << std::setprecision(18);
      // std::cout << stampd << "\n";
      if (is_relative_and_any_nanoseconds) {
        stampd *= 1e-9;
      } else {
        // if the number of digits is greater than 10 (which is the maximum number of digits
        // that can be represented with a 32 bits integer), the stamp is in nanoseconds instead
        // of seconds, perform conversion
        if (number_of_digits_integer_part(stampd) > 10) {
          stampd *= 1e-9;
        }
      }
      timestamps.emplace_back(stampd);
    }
    // std::cout << "is_relative: " << is_relative
    //           << ", is_relative_and_any_nanoseconds: " << is_relative_and_any_nanoseconds << "\n";
    // throw std::runtime_error("die");
    return timestamps;
  };

  // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
  using sensor_msgs::PointCloud2ConstIterator;
  if (timestamp_field.datatype == PointField::UINT32) {
    return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == PointField::FLOAT32) {
    return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == PointField::FLOAT64) {
    return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
  }
  // timestamp type not supported, please open an issue :)
  throw std::invalid_argument("timestamp field type not supported");
}

// in seconds
TimestampVector get_timestamps(const PointCloud2::ConstSharedPtr& msg) {
  const auto& timestamp_field = get_timestamp_field(msg);
  TimestampVector timestamps = extract_timestamps_from_msg(msg, timestamp_field);
  return timestamps;
}

} // namespace

namespace rko_lio_ros_utils {
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);

std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const PointCloud2::ConstSharedPtr& msg,
                                                                   const bool force_absolute) {
  TimestampVector timestamps = get_timestamps(msg);
  const auto& [min_it, max_it] = std::ranges::minmax_element(timestamps);
  const Secondsd scan_duration = std::chrono::abs(*max_it - *min_it);
  const Secondsd header_stamp = ros_time_to_seconds(msg->header.stamp);
  Secondsd begin_stamp;
  Secondsd end_stamp;
  if (!force_absolute && *min_it >= Secondsd(0) && std::chrono::abs(*min_it) < EPSILON_TIME) {
    // Case 1: Relative to scan start
    // assume scan header is stamped with the start. if its actually the end, well
    begin_stamp = header_stamp;
    end_stamp = begin_stamp + scan_duration;
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
  } else if (!force_absolute && *max_it <= Secondsd(0) && std::chrono::abs(*max_it) < EPSILON_TIME) {
    // Case 2: Relative to scan end
    // assume scan header is stamped with the end
    end_stamp = header_stamp;
    begin_stamp = end_stamp - scan_duration;
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
  } else if (force_absolute || std::chrono::abs(header_stamp - *min_it) < std::chrono::microseconds(100) ||
             std::chrono::abs(header_stamp - *max_it) <
                 std::chrono::microseconds(100)) { // because i saw 0.013ms in hesai
    // Case 3: absolute timestamps
    begin_stamp = *min_it;
    end_stamp = *max_it;
  } else {
    std::cout << std::setprecision(18);
    std::cout << "point min_time: " << (*min_it).count() << "\n";
    std::cout << "point max_time: " << (*max_it).count() << "\n";
    std::cout << "header_sec: " << header_stamp.count() << "\n";
    std::cout << "header_sec - min_time: " << (header_stamp - *min_it).count() << "\n";
    std::cout << "header_sec - max_time: " << (header_stamp - *max_it).count() << "\n";
    throw std::logic_error("Can not handle timestamp conversion. Some unique case encountered.");
  }
  return {begin_stamp, end_stamp, timestamps};
}

Vector3dVector point_cloud2_to_eigen(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  const size_t point_count = static_cast<size_t>(msg->height) * msg->width;
  Vector3dVector points;
  points.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
  for (size_t i = 0; i < point_count; ++i, ++msg_x, ++msg_y, ++msg_z) {
    points.emplace_back(*msg_x, *msg_y, *msg_z);
  }
  return points;
}
} // namespace rko_lio_ros_utils
