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
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
using rko_lio_ros_utils::Secondsd;
using rko_lio_ros_utils::TimestampVector;

sensor_msgs::PointField get_timestamp_field(const sensor_msgs::PointCloud2ConstPtr& msg) {
  sensor_msgs::PointField timestamp_field;
  for (const auto& field : msg->fields) {
    if ((field.name == "t" || field.name == "timestamp" || field.name == "time" || field.name == "stamps")) {
      timestamp_field = field;
    }
  }
  if (timestamp_field.count != 0U) {
    return timestamp_field;
  }
  throw std::runtime_error("Point cloud needs timestamps for deskewing");
}

inline TimestampVector extract_timestamps_from_msg(const sensor_msgs::PointCloud2ConstPtr& msg,
                                                   const sensor_msgs::PointField& timestamp_field) {
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
    // << ", is_relative_and_any_nanoseconds: " << is_relative_and_any_nanoseconds << "\n";
    return timestamps;
  };

  // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
  using sensor_msgs::PointCloud2ConstIterator;
  if (timestamp_field.datatype == sensor_msgs::PointField::UINT32) {
    return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == sensor_msgs::PointField::FLOAT32) {
    return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
  } else if (timestamp_field.datatype == sensor_msgs::PointField::FLOAT64) {
    return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
  }
  // timestamp type not supported, please open an issue :)
  throw std::runtime_error("timestamp field type not supported");
}

TimestampVector get_timestamps(const sensor_msgs::PointCloud2ConstPtr& msg) {
  const auto& timestamp_field = get_timestamp_field(msg);
  TimestampVector timestamps = extract_timestamps_from_msg(msg, timestamp_field);
  return timestamps;
}
} // namespace

namespace rko_lio_ros_utils {
constexpr auto EPSILON_TIME = std::chrono::nanoseconds(10);

std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const sensor_msgs::PointCloud2ConstPtr& msg,
                                                                   const bool force_absolute) {
  TimestampVector timestamps = get_timestamps(msg);
  const auto& [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
  const Secondsd scan_duration = std::chrono::abs(*max_it - *min_it);
  const Secondsd header_stamp = ros_time_to_seconds(msg->header.stamp);
  Secondsd begin_stamp;
  Secondsd end_stamp;
  if (force_absolute) {
    begin_stamp = *min_it;
    end_stamp = *max_it;
    return {begin_stamp, end_stamp, timestamps};
  }
  if (*min_it >= Secondsd(0) && std::chrono::abs(*min_it) < EPSILON_TIME) {
    // Case 1: timestamps are relative to scan start
    // We have to assume header stamp is the scan start time, otherwise it's impossible to recover absolute time
    // i.e. if header stamp is end stamp, its GG
    begin_stamp = header_stamp;
    end_stamp = begin_stamp + scan_duration;
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
    return {begin_stamp, end_stamp, timestamps};
  }
  if (*max_it <= Secondsd(0) && std::chrono::abs(*max_it) < std::chrono::milliseconds(4)) {
    // Case 2: timestamps are relative to scan end
    // We have to once again assume header stamp is scan end time. see above
    // the 4ms is because i saw this case happen on a VLP-16 (Leg-KILO, see below)
    // i literally kept increasing it by 1ms every time it failed to get to 4ms
    end_stamp = header_stamp;
    begin_stamp = end_stamp - scan_duration;
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
    return {begin_stamp, end_stamp, timestamps};
  }
  if (std::chrono::abs(header_stamp - *min_it) < std::chrono::milliseconds(1) ||
      std::chrono::abs(header_stamp - *max_it) < std::chrono::milliseconds(1)) {
    // Case 3: absolute timestamps
    // min or max is within 1ms to stamp time. if the timestamps are not absolute i don't know what they are
    begin_stamp = *min_it;
    end_stamp = *max_it;
    return {begin_stamp, end_stamp, timestamps};
  }
  // now we enter the realm of weird timestamp-ness
  if (*min_it < Secondsd(0) && *max_it > Secondsd(0) && *max_it < std::chrono::milliseconds(10)) {
    // Case VLP-16 on Leg-KILO dataset: timestamps are relative to the end, but somehow there are a few points with
    // timestamps ahead of the header stamp.
    begin_stamp = header_stamp + *min_it;
    end_stamp = header_stamp + *max_it;
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps.begin(),
                   [&header_stamp](const Secondsd& ts) { return ts + header_stamp; });
    return {begin_stamp, end_stamp, timestamps};
  }
  std::cout << std::setprecision(18);
  std::cout << "point min_time: " << (*min_it).count() << "\n";
  std::cout << "point max_time: " << (*max_it).count() << "\n";
  std::cout << "header_sec: " << header_stamp.count() << "\n";
  std::cout << "header_sec - min_time: " << (header_stamp - *min_it).count() << "\n";
  std::cout << "header_sec - max_time: " << (header_stamp - *max_it).count() << "\n";
  throw std::runtime_error(
      "Can not handle timestamp conversion. Some unique case encountered. Please report an issue with this data.");
}

Vector3dVector point_cloud2_to_eigen(const sensor_msgs::PointCloud2ConstPtr& msg) {
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
