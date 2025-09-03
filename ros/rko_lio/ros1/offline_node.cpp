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

#include "node.hpp"
// other
#include <pb_utils/profiler.hpp>
#include <pb_utils/timer.hpp>
// stl
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
// ros
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <std_msgs/Float32.h>

namespace rko_lio_ros {
class OfflineNode : public Node {
public:
  std::unique_ptr<rko_lio_ros_utils::BufferableBag> bag;

  ros::Publisher raw_imu_publisher;
  ros::Publisher raw_lidar_publisher;
  ros::Publisher bag_progress_publisher;
  float total_bag_msgs = 0;
  float processed_bag_msgs = 0;

  OfflineNode() {
    max_lidar_buffer_size = 100;
    // bag reading
    double skip_to_time_sec = 0.0;
    nh.param("skip_to_time", skip_to_time_sec, skip_to_time_sec);
    const Secondsd skip_to_time(skip_to_time_sec);
    std::string bag_filename;
    nh.param("bag_filename", bag_filename, ""s);
    if (bag_filename.empty()) {
      throw std::runtime_error("Bag filename cannot be empty.");
    }
    bag = std::make_unique<rko_lio_ros_utils::BufferableBag>(
        bag_filename, std::make_shared<rko_lio_ros_utils::BufferableBag::TFBridge>(),
        std::vector{imu_topic, lidar_topic, "/tf"s, "/tf_static"s}, skip_to_time, std::chrono::seconds(1));

    total_bag_msgs = static_cast<float>(bag->message_count);
    bag_progress_publisher = nh.advertise<std_msgs::Float32>("/rko_lio/bag_progress", 10);
    if (debug) {
      raw_imu_publisher = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
      raw_lidar_publisher = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic, 1);
    }

    ROS_INFO_STREAM((debug ? " Debug: Publishing compensated IMU acceleration to /rko_lio/body_accel. Debug: "
                             "Re-publishing IMU data to " +
                                 imu_topic + " and lidar data to " + lidar_topic + "."
                           : ""));
  }

  void run() {
    while (ros::ok() && !bag->finished()) {
      {
        if (lidar_buffer.size() >= 0.9 * max_lidar_buffer_size) {
          // Throttle reading when buffer is full
          ROS_WARN_STREAM_ONCE("lidar buffer size: " << lidar_buffer.size()
                                                     << ", max_lidar_buffer_size: " << max_lidar_buffer_size
                                                     << ", throttling the bag reading thread\n");
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
      }
      const rosbag::MessageInstance msg_instance = bag->pop_next_message();
      const std::string& topic_name = msg_instance.getTopic();
      processed_bag_msgs++;
      if (total_bag_msgs > 0) {
        std_msgs::Float32 progress_msg;
        progress_msg.data = 100.0F * processed_bag_msgs / total_bag_msgs;
        bag_progress_publisher.publish(progress_msg);
      }
      if (topic_name == imu_topic) {
        const auto& imu_msg = msg_instance.instantiate<sensor_msgs::Imu>();
        if (!imu_msg) {
          continue;
        }
        imu_callback(imu_msg);
        if (!debug) {
          continue;
        }
        if (imu_frame != "" && imu_msg->header.frame_id != imu_frame) {
          // because some people are retarded and can't do a tf tree properly
          sensor_msgs::Imu proper_imu_msg = *imu_msg;
          proper_imu_msg.header.frame_id = imu_frame;
          raw_imu_publisher.publish(proper_imu_msg);
        } else {
          raw_imu_publisher.publish(*imu_msg);
        }
      } else if (topic_name == lidar_topic) {
        const auto& lidar_msg = msg_instance.instantiate<sensor_msgs::PointCloud2>();
        if (!lidar_msg) {
          continue;
        }
        lidar_callback(lidar_msg);
        if (!debug) {
          continue;
        }
        if (lidar_frame != "" && lidar_msg->header.frame_id != lidar_frame) {
          sensor_msgs::PointCloud2 proper_lidar_msg = *lidar_msg;
          proper_lidar_msg.header.frame_id = lidar_frame;
          raw_lidar_publisher.publish(proper_lidar_msg);
        } else {
          raw_lidar_publisher.publish(*lidar_msg);
        }
      }
    }
    while (atomic_node_running) {
      // even if the bag finishes, we need to wait on the registration thread (buffers) to finish (empty)
      std::lock_guard lock(buffer_mutex);
      if (imu_buffer.empty() || lidar_buffer.empty()) {
        break;
      }
    }
  }
};
} // namespace rko_lio_ros

int main(int argc, char** argv) {
  pb_utils::Timer timer("ROS Offline Node");
  ros::init(argc, argv, "rko_lio_offline_node");
  rko_lio_ros::OfflineNode offline_node;
  try {
    offline_node.run();
  } catch (const std::runtime_error& error) {
    ROS_WARN("Encountered runtime_error. Still attempting to dump trajectory to disk. The error was: %s", error.what());
  }
  offline_node.lio->dump_results_to_disk(offline_node.results_dir, offline_node.run_name);
  ros::shutdown();
  return 0;
}
