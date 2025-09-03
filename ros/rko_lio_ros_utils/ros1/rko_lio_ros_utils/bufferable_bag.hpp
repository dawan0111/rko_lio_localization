// modified from kinematic icp
// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include "pb_utils/bar.hpp"
// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// ROS
#include "rosbag/message_instance.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// stl
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace rko_lio_ros_utils {
using Secondsd = std::chrono::duration<double>;

class BufferableBag {
public:
  // Wrapper node to process the transforamtions present in the bagfile
  struct TFBridge {
    explicit TFBridge();
    void process_TFMessage(const rosbag::MessageInstance& msg) const;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
  };
  BufferableBag(const BufferableBag&) = delete;
  BufferableBag(BufferableBag&&) = delete;
  BufferableBag& operator=(const BufferableBag&) = delete;
  BufferableBag& operator=(BufferableBag&&) = delete;

  BufferableBag(const std::string& bag_directory,
                const std::shared_ptr<TFBridge>& tf_bridge,
                const std::vector<std::string>& topics,
                const Secondsd seek = Secondsd(0),
                const Secondsd buffer_size = std::chrono::seconds(1));

  void publish_tf_static(const std::vector<std::string>& bag_files);
  void buffer_messages();
  rosbag::MessageInstance pop_next_message();
  // finished cant be const because the end() iterator is not const
  bool finished();
  // RAII close bags
  ~BufferableBag();

  size_t message_count{0};

private:
  std::shared_ptr<TFBridge> tf_bridge_;
  std::vector<std::unique_ptr<rosbag::Bag>> bags_;
  std::queue<rosbag::MessageInstance> buffer_;
  Secondsd buffer_size_;
  std::vector<std::string> topics_;
  rosbag::View view_;
  rosbag::View::iterator view_it_;
  std::unique_ptr<pb_utils::bar> read_progress_;
};
} // namespace rko_lio_ros_utils
