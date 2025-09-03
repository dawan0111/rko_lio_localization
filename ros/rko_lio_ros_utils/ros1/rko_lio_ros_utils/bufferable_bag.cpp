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
#include "bufferable_bag.hpp"
#include "rosbag/message_instance.h"
// ROS
#include <filesystem>
#include <tf2_msgs/TFMessage.h>
// stl
#include <algorithm>

namespace rko_lio_ros_utils {
// TFBridge----------------------------------------------------------------------------------------
BufferableBag::TFBridge::TFBridge() {
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
  tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
}

void BufferableBag::TFBridge::process_TFMessage(const rosbag::MessageInstance& msg) const {
  const auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
  if (tf_message) {
    if (msg.getTopic() == "/tf_static") {
      tf_static_broadcaster->sendTransform(tf_message->transforms);
    } else {
      tf_broadcaster->sendTransform(tf_message->transforms);
    }
  }
}

// BufferableBag-----------------------------------------------------------------------------------
BufferableBag::BufferableBag(const std::string& bag_directory,
                             const std::shared_ptr<TFBridge>& tf_bridge,
                             const std::vector<std::string>& topics,
                             const Secondsd seek,
                             const Secondsd buffer_size)
    : tf_bridge_(tf_bridge), buffer_size_(buffer_size), topics_(topics) {
  std::vector<std::string> bag_files;
  if (!std::filesystem::is_directory(bag_directory)) {
    throw std::runtime_error(bag_directory + " is not a directory");
  }
  for (const auto& entry : std::filesystem::directory_iterator(bag_directory)) {
    if (entry.path().extension() == ".bag") {
      bag_files.push_back(entry.path().string());
    }
  }
  if (bag_files.empty()) {
    throw std::runtime_error("No .bag files found in directory: " + bag_directory);
  }
  std::sort(bag_files.begin(), bag_files.end());

  for (const auto& file : bag_files) {
    auto bag = std::make_unique<rosbag::Bag>();
    bag->open(file, rosbag::bagmode::Read);
    bags_.push_back(std::move(bag));
    ROS_INFO_STREAM("Added bag: " << file);
  }

  // need an extra view to get the start time. TODO: see if this can be improved
  const ros::Time start_time = rosbag::View(*bags_.front()).getBeginTime() + ros::Duration(seek.count());
  for (const auto& bag : bags_) {
    view_.addQuery(*bag, rosbag::TopicQuery(topics_), start_time);
  }
  view_it_ = view_.begin();

  message_count = std::distance(view_.begin(), view_.end());
  read_progress_ = std::make_unique<pb_utils::bar>(message_count);

  std::cout << "Bag reader initialized with total message count: " << message_count << " from " << bags_.size()
            << " bags\n";
  buffer_messages();
}

void BufferableBag::buffer_messages() {
  auto buffer_is_filled = [&]() -> bool {
    if (buffer_.empty()) {
      return false;
    }
    const ros::Time first_stamp = buffer_.front().getTime();
    const ros::Time last_stamp = buffer_.back().getTime();
    return (last_stamp - first_stamp) > ros::Duration(buffer_size_.count());
  };

  // Advance reading one message until the buffer is filled or we finish the bagfile
  while (!buffer_is_filled() && view_it_ != view_.end()) {
    const auto msg = *view_it_;
    if (msg.getTopic() == "/tf" || msg.getTopic() == "/tf_static") {
      tf_bridge_->process_TFMessage(msg);
    } else if (std::find(topics_.cbegin(), topics_.cend(), msg.getTopic()) != topics_.end()) {
      buffer_.push(msg);
    }
    ++view_it_;
    read_progress_->tick();
  }
}

rosbag::MessageInstance BufferableBag::pop_next_message() {
  const auto msg = buffer_.front();
  buffer_.pop();
  buffer_messages();
  return msg;
}

bool BufferableBag::finished() { return (view_it_ == view_.end()) && buffer_.empty(); };

BufferableBag::~BufferableBag() {
  for (auto& bag : bags_) {
    if (bag->isOpen()) {
      bag->close();
    }
  }
}
} // namespace rko_lio_ros_utils
