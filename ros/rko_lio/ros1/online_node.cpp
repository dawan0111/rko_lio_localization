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
#include <Eigen/src/Geometry/Quaternion.h>
#include <pb_utils/profiler.hpp>
#include <pb_utils/timer.hpp>
// stl
#include <cmath>
#include <string>
// ros
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rko_lio_ros {
class OnlineNode : public Node {
public:
  ros::Subscriber imu_sub;
  ros::Subscriber lidar_sub;

  OnlineNode() {
    // subscribe
    imu_sub =
        nh.subscribe<const sensor_msgs::Imu::ConstPtr&, OnlineNode>(imu_topic, 100, &OnlineNode::imu_callback, this);
    lidar_sub = nh.subscribe<const sensor_msgs::PointCloud2::ConstPtr&, OnlineNode>(lidar_topic, 10,
                                                                                    &OnlineNode::lidar_callback, this);
  }
};
} // namespace rko_lio_ros

int main(int argc, char** argv) {
  pb_utils::Timer timer("ROS Online Node");
  ros::init(argc, argv, "rko_lio_online_node");
  rko_lio_ros::OnlineNode online_node;
  ros::spin();
  online_node.lio->dump_results_to_disk(online_node.results_dir, online_node.run_name);
  return 0;
}
