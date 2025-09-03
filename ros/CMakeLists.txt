# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.28)
project(rko_lio VERSION 0.0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

include(StandardProjectOptions)
include(dependencies)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_CURRENT_BINARY_DIR}/rko_lio_cpp)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  # ROS dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rclcpp_components REQUIRED)

  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rko_lio_ros_utils/ros2/rko_lio_ros_utils)
  add_subdirectory(rko_lio/ros2)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_package()
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(
    catkin REQUIRED
    COMPONENTS roscpp
               std_msgs
               tf2
               tf2_ros
               sensor_msgs
               nav_msgs
               geometry_msgs
               rosbag)
  catkin_package()

  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rko_lio_ros_utils/ros1/rko_lio_ros_utils)
  add_subdirectory(rko_lio/ros1)
  install(DIRECTORY config/ launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is improper.")
endif()
