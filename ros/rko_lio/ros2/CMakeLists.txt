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

add_library(lio_node node.cpp)
ament_target_dependencies(
  lio_node
  PUBLIC
  rclcpp
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs)
target_link_libraries(
  lio_node
  PUBLIC rko_lio_ros_utils
         rko_lio_cpp
         Sophus::Sophus
         Eigen3::Eigen)

add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE lio_node pb_utils)
install(TARGETS offline_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

add_library(online_node_component SHARED online_node.cpp)
ament_target_dependencies(online_node_component PUBLIC rclcpp_components)
target_link_libraries(online_node_component PRIVATE lio_node pb_utils)
# the component is different from the executable node, the install for both needs to be seperate
# the following creates an executable "online_node", check the code for the macro, while registering the component to the ament index
# default will use a single threaded executor
rclcpp_components_register_node(
  online_node_component
  PLUGIN
  "rko_lio_ros::OnlineNode"
  EXECUTABLE
  online_node)
install(TARGETS online_node_component ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

add_library(velocity_filter_component SHARED velocity_filter.cpp)
ament_target_dependencies(
  velocity_filter_component
  PUBLIC
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs)
target_link_libraries(velocity_filter_component PUBLIC rko_lio_ros_utils Sophus::Sophus Eigen3::Eigen)
rclcpp_components_register_node(
  velocity_filter_component
  PLUGIN
  "rko_lio_ros::VelocityFilter"
  EXECUTABLE
  velocity_filter)
install(TARGETS velocity_filter_component ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(attitude_filter attitude_filter.cpp)
ament_target_dependencies(
  attitude_filter
  PUBLIC
  rclcpp
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs)
target_link_libraries(attitude_filter PUBLIC rko_lio_ros_utils Sophus::Sophus Eigen3::Eigen)
install(TARGETS attitude_filter ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
