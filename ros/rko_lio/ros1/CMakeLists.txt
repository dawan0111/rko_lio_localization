# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi
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

add_library(rko_lio_node node.cpp)
target_include_directories(rko_lio_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(
  rko_lio_node
  PUBLIC rko_lio_cpp
         rko_lio_ros_utils
         ${catkin_LIBRARIES}
         Eigen3::Eigen
         Sophus::Sophus
         pb_utils)

add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE rko_lio_node)
install(TARGETS offline_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(online_node online_node.cpp)
target_link_libraries(online_node PRIVATE rko_lio_node)
install(TARGETS online_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
