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
set(PROJECT_PREFIX "RKO_LIO")
string(TOUPPER ${PROJECT_PREFIX} PROJECT_PREFIX_UP)

option(${PROJECT_PREFIX_UP}_ENABLE_FETCHCONTENT "Enable FetchContent for all dependencies" ON)

# find dependencies in the system or fetchcontent them
function(
  find_dep
  PACKAGE_NAME
  TARGET_NAME
  FETCH_CMAKE_FILE)
  # Optional version argument
  set(VERSION "")
  if(ARGC GREATER 3)
    list(
      GET
      ARGV
      3
      VERSION)
  endif()

  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "${PROJECT_PREFIX_UP}_USE_SYSTEM_${PACKAGE_NAME_UP}")

  # Try system package if requested
  if(${${USE_FROM_SYSTEM_OPTION}})
    if(VERSION)
      message(STATUS "Looking for system package: ${PACKAGE_NAME} (>= ${VERSION})")
      find_package(${PACKAGE_NAME} ${VERSION} QUIET)
    else()
      message(STATUS "Looking for system package: ${PACKAGE_NAME}")
      find_package(${PACKAGE_NAME} QUIET)
    endif()

    if(TARGET ${TARGET_NAME})
      message(STATUS "Found ${PACKAGE_NAME} ${VERSION} (target: ${TARGET_NAME}) - System Package")
      return()
    else()
      message(STATUS "System package ${PACKAGE_NAME}(target: ${TARGET_NAME}) not found.")
    endif()
  endif()

  # Fallback to FetchContent
  message(STATUS "Fetching ${PACKAGE_NAME} (target: ${TARGET_NAME}) using FetchContent (${FETCH_CMAKE_FILE})")
  include(${FETCH_CMAKE_FILE})

  if(TARGET ${TARGET_NAME})
    message(STATUS "Found ${TARGET_NAME} - FetchContent (${FETCH_CMAKE_FILE})")
  else()
    message(WARNING "Failed to fetch target: ${TARGET_NAME} for package: ${PACKAGE_NAME}")
  endif()
endfunction()

include(FetchContent)

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_EIGEN3 "Use system Eigen" ON)
find_dep(
  "Eigen3"
  "Eigen3::Eigen"
  "${CMAKE_CURRENT_LIST_DIR}/dependencies/eigen/eigen.cmake"
  3.4)

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_SOPHUS "Use system Sophus" ON)
find_dep("Sophus" "Sophus::Sophus" "${CMAKE_CURRENT_LIST_DIR}/dependencies/sophus/sophus.cmake")

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_JSON "Use system nlohmann Json" ON)
find_dep("nlohmann_json" "nlohmann_json::nlohmann_json"
         "${CMAKE_CURRENT_LIST_DIR}/dependencies/json/nlohmann_json.cmake")

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_PB_UTILS "Use system pb_utils" OFF)
find_dep("pb_utils" "pb_utils" "${CMAKE_CURRENT_LIST_DIR}/dependencies/pb_utils/pb_utils.cmake")

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_BONXAI "Use system Bonxai" OFF)
find_dep("bonxai" "bonxai_core" "${CMAKE_CURRENT_LIST_DIR}/dependencies/bonxai/bonxai.cmake")

option(${PROJECT_PREFIX_UP}_USE_SYSTEM_TBB "Use system TBB" ON)
find_dep("TBB" "TBB::tbb" "${CMAKE_CURRENT_LIST_DIR}/dependencies/tbb/tbb.cmake")
