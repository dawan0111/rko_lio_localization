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

cmake_minimum_required(VERSION 3.28)
project(rko_lio_python VERSION 0.0.1 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../cpp ${CMAKE_CURRENT_BINARY_DIR}/rko_lio_cpp_build)

set(PYBIND11_NEWPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(
  rko_lio_pybind
  MODULE
  rko_lio_pybind.cpp
  read_timestamps.cpp)
target_link_libraries(rko_lio_pybind PRIVATE rko_lio_cpp)
# put it alonside the python package's __init__.py
install(TARGETS rko_lio_pybind DESTINATION rko_lio)

pybind11_add_module(helipr_file_reader_pybind MODULE helipr_file_reader_pybind.cpp)
target_compile_features(helipr_file_reader_pybind PUBLIC cxx_std_20)
# put it alonside the python package's dataloaders/__init__.py
install(TARGETS helipr_file_reader_pybind DESTINATION rko_lio/dataloaders)
