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

// NOTE: This is a convenience file for now, because we have ros_utils as a separate package.
// for the final release, some how use the same file for timestamps everywhere because i just want one location where we
// handle this logic. this is actually the most important after all. and then pybind that so we can use it directly
// in the rosbag dataloader.
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace timestamp_util {

using Secondsd = std::chrono::duration<double>;
using TimestampVector = std::vector<Secondsd>;

/**
 * The timestamps are either in seconds or in nanoseconds, we handle no other case for now
 */
std::tuple<Secondsd, Secondsd, TimestampVector> timestamps_in_sec_from_raw(const std::vector<double>& raw_timestamps);

// Process raw timestamps to determine absolute timestamps in seconds.
std::tuple<Secondsd, Secondsd, TimestampVector> process_timestamps(const std::vector<double>& raw_timestamps,
                                                                   const Secondsd& header_stamp,
                                                                   const bool force_absolute = false);
} // namespace timestamp_util
