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

"""
this file is a merge (and extension) of kiss_icp/tools/read_point_cloud.py and kiss_icp/datasets/rosbag.py.
the rosbag dataloader has been simplified and then extended for handling both imu and lidar.
plus all the logic i have for reading absolute timestamps from the ros wrapper side is ported over.
unfortunately, there is no way around double maintaining the logic (possibly triple for the ros1 wrapper as well).
i've elected to keep all rosbag related stuff in one single file, as a consequence of which this file is quite large.
it is what it is.
"""

__TIMESTAMP_ATTRIBUTE_NAMES__ = ["time", "timestamps", "timestamp", "t"]

# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This file is based on https://github.com/ros2/common_interfaces/blob/4bac182a0a582b5e6b784d9fa9f0dabc1aca4d35/sensor_msgs_py/sensor_msgs_py/point_cloud2.py
All rights reserved to the original authors: Tim Field and Florian Vahl.
"""

import sys
from typing import Iterable, List, Optional, Tuple

import numpy as np

try:
    from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as PointCloud2
    from rosbags.typesys.types import sensor_msgs__msg__PointField as PointField
except ImportError as e:
    raise ImportError(
        'rosbags library not installed, run "pip install -U rosbags"'
    ) from e


_DATATYPES = {}
_DATATYPES[PointField.INT8] = np.dtype(np.int8)
_DATATYPES[PointField.UINT8] = np.dtype(np.uint8)
_DATATYPES[PointField.INT16] = np.dtype(np.int16)
_DATATYPES[PointField.UINT16] = np.dtype(np.uint16)
_DATATYPES[PointField.INT32] = np.dtype(np.int32)
_DATATYPES[PointField.UINT32] = np.dtype(np.uint32)
_DATATYPES[PointField.FLOAT32] = np.dtype(np.float32)
_DATATYPES[PointField.FLOAT64] = np.dtype(np.float64)

DUMMY_FIELD_PREFIX = "unnamed_field"


def dtype_from_fields(
    fields: Iterable[PointField], point_step: Optional[int] = None
) -> np.dtype:
    """
    Convert a Iterable of sensor_msgs.msg.PointField messages to a np.dtype.
    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param point_step: Point step size in bytes. Calculated from the given fields by default.
                       (Type: optional of integer)
    :returns: NumPy datatype
    """
    # Create a lists containing the names, offsets and datatypes of all fields
    field_names = []
    field_offsets = []
    field_datatypes = []
    for i, field in enumerate(fields):
        # Datatype as numpy datatype
        datatype = _DATATYPES[field.datatype]
        # Name field
        if field.name == "":
            name = f"{DUMMY_FIELD_PREFIX}_{i}"
        else:
            name = field.name
        # Handle fields with count > 1 by creating subfields with a suffix consiting
        # of "_" followed by the subfield counter [0 -> (count - 1)]
        assert field.count > 0, "Can't process fields with count = 0."
        for a in range(field.count):
            # Add suffix if we have multiple subfields
            if field.count > 1:
                subfield_name = f"{name}_{a}"
            else:
                subfield_name = name
            assert (
                subfield_name not in field_names
            ), "Duplicate field names are not allowed!"
            field_names.append(subfield_name)
            # Create new offset that includes subfields
            field_offsets.append(field.offset + a * datatype.itemsize)
            field_datatypes.append(datatype.str)

    # Create dtype
    dtype_dict = {
        "names": field_names,
        "formats": field_datatypes,
        "offsets": field_offsets,
    }
    if point_step is not None:
        dtype_dict["itemsize"] = point_step
    return np.dtype(dtype_dict)


def read_points(
    cloud: PointCloud2,
    field_names: Optional[List[str]] = None,
    uvs: Optional[Iterable] = None,
    reshape_organized_cloud: bool = False,
) -> np.ndarray:
    """
    Read points from a sensor_msgs.PointCloud2 message.
    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :param reshape_organized_cloud: Returns the array as an 2D organized point cloud if set.
    :return: Structured NumPy array containing all points.
    """
    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width * cloud.height,),
        dtype=dtype_from_fields(cloud.fields, point_step=cloud.point_step),
        buffer=cloud.data,
    )

    # Keep only the requested fields
    if field_names is not None:
        assert all(
            field_name in points.dtype.names for field_name in field_names
        ), "Requests field is not in the fields of the PointCloud!"
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != "little") != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=True)

    # Select points indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested points
        points = points[uvs]

    # Cast into 2d array if cloud is 'organized'
    if reshape_organized_cloud and cloud.height > 1:
        points = points.reshape(cloud.width, cloud.height)

    return points


def read_point_cloud(msg: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract poitns and timestamps from a PointCloud2 message.

    :return: Tuple of [points, timestamps]
        points: array of x, y z points, shape: (N, 3)
        timestamps: array of per-pixel timestamps, shape: (N,)
    """
    field_names = ["x", "y", "z"]
    t_field = None
    for field in msg.fields:
        if field.name in __TIMESTAMP_ATTRIBUTE_NAMES__:
            t_field = field.name
            field_names.append(t_field)
            break

    points_structured = read_points(msg, field_names=field_names)
    points = np.column_stack(
        [points_structured["x"], points_structured["y"], points_structured["z"]]
    )

    # Remove nan if any
    # need to handle the change in timestamps too
    # points = points[~np.any(np.isnan(points), axis=1)]

    if t_field:
        timestamps = points_structured[t_field].astype(np.float64)
    else:
        timestamps = np.array([])
    return points.astype(np.float64), timestamps


# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
import sys
from pathlib import Path


class RosbagDataLoader:
    def __init__(self, data_path: Path, imu_topic: str | None, lidar_topic: str | None):
        try:
            from rosbags.highlevel import AnyReader
        except ModuleNotFoundError:
            print(
                'rosbags library not installed for using rosbag dataloader, please install with "pip install -U rosbags"'
            )
            sys.exit(1)

        assert (
            data_path.is_dir()
        ), "Pass a directory to data_path with ros1 or ros2 bag files"

        from .. import rko_lio_pybind

        self.rko_lio_pybind = rko_lio_pybind

        ros1_bagfiles = sorted(list(data_path.glob("*.bag")))
        bagfiles = ros1_bagfiles if ros1_bagfiles else [data_path]
        self.bag = AnyReader(bagfiles)
        self.bag.open()
        self.lidar_topic = self.check_topic(
            lidar_topic, expected_msgtype="sensor_msgs/msg/PointCloud2"
        )
        self.imu_topic = self.check_topic(
            imu_topic, expected_msgtype="sensor_msgs/msg/Imu"
        )
        self.connections = [
            x
            for x in self.bag.connections
            if (x.topic == self.imu_topic or x.topic == self.lidar_topic)
        ]
        self.msgs = self.bag.messages(connections=self.connections)

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return (
            self.bag.topics[self.imu_topic].msgcount
            + self.bag.topics[self.lidar_topic].msgcount
        )

    def __getitem__(self, idx):
        connection, bag_timestamp, rawdata = next(self.msgs)
        deserialized_data = self.bag.deserialize(rawdata, connection.msgtype)
        if connection.topic == self.imu_topic:
            return "imu", self.read_imu(deserialized_data)
        elif connection.topic == self.lidar_topic:
            return "lidar", self.read_point_cloud(deserialized_data)
        else:
            raise NotImplementedError("Shouldn't happen.")

    def read_imu(self, data):
        header_stamp = data.header.stamp
        timestamp = header_stamp.sec + (header_stamp.nanosec / 1e9)
        gyro = [
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        ]
        gyro_cov = data.angular_velocity_covariance.reshape(3, 3)
        accel = [
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ]
        accel_cov = data.linear_acceleration_covariance.reshape(3, 3)
        return timestamp, accel, accel_cov, gyro, gyro_cov

    def read_point_cloud(self, data):
        header_stamp = data.header.stamp
        header_stamp_sec = header_stamp.sec + (header_stamp.nanosec / 1e9)
        points, raw_timestamps = read_point_cloud(data)
        _, _, abs_timestamps = self.rko_lio_pybind._process_timestamps(
            self.rko_lio_pybind._VectorDouble(raw_timestamps),
            header_stamp_sec,
            force_absolute=False,
        )
        return points, np.asarray(abs_timestamps)

    def check_topic(self, topic: str | None, expected_msgtype: str) -> str:
        topics_of_type = [
            topic_name
            for topic_name, info in self.bag.topics.items()
            if info.msgtype == expected_msgtype
        ]

        def print_available_topics_and_exit():
            print(50 * "-")
            for t in topics_of_type:
                print(f"--{'imu' if expected_msgtype.endswith('Imu') else 'lidar'} {t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in topics_of_type:
            return topic
        if topic and topic not in topics_of_type:
            print(
                f'[ERROR] Dataset does not contain any msg with the topic name "{topic}". '
                f"Please select one of these for {expected_msgtype}:"
            )
            print_available_topics_and_exit()
        if len(topics_of_type) > 1:
            print(
                f"Multiple {expected_msgtype} topics available. Please select one with the appropriate flag."
            )
            print_available_topics_and_exit()
        if len(topics_of_type) == 0:
            print(f"[ERROR] Your dataset does not contain any {expected_msgtype} topic")
            sys.exit(1)
        return topics_of_type[0]

    def __repr__(self):
        bag_type = "TBD"
        path_info = "TBD"
        imu_info = f"imu_topic='{getattr(self, 'imu_topic', 'N/A')}'"
        lidar_info = f"lidar_topic='{getattr(self, 'lidar_topic', 'N/A')}'"
        msg_counts = (
            f"{self.bag.topics[self.imu_topic].msgcount if hasattr(self, 'bag') and self.imu_topic in self.bag.topics else 0} IMU msgs, "
            f"{self.bag.topics[self.lidar_topic].msgcount if hasattr(self, 'bag') and self.lidar_topic in self.bag.topics else 0} LiDAR msgs"
        )
        return f"<RosbagDataLoader({bag_type}, {path_info}, {imu_info}, {lidar_info}, {msg_counts})>"
