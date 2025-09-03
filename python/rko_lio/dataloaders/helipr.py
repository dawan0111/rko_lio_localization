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

import csv
from pathlib import Path

import numpy as np


class HeliprDataLoader:
    """
    Dataset loader for HeLiPR dataset.
    data_path: e.g. 'helipr/Bridge01'
    sequence: LiDAR sensor name: 'Aeva', 'Avia', 'Ouster', 'Velodyne'
    """

    def __init__(self, data_path: Path, sequence: str):
        from .. import rko_lio_pybind

        self.rko_lio_pybind = rko_lio_pybind

        from .helipr_file_reader_pybind import read_lidar_bin

        self.read_lidar_bin = read_lidar_bin

        self.data_path = Path(data_path)
        self.sensor = sequence  # e.g. "Aeva"
        self._imu_data = self._load_imu()
        self._lidar_entries = self._find_lidar_bin_files()
        self.entries = self._build_entries()

    def _load_imu(self):
        imu_file = self.data_path / "Inertial_data" / "xsens_imu.csv"
        assert imu_file.exists(), "{imu_file} does not exist for data path {data_path}"
        imu_data = []
        with open(imu_file, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                ts = int(row[0])
                gyro = np.array(
                    [float(row[8]), float(row[9]), float(row[10])], dtype=np.float32
                )
                accel = np.array(
                    [float(row[11]), float(row[12]), float(row[13])], dtype=np.float32
                )
                imu_data.append({"timestamp": ts, "gyro": gyro, "accel": accel})
        return imu_data

    def _find_lidar_bin_files(self):
        lidar_dir = self.data_path / "LiDAR" / self.sensor
        bin_files = sorted(lidar_dir.glob("*.bin"))
        if not bin_files:
            raise FileNotFoundError(f"No .bin files found in {lidar_dir}")
        lidar_entries = []
        for binfile in bin_files:
            ts = int(binfile.stem)  # ns int timestamp
            lidar_entries.append({"timestamp": ts, "filename": binfile})
        return lidar_entries

    def _build_entries(self):
        entries = []
        for imu in self._imu_data:
            entries.append(("imu", imu["timestamp"], imu))
        for lidar in self._lidar_entries:
            entries.append(("lidar", lidar["timestamp"], lidar))
        entries.sort(key=lambda x: x[1])
        return entries

    def __len__(self):
        return len(self.entries)

    def __getitem__(self, idx):
        kind, _, data = self.entries[idx]
        if kind == "imu":
            return (
                "imu",
                (
                    data["timestamp"] / 1e9,
                    data["accel"],
                    np.eye(3, dtype=np.float32),  # accel cov
                    data["gyro"],
                    np.eye(3, dtype=np.float32),  # gyro cov
                ),
            )
        elif kind == "lidar":
            header_stamp_sec = data["timestamp"] / 1e9
            points, raw_timestamps = self.read_lidar_bin(
                str(data["filename"]), self.sensor
            )
            points_arr = np.asarray(points).reshape(-1, 3)
            _, _, abs_timestamps = self.rko_lio_pybind._process_timestamps(
                self.rko_lio_pybind._VectorDouble(np.asarray(raw_timestamps)),
                header_stamp_sec,
                force_absolute=False,
            )
            return ("lidar", (points_arr, np.asarray(abs_timestamps)))
        else:
            raise IndexError("Unknown entry type.")

    def __repr__(self):
        return f"<HeliprDataLoader {self.data_path.name}, Sensor={self.sensor}, {len(self.entries)} entries>"
