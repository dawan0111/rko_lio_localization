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
Equivalent logic to the ros wrapper's message buffering.
A convenience class to buffer IMU and LiDAR messages to ensure the core cpp implementation always
gets the data in sync.
"""

from pathlib import Path

import numpy as np

from .lio import LIO, LIOConfig


class LIOPipeline:
    """
    Minimal sequential pipeline for LIO processing with out-of-sync IMU/lidar.
    Buffers are managed internally; data arrives via add_imu and add_lidar.
    When IMU data covers an already available lidar frame, registration is triggered.
    """

    def __init__(
        self,
        config: LIOConfig,
        extrinsic_imu2base: np.ndarray | None = None,
        extrinsic_lidar2base: np.ndarray | None = None,
        viz: bool = False,
    ):
        """
        Parameters
        ----------
        config : LIOConfig
            Config used to instantiate a LIO estimator.
        extrinsic_imu2base : np.ndarray[4,4] or None
            If provided, used for all IMU updates.
        extrinsic_lidar2base : np.ndarray[4,4] or None
            If provided, used for all scan registrations.
        """
        self.lio = LIO(config)
        self.extrinsic_imu2base = extrinsic_imu2base
        self.extrinsic_lidar2base = extrinsic_lidar2base
        self.viz = viz
        if viz:
            import rerun

            self.rerun = rerun

        # Each: dict with keys 'time', 'accel', 'accel_cov', 'gyro', 'gyro_cov'
        self.imu_buffer: list[dict] = []
        # Each: dict with keys 'start_time', 'end_time', 'scan', 'timestamps'
        self.lidar_buffer: list[dict] = []
        self.last_xyz = np.zeros(3)

    def add_imu(
        self,
        time: float,
        acceleration: np.ndarray,
        acceleration_cov: np.ndarray,
        angular_velocity: np.ndarray,
        angular_velocity_cov: np.ndarray,
    ):
        """
        Add IMU measurement to pipeline (will be buffered until processed by lidar).

        Parameters
        ----------
        time : float
            Measurement timestamp in seconds.
        acceleration : array-like of float, shape (3,)
            Acceleration vector in m/s^2.
        acceleration_cov : array-like of float, shape (3,3)
            Acceleration covariance matrix.
        angular_velocity : array-like of float, shape (3,)
            Angular velocity in rad/s.
        angular_velocity_cov : array-like of float, shape (3,3)
            Angular velocity covariance matrix.
        """
        self.imu_buffer.append(
            dict(
                time=float(time),
                acceleration=np.asarray(acceleration, dtype=np.float64),
                acceleration_cov=np.asarray(acceleration_cov, dtype=np.float64),
                angular_velocity=np.asarray(angular_velocity, dtype=np.float64),
                angular_velocity_cov=np.asarray(angular_velocity_cov, dtype=np.float64),
            )
        )
        self._try_register()

    def add_lidar(
        self,
        scan: np.ndarray,
        timestamps: np.ndarray,
    ):
        """
        Add a lidar point cloud and *absolute* timestamps. Scan start and end times
        are inferred from the timestamps vector.

        Parameters
        ----------
        scan : array-like of float, shape (N,3)
            Point cloud (N points, 3 spatial dims).
        timestamps : array-like of float, shape (N,)
            Absolute timestamps (seconds) for each point.

        Raises
        ------
        ValueError
            If input shapes are not compatible.
        """
        scan_arr = np.asarray(scan, dtype=np.float64)
        time_arr = np.asarray(timestamps, dtype=np.float64)
        if scan_arr.ndim != 2 or scan_arr.shape[1] != 3:
            raise ValueError(f"scan: expected (N,3), got {scan_arr.shape}")
        if time_arr.shape != (scan_arr.shape[0],):
            raise ValueError(
                f"timestamps: expected ({scan_arr.shape[0]},), got {time_arr.shape}"
            )

        start_time = float(np.min(time_arr))
        end_time = float(np.max(time_arr))
        self.lidar_buffer.append(
            dict(
                scan=scan_arr,
                timestamps=time_arr,
                start_time=start_time,
                end_time=end_time,
            )
        )
        self._try_register()

    def _try_register(self):
        """
        Try to align and process available lidar with buffered imu.
        If latest IMU.timestamp > earliest lidar.end_time, start popping and then register.
        """
        while (
            self.lidar_buffer
            and self.imu_buffer
            and self.imu_buffer[-1]["time"] > self.lidar_buffer[0]["end_time"]
        ):
            frame = self.lidar_buffer.pop(0)
            # Find all IMU measurements up to lidar end_time
            imu_to_process = [
                imu for imu in self.imu_buffer if imu["time"] < frame["end_time"]
            ]
            for imu in imu_to_process:
                if self.extrinsic_imu2base is not None:
                    self.lio.update_imu_motion_with_extrinsic(
                        self.extrinsic_imu2base,
                        imu["acceleration"],
                        imu["acceleration_cov"],
                        imu["angular_velocity"],
                        imu["angular_velocity_cov"],
                        imu["time"],
                    )
                else:
                    self.lio.update_imu_motion(
                        imu["acceleration"],
                        imu["acceleration_cov"],
                        imu["angular_velocity"],
                        imu["angular_velocity_cov"],
                        imu["time"],
                    )
            # Remove processed IMUs from buffer (those with time < lidar end_time)
            self.imu_buffer = [
                imu for imu in self.imu_buffer if imu["time"] >= frame["end_time"]
            ]
            # Register the lidar scan
            deskewed_scan = None
            if self.extrinsic_lidar2base is not None:
                deskewed_scan = self.lio.register_scan_with_extrinsic(
                    self.extrinsic_lidar2base,
                    frame["scan"],
                    frame["timestamps"],
                )
            else:
                deskewed_scan = self.lio.register_scan(
                    frame["scan"],
                    frame["timestamps"],
                )
            # save_deskewed_scan_as_ply(
            #     deskewed_scan, self.lio.pose(), frame["end_time"], output_dir="ply_dump"
            # )

            if self.viz:
                if self.lio.config.initialization_phase:
                    self.rerun.log(
                        "world", self.rerun.ViewCoordinates.RIGHT_HAND_Z_UP, static=True
                    )
                self.rerun.set_time("data_time", timestamp=frame["end_time"])
                pose = self.lio.pose()
                self.rerun.log(
                    "world/lidar",
                    self.rerun.Transform3D(
                        translation=pose[:3, 3], mat3x3=pose[:3, :3], axis_length=2
                    ),
                )
                self.rerun.log(
                    "world/lidar/raw_san", self.rerun.Points3D(frame["scan"])
                )

                self.rerun.log(
                    "world/lidar/deskewed_scan", self.rerun.Points3D(deskewed_scan)
                )
                self.rerun.log(
                    "world/view_anchor",
                    self.rerun.Transform3D(translation=pose[:3, 3]),
                )
                traj_pts = np.array([self.last_xyz, pose[:3, 3]])
                self.rerun.log(
                    "world/trajectory",
                    self.rerun.LineStrips3D(
                        [traj_pts], radii=[0.1], colors=[255, 111, 111]
                    ),
                )
                self.last_xyz = pose[:3, 3].copy()
                local_map_points = self.lio.local_map_point_cloud()
                if local_map_points.size > 0:
                    colors = height_colors_from_points(local_map_points)
                    self.rerun.log(
                        "world/local_map",
                        self.rerun.Points3D(local_map_points, colors=colors),
                    )

    def dump_results_to_disk(self, results_dir: Path, run_name: str):
        """
        Write all result data to disk.

        Parameters
        ----------
        results_dir : str or pathlib.Path
            Output directory.
        run_name : str
            Output file naming prefix.
        """
        self.lio.dump_results_to_disk(str(results_dir), str(run_name))


def save_deskewed_scan_as_ply(
    deskewed_scan: np.ndarray,
    pose: np.ndarray,
    end_time_seconds: float,
    output_dir: str = "ply_dump",
):
    """
    Transforms the deskewed_scan by pose and dumps as PLY.
    The filename is <int_nanoseconds>.ply based on end_time_seconds.
    """
    import open3d as o3d

    if deskewed_scan is None or len(deskewed_scan) == 0:
        return

    Path(output_dir).mkdir(exist_ok=True, parents=True)

    fname = Path(output_dir) / f"{int(end_time_seconds * 1e9)}.ply"

    ones = np.ones((deskewed_scan.shape[0], 1), dtype=deskewed_scan.dtype)
    pts_hom = np.hstack([deskewed_scan, ones])  # (N, 4)
    pts_tr = (pose @ pts_hom.T).T[:, :3]  # (N, 3)

    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(pts_tr)
    o3d.io.write_point_cloud(fname.as_posix(), pc_o3d)


def height_colors_from_points(points: np.ndarray) -> np.ndarray:
    """
    Given Nx3 array of points, return Nx3 array of RGB colors (dtype uint8)
    mapped from the z values using the viridis colormap.
    Colors are uint8 in range [0, 255].
    """
    import matplotlib.pyplot as plt
    from matplotlib import cm

    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    if z_max == z_min:
        norm_z = np.zeros_like(z)
    else:
        norm_z = (z - z_min) / (z_max - z_min)

    viridis = cm.get_cmap("viridis")
    colors_float = viridis(norm_z)[:, :3]  # Nx3 floats in [0,1]

    colors_uint8 = (colors_float * 255).astype(np.uint8)
    return colors_uint8
