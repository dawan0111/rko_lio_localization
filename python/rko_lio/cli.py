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
Entrypoint typer application for the python wrapper.
"""

from pathlib import Path

import numpy as np
import typer


def dataloader_name_callback(value: str):
    from .dataloaders import available_dataloaders

    if not value:
        return value
    dl = available_dataloaders()
    if value.lower() not in [d.lower() for d in dl]:
        raise typer.BadParameter(f"Supported dataloaders are: {', '.join(dl)}")
    # Normalize case to match one of the known dataloaders
    for d in dl:
        if value.lower() == d.lower():
            return d
    return value


def convert_quat_xyzw_xyz_to_matrix(quat_xyzw_xyz: np.ndarray) -> np.ndarray:
    """
    Convert [qx, qy, qz, qw, x, y, z] to 4x4 transformation matrix.
    """
    from scipy.spatial.transform import Rotation

    quat = quat_xyzw_xyz[:4]
    xyz = quat_xyzw_xyz[4:]
    rot = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot.as_matrix()
    transform[:3, 3] = xyz
    return transform


def get_extrinsic_from_cli(cli_value):
    """
    Given CLI extrinsic values, return the qx, qy, qz, qw, x, y, z
    """
    if cli_value is None:
        return None
    vals = [float(x) for x in cli_value.replace(" ", "").split(",") if x]
    if len(vals) != 7:
        raise ValueError(
            "Extrinsic CLI value must be 7 comma-separated numbers: qx,qy,qz,qw,x,y,z"
        )
    return np.array(vals, dtype=np.float64)


def handle_extrinsics(config_data: dict, cli_ext_imu2base, cli_ext_lidar2base):
    # because this has a fair bit of logic going on, splitting it off
    extrinsic_imu2base = None
    extrinsic_lidar2base = None

    imu_config_val = config_data.pop("extrinsic_imu2base_quat_xyz_xyz", None)
    lidar_config_val = config_data.pop("extrinsic_lidar2base_quat_xyz_xyz", None)

    imu_cli_arr = get_extrinsic_from_cli(cli_ext_imu2base)
    if imu_cli_arr is not None:
        if imu_config_val is not None:
            print(
                "WARNING: Both CLI and config for IMU extrinsic given. CLI takes priority."
            )
        extrinsic_imu2base = convert_quat_xyzw_xyz_to_matrix(imu_cli_arr)
    elif imu_config_val is not None:
        extrinsic_imu2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(imu_config_val, dtype=np.float64)
        )

    lidar_cli_arr = get_extrinsic_from_cli(cli_ext_lidar2base)
    if lidar_cli_arr is not None:
        if lidar_config_val is not None:
            print(
                "WARNING: Both CLI and config for lidar extrinsic given. CLI takes priority."
            )
        extrinsic_lidar2base = convert_quat_xyzw_xyz_to_matrix(lidar_cli_arr)
    elif lidar_config_val is not None:
        extrinsic_lidar2base = convert_quat_xyzw_xyz_to_matrix(
            np.asarray(lidar_config_val, dtype=np.float64)
        )

    if extrinsic_imu2base is None and extrinsic_lidar2base is None:
        print(
            "WARNING: You're not passing any extrinsics, which means IMU and LiDAR frame are assumed to be the same. Which is probably wrong. Pass the extrinsics as part of the config file or as cli arguments --ext_imu2base or --ext_lidar2base"
        )
    return extrinsic_imu2base, extrinsic_lidar2base


app = typer.Typer()


@app.command()
def cli(
    data_path: Path = typer.Argument(..., exists=True, help="Path to data folder"),
    config_fp: Path | None = typer.Option(
        None, "--config", "-c", exists=True, help="Path to config.yaml"
    ),
    dataloader: str | None = typer.Option(
        None,
        "--dataloader",
        "-d",
        help="Specify a dataloader: [rosbag, raw, helipr]",  # make this generic later
        show_choices=True,
        callback=dataloader_name_callback,
        case_sensitive=False,
    ),
    viz: bool = typer.Option(False, "--viz", "-v", help="Enable Rerun visualization"),
    log_results: bool = typer.Option(
        True,
        "--log",
        "-l",
        help="Log trajectory results to disk at 'results_dir' on completion",
    ),
    results_dir: Path | None = typer.Option(
        "results", "--results_dir", "-r", help="Where to dump LIO results"
    ),
    run_name: str = typer.Option(
        "rko_lio_experiment", "--run_name", "-n", help="Name prefix for output files"
    ),
    sequence: str | None = typer.Option(
        None,
        "--sequence",
        help="Extra dataloader argument: sensor sequence (for helipr only)",
    ),
    imu_topic: str | None = typer.Option(
        None, "--imu", help="Extra dataloader argument: imu topic (for rosbag only)"
    ),
    lidar_topic: str | None = typer.Option(
        None, "--lidar", help="Extra dataloader argument: lidar topic (for rosbag only)"
    ),
    cli_ext_imu2base: str | None = typer.Option(
        None,
        "--ext_imu2base",
        help="Extrinsic IMU-to-base transformation (as 7 floats: qx,qy,qz,qw,x,y,z)",
    ),
    cli_ext_lidar2base: str | None = typer.Option(
        None,
        "--ext_lidar2base",
        help="Extrinsic lidar-to-base transformation (as 7 floats: qx,qy,qz,qw,x,y,z)",
    ),
):
    """
    Run LIO pipeline with the selected dataloader and parameters.
    """

    if viz:
        try:
            import rerun as rr

            rr.init("rko_lio")
            rr.spawn(memory_limit="15GB")

        except ImportError:
            raise ImportError(
                "Please install rerun with `pip install rerun-sdk` to enable visualization."
            )

    config_data = {}
    if config_fp:
        with open(config_fp, "r") as f:
            import yaml

            config_data.update(yaml.safe_load(f))

    extrinsic_imu2base, extrinsic_lidar2base = handle_extrinsics(
        config_data, cli_ext_imu2base, cli_ext_lidar2base
    )

    from .lio import LIOConfig
    from .lio_pipeline import LIOPipeline

    config = LIOConfig(**config_data)
    pipeline = LIOPipeline(
        config,
        extrinsic_imu2base=extrinsic_imu2base,
        extrinsic_lidar2base=extrinsic_lidar2base,
        viz=viz,
    )

    from .dataloaders import get_dataloader

    dataloader = get_dataloader(dataloader, data_path, sequence, imu_topic, lidar_topic)
    data_count = len(dataloader)
    print("Loaded dataloader:", dataloader)

    from tqdm import tqdm

    from .scoped_profiler import ScopedProfiler

    for idx in tqdm(range(data_count), total=data_count, desc="Data"):
        kind, data_tuple = dataloader[idx]

        with ScopedProfiler("Registration") as reg_timer:
            if kind == "imu":
                pipeline.add_imu(*data_tuple)
            elif kind == "lidar":
                pipeline.add_lidar(*data_tuple)
            else:
                raise ValueError(f"data type {kind} is invalid from the dataloader")

    # Output results
    if log_results and results_dir:
        results_dir.mkdir(parents=True, exist_ok=True)
        pipeline.dump_results_to_disk(results_dir, run_name)
        print(f"Results dumped to: {results_dir} [prefix: {run_name}]")


if __name__ == "__main__":
    app()
