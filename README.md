# fast-livo2-go2w

ROS 2 Humble workspace for running FAST-LIVO2 on a Unitree Go2W with a Hesai XT16 LiDAR, Go2W body IMU, and Intel RealSense color camera.

The Go2W local JetPack environment can be old, so this repository is intended to be built and developed inside Docker. The workspace keeps the sensor bringup packages and the Humble port of FAST-LIVO2 in one place.

## Status

- ROS distribution: Humble only
- Development environment: Docker
- LiDAR input: Hesai ROS 2 driver, `sensor_msgs/msg/PointCloud2`
- IMU input: Go2W lowstate bridge to `sensor_msgs/msg/Imu`
- Camera input: `realsense-ros` built from source
- FAST-LIVO2 source: ported from the public ROS Noetic implementation to ROS 2 Humble

The package builds in the Docker Humble environment. Runtime accuracy depends on replacing the placeholder calibration values in `humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml`.

## Clone

Clone this repository with submodules:

```bash
git clone --recurse-submodules https://github.com/torasuke12/fast-livo2-go2w.git
cd fast-livo2-go2w
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Repository Layout

```text
fast-livo2-go2w/
  docker/
    Dockerfile
    docker-compose.yml
    ros_entrypoint.sh
  humble_ws/
    src/
      fast-livo2/                  # submodule: ROS 2 Humble port of FAST-LIVO2
      go2w-hesai-lidar-driver/     # submodule
      go2w-imu-publisher/          # submodule
      unitree_ros2/                # submodule
      realsense-ros/               # submodule
      librealsense/                # submodule
  scripts/
    build_humble_ws.sh
    import_realsense_sources.sh
    run_sensors.sh
  dependencies.repos
```

## Sensor Topics

| Stream | Topic | Type |
| --- | --- | --- |
| Hesai XT16 LiDAR | `/hesai_node/points_raw` | `sensor_msgs/msg/PointCloud2` |
| Go2W IMU | `/go2w/imu` | `sensor_msgs/msg/Imu` |
| RealSense color image | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| RealSense camera info | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

FAST-LIVO2 subscribes to the first three topics. Use `camera_info` to fill the camera intrinsics in the FAST-LIVO2 config.

## Docker Setup

From this directory:

```bash
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml exec fast-livo2-go2w-humble bash
```

The container uses host networking, privileged device access, and Cyclone DDS:

- `network_mode: host` for ROS 2 and Unitree DDS discovery
- `/dev` mounted for LiDAR, RealSense, and robot interfaces
- `CYCLONEDDS_URI` points to the Unitree Cyclone DDS config
- `ROS_DOMAIN_ID` is inherited from the host, defaulting to `0`

## Submodules and RealSense Sources

The packages under `humble_ws/src` are managed as Git submodules. Update them with:

```bash
git submodule update --init --recursive
```

`realsense-ros` and `librealsense` are included as source submodules. This keeps the workspace independent from `ros-humble-realsense2-*` or `ros-humble-librealsense2*` apt packages.

If you intentionally remove the RealSense submodule directories and want to re-import them from `dependencies.repos`, run this inside the container:

```bash
cd /workspace/fast_livo2_go2w/src/fast-livo2-go2w
./scripts/import_realsense_sources.sh
```

## Build

Inside the container:

```bash
cd /workspace/fast_livo2_go2w/src/fast-livo2-go2w
./scripts/build_humble_ws.sh
```

The script runs `rosdep install` and builds the Humble workspace up to `fast_livo2`.

## Run Sensors Only

```bash
cd /workspace/fast_livo2_go2w/src/fast-livo2-go2w
./scripts/run_sensors.sh
```

Examples:

```bash
./scripts/run_sensors.sh color_profile:=1280x720x30 enable_depth:=true
./scripts/run_sensors.sh serial_no:=_123456789 camera_name:=d455
```

## Run FAST-LIVO2

After building:

```bash
source humble_ws/install/setup.bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py
```

By default, this launch file also starts the Hesai, Go2W IMU, and RealSense sensor bringup from the `fast_livo2` package.

Run FAST-LIVO2 only, assuming the sensor topics are already being published:

```bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py use_sensor_bringup:=false
```

Use a custom parameter file:

```bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py \
  params_file:=/path/to/go2w_fast_livo2_calibrated.yaml
```

## Calibration Requirements

`humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` contains bootstrapping values only. Before collecting meaningful odometry, replace these fields:

- `camera.cam_fx`, `camera.cam_fy`, `camera.cam_cx`, `camera.cam_cy`
- `camera.cam_d0` through `camera.cam_d4`
- `extrin_calib.extrinsic_T`
- `extrin_calib.extrinsic_R`
- `extrin_calib.Pcl`
- `extrin_calib.Rcl`
- `time_offset.imu_time_offset`
- `time_offset.img_time_offset`
- `time_offset.lidar_time_offset`

Interpretation:

- `extrinsic_T` and `extrinsic_R`: LiDAR to Go2W IMU transform used by the LIO state.
- `Pcl` and `Rcl`: LiDAR to RealSense color camera transform used by visual projection.
- Camera intrinsics should come from `/camera/camera/color/camera_info` for the exact RealSense device, profile, and resolution used at runtime.

Do not use the identity extrinsics in the default config for evaluation or field operation.

## FAST-LIVO2 Port Notes

The upstream FAST-LIVO2 code was ROS Noetic based. This workspace adds a ROS 2 Humble package named `fast_livo2` with:

- `rclcpp` node, parameters, publishers, subscribers, and timers
- ROS 2 message types and `tf2_ros` transform publishing
- Standard `sensor_msgs/msg/PointCloud2` input path for the Hesai driver
- Minimal compatibility replacements for the vikit camera utilities used by FAST-LIVO2
- RealSense-compatible RadTan and equidistant camera model loading from ROS 2 parameters

Livox `CustomMsg` compatibility structs are kept only so the original preprocessing code can compile. The Go2W runtime path uses Hesai `PointCloud2`.

## Useful Commands

Check launch arguments:

```bash
source humble_ws/install/setup.bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py --show-args
```

Check expected topics:

```bash
ros2 topic list
ros2 topic echo /go2w/imu --once
ros2 topic hz /hesai_node/points_raw
ros2 topic hz /camera/camera/color/image_raw
```

Rebuild only FAST-LIVO2:

```bash
cd humble_ws
colcon build --symlink-install --packages-select fast_livo2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Known Limitations

- The default FAST-LIVO2 config is not calibrated for any specific Go2W hardware.
- RealSense intrinsics depend on camera model, serial, resolution, and stream profile.
- Go2W IMU, Hesai LiDAR, and RealSense camera time synchronization must be validated on the target robot.
- The ROS 2 port is focused on the Go2W/Hesai/RealSense data path; unrelated LiDAR modes from the original FAST-LIVO2 source are preserved where practical but are not the primary supported runtime path.

## License and Attribution

This workspace contains a ROS 2 Humble port of FAST-LIVO2 plus integration with Go2W sensor packages. Keep the upstream FAST-LIVO2 license and attribution when redistributing the `fast_livo2` package, and keep third-party source licenses for `realsense-ros`, `librealsense`, Unitree ROS 2 interfaces, and the Hesai driver.
