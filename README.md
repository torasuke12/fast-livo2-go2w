# fast-livo2-go2w

ROS 2 Humble workspace for running **FAST-LIVO2** on a Unitree GO2-W with a Hesai PandarXT-16 LiDAR, the GO2-W body IMU, and an Intel RealSense color camera.

This repository follows the same deployment shape as `dlio-go2w`: the robot-side stack lives in `humble_ws/src`, the Jetson image is under `docker/robot`, robot sessions are defined by `catmux`, and bag replay/reconstruction helpers live under `scripts/fast_livo2`.

## Attribution

FAST-LIVO2 is developed by HKU-MARS:

- Upstream project: `hku-mars/FAST-LIVO2`
- Paper: FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

This workspace uses a ROS 2 Humble Go2W integration package at `humble_ws/src/fast-livo2`. In the `dlio-go2w` architecture, this is the replacement for `humble_ws/src/direct_lidar_inertial_odometry`.

## Repository Layout

```text
fast-livo2-go2w/
├── humble_ws/src/
│   ├── fast-livo2/                  FAST-LIVO2 ROS 2 Humble port (submodule)
│   ├── go2w-imu-publisher/          IMU republisher from GO2-W lowstate (submodule)
│   ├── go2w-hesai-lidar-driver/     Hesai PandarXT-16 ROS 2 driver (submodule)
│   ├── unitree_ros2/                Unitree DDS bindings + setup.sh (submodule)
│   ├── realsense-ros/               RealSense ROS 2 camera driver (submodule)
│   └── librealsense/                RealSense SDK source dependency (submodule)
├── docker/robot/                    ARM64 ROS 2 Humble image for the Jetson
├── .devcontainer/                   Desktop devcontainer for bag replay/reconstruction
├── catmux/                          Robot-side tmux session definitions
├── scripts/fast_livo2/              Desktop/robot offline helpers
├── config/fast_livo2/               RViz config for FAST-LIVO2 outputs
├── config/sensor/                   Shared Go2W calibration reference
└── bags/                            Bag storage (gitignored)
```

`config/sensor/go2w_calibration.yaml` is the shared calibration reference. FAST-LIVO2 still consumes its native parameter file at `humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml`.

## Submodules

| Package | Repository | Purpose |
|---|---|---|
| `fast-livo2` | `torasuke12/fast-livo2` | FAST-LIVO2 ROS 2 Humble port |
| `go2w-imu-publisher` | `koki67/go2w-imu-publisher` | GO2-W IMU republisher |
| `go2w-hesai-lidar-driver` | `koki67/go2w-hesai-lidar-driver` | Hesai XT16 ROS 2 driver |
| `unitree_ros2` | `koki67/unitree_ros2` | Unitree ROS 2 bindings and DDS setup |
| `realsense-ros` | `realsenseai/realsense-ros` | RealSense camera driver |
| `librealsense` | `realsenseai/librealsense` | RealSense SDK source |

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/torasuke12/fast-livo2-go2w.git
cd fast-livo2-go2w
```

If you already cloned without submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Docker Setup

Before building or entering the container, make sure the source submodules are
present. A clone made without `--recurse-submodules` leaves the directories
under `humble_ws/src` empty, and `catmux_create_session` will later fail because
the ROS packages are missing.

From the repository root on the host:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

The GO2-W Jetson Docker daemon may only support Docker API `1.43`. Newer Docker CLI/Compose clients can fail with:

```text
client version 1.53 is too new. Maximum supported API version is 1.43
```

The robot scripts below set `DOCKER_API_VERSION=1.43` by default, matching the workaround used for older Jetson Docker daemons.
If you run the same scripts against a newer Docker daemon that rejects API `1.43` as too old, override it explicitly, for example `DOCKER_API_VERSION=1.44 bash docker/robot/build.sh`.

1. Build the robot Docker image on the Jetson:

   ```bash
   bash docker/robot/build.sh
   ```

2. Start an interactive robot container.

   Standard robot script:

   ```bash
   bash docker/robot/run.sh
   ```

   Compatibility Compose wrapper, if you prefer Docker Compose:

   ```bash
   ./docker/compose.sh up -d --build
   ./docker/compose.sh exec fast-livo2-go2w-humble bash
   ```

   The Compose wrapper is only an alternative way to start and enter the same
   robot container. It does not replace the workspace build step below.

3. Required: inside the container, initialize/build the ROS workspace before
   running any `catmux_create_session` command:

   ```bash
   cd /external
   bash scripts/build_humble_ws.sh
   source humble_ws/install/setup.bash
   ```

Do not use bare `docker compose -f docker/docker-compose.yml ...` on the Jetson unless you export `DOCKER_API_VERSION=1.43` yourself. A Compose file cannot reliably set the Docker client API version before the client connects to the daemon.

## Quick Start: Online FAST-LIVO2

Complete Docker setup first, then inside the robot container:

```bash
catmux_create_session /external/catmux/online_fast_livo2.yaml
```

This is the pure live launch session. It starts:

- GO2-W IMU publisher
- Hesai LiDAR driver through `scripts/run_hesai_lidar_points.sh`
- RealSense color camera driver
- FAST-LIVO2 mapping node

It does not start RViz and does not record bags.

The Hesai wrapper remaps the driver's relative `points_raw` publisher to
`/hesai_node/points_raw`, matching the FAST-LIVO2 config, and sets
`publish_type:=points` so the unused raw packet topic is not published.

CycloneDDS uses the Unitree config at `humble_ws/src/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml`.

## Online FAST-LIVO2 With RViz

Use this when you want the same live sensor/FAST-LIVO2 stack plus robot-side
RViz visualization:

```bash
catmux_create_session /external/catmux/online_fast_livo2_rviz.yaml
```

RViz uses `config/fast_livo2/fast_livo2.rviz` and visualizes the main FAST-LIVO2
outputs:

- `/aft_mapped_to_init`
- `/path`
- `/cloud_registered`
- `/Laser_map`
- `/rgb_img`
- `/LIVO2/imu_propagate`

For desktop-side live RViz over WiFi, use:

```bash
bash scripts/fast_livo2/live_rviz.sh --iface enp97s0
```

## Bag Types

| Bag prefix | Contents | Use for |
|---|---|---|
| `fast_livo2_YYYYMMDD_HHMMSS` | FAST-LIVO2 output topics such as odom, path, map, image, and TF | Fast replay without rerunning FAST-LIVO2 |
| `raw_YYYYMMDD_HHMMSS` | Raw sensors: IMU, LiDAR, color image, camera info | Re-running FAST-LIVO2 with changed parameters |

## Record FAST-LIVO2 Outputs

Use this when you want compact replay of already-computed FAST-LIVO2 results.
This starts the live stack and records only processed output topics:

```bash
catmux_create_session /external/catmux/record_fast_livo2.yaml
```

Recorded topics:

```text
/aft_mapped_to_init
/path
/cloud_registered
/Laser_map
/rgb_img
/LIVO2/imu_propagate
/tf
/tf_static
```

Bags are saved to `/external/bags/fast_livo2_YYYYMMDD_HHMMSS`.

## Record Raw Sensor Data

Use this when you want to re-run FAST-LIVO2 later with changed parameters. This
starts only the sensor bringup and records raw sensor topics:

```bash
catmux_create_session /external/catmux/record_raw.yaml
```

Recorded topics:

```text
/go2w/imu
/hesai_node/points_raw
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Copy and Replay Bags on a Visualization PC

Use this workflow when live RViz over DDS is unstable or when you want to inspect
the FAST-LIVO2 output offline on the visualization PC. Record on the robot side
first, copy the whole bag directory, then replay it locally with RViz.

1. Record a processed FAST-LIVO2 output bag inside the robot container:

   ```bash
   mkdir -p /external/bags
   catmux_create_session /external/catmux/record_fast_livo2.yaml
   ```

   Stop the `bag_record` pane when the run is finished. Confirm the recorded bag
   directory and metadata:

   ```bash
   ls -lh /external/bags
   ros2 bag info /external/bags/fast_livo2_YYYYMMDD_HHMMSS
   ```

   If you need to re-run FAST-LIVO2 later with changed parameters, record raw
   sensor data instead:

   ```bash
   catmux_create_session /external/catmux/record_raw.yaml
   ros2 bag info /external/bags/raw_YYYYMMDD_HHMMSS
   ```

2. Copy the complete bag directory to the visualization PC. Do not copy only the
   `.db3` file; `metadata.yaml` must be copied with it.

   From the robot host or container:

   ```bash
   scp -r /external/bags/fast_livo2_YYYYMMDD_HHMMSS \
     srg-f-rei@192.168.123.203:/home/srg-f-rei/fast_livo2_go2w/src/fast-livo2-go2w/bags/
   ```

   Use the destination user's home directory, not `/home` itself. If the local
   `bags` directory does not exist on the visualization PC, create it first:

   ```bash
   mkdir -p ~/fast_livo2_go2w/src/fast-livo2-go2w/bags
   ```

3. Replay the processed FAST-LIVO2 output on the visualization PC:

   ```bash
   cd ~/fast_livo2_go2w/src/fast-livo2-go2w
   bash scripts/fast_livo2/playback.sh bags/fast_livo2_YYYYMMDD_HHMMSS
   ```

   The helper plays the bag with `/clock`, isolates playback on a local ROS
   domain, and opens RViz with `use_sim_time:=true`.

4. Check replayed topics from another terminal if RViz does not show an output:

   ```bash
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=1
   unset CYCLONEDDS_URI

   ros2 topic list
   ros2 topic hz /cloud_registered
   ros2 topic echo --once --field width /cloud_registered
   ros2 topic hz /tf
   ```

For looped playback, run the bag player manually in one terminal and RViz in
another:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

ros2 bag play bags/fast_livo2_YYYYMMDD_HHMMSS --clock --loop
```

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

rviz2 -d config/fast_livo2/fast_livo2.rviz --ros-args -p use_sim_time:=true
```

## Common Robot Sessions

```bash
catmux_create_session /external/catmux/online_fast_livo2.yaml
catmux_create_session /external/catmux/online_fast_livo2_rviz.yaml
catmux_create_session /external/catmux/record_fast_livo2.yaml
catmux_create_session /external/catmux/record_raw.yaml
```

Edit the `bag:` parameter in these files before replay/reconstruction:

```bash
catmux_create_session /external/catmux/playback_fast_livo2.yaml
catmux_create_session /external/catmux/reconstruct_raw_fast_livo2.yaml
```

## Desktop Helpers

Open the repository in the devcontainer for desktop-side RViz and offline runs.
All helpers use `config/fast_livo2/fast_livo2.rviz`.

Live RViz over WiFi:

```bash
bash scripts/fast_livo2/live_rviz.sh --iface enp97s0
```

The live RViz helper uses CycloneDDS when `rmw_cyclonedds_cpp` is installed. If
the desktop reports that `librmw_cyclonedds_cpp.so` is missing, install the
package for the desktop ROS 2 distro:

```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

For Ubuntu 24.04 / ROS 2 Jazzy, use `sudo apt install
ros-jazzy-rmw-cyclonedds-cpp`.

If you intentionally want to use the desktop's current ROS 2 RMW instead, run:

```bash
bash scripts/fast_livo2/live_rviz.sh --iface enp97s0 --rmw system
```

Replay processed FAST-LIVO2 output:

```bash
bash scripts/fast_livo2/playback.sh bags/fast_livo2_YYYYMMDD_HHMMSS
```

Reconstruct FAST-LIVO2 output from raw sensors:

```bash
bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_YYYYMMDD_HHMMSS
```

## Sensor Topics

| Stream | Topic | Type |
|---|---|---|
| Hesai XT16 LiDAR | `/hesai_node/points_raw` | `sensor_msgs/msg/PointCloud2` |
| GO2-W IMU | `/go2w/imu` | `sensor_msgs/msg/Imu` |
| RealSense color image | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| RealSense camera info | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

FAST-LIVO2 subscribes to LiDAR, IMU, and image. Use `camera_info` to fill camera intrinsics in the FAST-LIVO2 config.

## Direct Launch

Run FAST-LIVO2 with sensor bringup:

```bash
source humble_ws/install/setup.bash
source humble_ws/src/unitree_ros2/setup.sh
ros2 launch fast_livo2 go2w_fast_livo2.launch.py
```

Run FAST-LIVO2 only when sensor topics are already available:

```bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py use_sensor_bringup:=false
```

Use a calibrated parameter file:

```bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py \
  params_file:=/path/to/go2w_fast_livo2_calibrated.yaml
```

## Calibration Requirements

`humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` contains bootstrapping values only. Replace these before collecting meaningful odometry:

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

- `extrinsic_T` and `extrinsic_R`: LiDAR to GO2-W IMU transform used by the LIO state.
- `Pcl` and `Rcl`: LiDAR to RealSense color camera transform used by visual projection.

Do not use the identity extrinsics in the default config for evaluation or field operation.

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

If `/hesai_node/points_raw` is missing, check the `hesai_lidar_node` catmux
window. The expected launch path is `bash /external/scripts/run_hesai_lidar_points.sh`;
using the upstream `ros2 launch hesai_lidar hesai_lidar_launch.py` directly can
publish the point cloud on a different topic and also enables the unused raw
packet publisher.

Rebuild only FAST-LIVO2:

```bash
cd humble_ws
colcon build --symlink-install --packages-select fast_livo2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## License

This workspace contains a ROS 2 Humble port of FAST-LIVO2 plus integration with GO2-W sensor packages. Keep upstream FAST-LIVO2 attribution and licensing when redistributing `fast_livo2`, and keep third-party source licenses for `realsense-ros`, `librealsense`, Unitree ROS 2 interfaces, and the Hesai driver.
