# fast-livo2-go2w

ROS 2 Humble workspace for running **FAST-LIVO2** on a Unitree GO2-W with a Livox MID360 LiDAR, the MID360 internal IMU, and an Intel RealSense color camera.

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
│   ├── livox_ros_driver2/           Livox MID360 ROS 2 driver (linked from sibling clone)
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

`config/sensor/go2w_mid360_calibration.yaml` is the shared Livox MID360 calibration reference. FAST-LIVO2 still consumes its native parameter file at `humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml`. The previous Hesai/GO2-W IMU parameter file is preserved as `humble_ws/src/fast-livo2/config/go2w_hesai_fast_livo2.yaml`.

## Package Sources

| Package | Repository | Purpose |
|---|---|---|
| `fast-livo2` | `torasuke12/fast-livo2` | FAST-LIVO2 ROS 2 Humble port |
| `livox_ros_driver2` | `Livox-SDK/livox_ros_driver2` | Livox MID360 ROS 2 driver |
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

The Livox MID360 driver is expected to be cloned next to this repository, for
example:

```bash
cd ~/fast_livo2_go2w/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

`scripts/build_humble_ws.sh` links that clone into `humble_ws/src/livox_ros_driver2`
and copies `package_ROS2.xml` to `package.xml` before building. In the robot
container, `docker/robot/run.sh` mounts the sibling clone at
`/fast_livo2/src/livox_ros_driver2`. If your clone lives elsewhere, set
`LIVOX_ROS_DRIVER2_HOST=/path/to/livox_ros_driver2` before starting the
container.

If you already cloned the driver directly under `humble_ws/src` as either
`livox_ros_driver2` or `livox_ros_driver`, `scripts/build_humble_ws.sh` will use
that source tree and prepare the ROS 2 manifest.

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

   The image installs Livox-SDK2, which is required to build and run
   `livox_ros_driver2`. Rebuild this image after pulling the Livox MID360
   support changes.

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

- Livox MID360 driver through `scripts/run_livox_mid360_points.sh`
- MID360 internal IMU published by `livox_ros_driver2`
- RealSense color camera driver
- FAST-LIVO2 mapping node

It does not start RViz and does not record bags.

The Livox wrapper launches `livox_ros_driver2` with `xfer_format:=0`, so FAST-LIVO2 subscribes to `/livox/lidar` as `sensor_msgs/msg/PointCloud2` and `/livox/imu` as `sensor_msgs/msg/Imu`.

The previous Hesai XT16 + GO2-W IMU session is still available:

```bash
catmux_create_session /external/catmux/online_fast_livo2_hesai.yaml
```

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
/livox/lidar
/livox/imu
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

   If you copied a raw sensor bag and want to re-run FAST-LIVO2 from the
   recorded LiDAR, IMU, and camera topics, use the reconstruction helper:

   ```bash
   cd ~/fast_livo2_go2w/src/fast-livo2-go2w
   bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_YYYYMMDD_HHMMSS
   ```

   The reconstruction helper delays `ros2 bag play` by 5 seconds by default so
   FAST-LIVO2 subscriptions and bag publishers can finish DDS discovery before
   sensor messages start. If startup sync warnings still appear, increase the
   delay with `FAST_LIVO2_BAG_PLAY_DELAY=10` or pass `--delay 10` after the bag
   path.

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

### CycloneDDS Domain Creation Error on a Visualization PC

If every ROS 2 command fails on a non-robot visualization PC with an error like
this:

```text
[rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain
what(): failed to initialize rcl node: rcl node's rmw handle is invalid
```

the usual cause is a robot-side CycloneDDS configuration being sourced on the
visualization PC. In particular, `CYCLONEDDS_URI` may point to a Unitree or
robot network config that names an interface that does not exist on the
visualization PC.

For local bag replay or raw-bag reconstruction on the visualization PC, use a
local-only ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
source humble_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

ros2 daemon stop
ros2 topic list
```

Check the current DDS-related environment when diagnosing this:

```bash
env | grep -E '^(RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|CYCLONEDDS_URI)='
ip -br link
```

Do not source the Unitree `setup.sh` on the visualization PC for offline bag
work unless you also provide a CycloneDDS XML file that matches that PC's
network interface. If you need live DDS communication with the robot instead of
local bag replay, set `ROS_LOCALHOST_ONLY=0` and make `CYCLONEDDS_URI` point to
a CycloneDDS config whose network interface exists on the visualization PC.

If CycloneDDS still fails and you only need local bag playback, switch to the
default Fast DDS RMW for that terminal:

```bash
source /opt/ros/humble/setup.bash
source humble_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic list
```

## Common Robot Sessions

```bash
catmux_create_session /external/catmux/online_fast_livo2.yaml
catmux_create_session /external/catmux/online_fast_livo2_rviz.yaml
catmux_create_session /external/catmux/record_fast_livo2.yaml
catmux_create_session /external/catmux/record_raw.yaml
```

Legacy Hesai XT16 + GO2-W IMU sessions:

```bash
catmux_create_session /external/catmux/online_fast_livo2_hesai.yaml
catmux_create_session /external/catmux/online_fast_livo2_hesai_rviz.yaml
catmux_create_session /external/catmux/record_fast_livo2_hesai.yaml
catmux_create_session /external/catmux/record_raw_hesai.yaml
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

## Sensor Topics

| Stream | Topic | Type |
|---|---|---|
| Livox MID360 LiDAR | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` |
| Livox MID360 IMU | `/livox/imu` | `sensor_msgs/msg/Imu` |
| RealSense color image | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| RealSense camera info | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

FAST-LIVO2 subscribes to LiDAR, IMU, and image. Use `camera_info` to fill camera intrinsics in the FAST-LIVO2 config. If no parameter file is loaded, the compiled fallback defaults are the Livox MID360 topics: `/livox/lidar` and `/livox/imu`.

Legacy Hesai topics are still supported through the `_hesai` catmux files and
`go2w_hesai_fast_livo2.yaml`:

| Stream | Topic | Type |
|---|---|---|
| Hesai XT16 LiDAR | `/hesai_node/points_raw` | `sensor_msgs/msg/PointCloud2` |
| GO2-W IMU | `/go2w/imu` | `sensor_msgs/msg/Imu` |

## Direct Launch

Run FAST-LIVO2 with sensor bringup:

```bash
source humble_ws/install/setup.bash
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

Run the legacy Hesai XT16 + GO2-W IMU stack with the preserved catmux file:

```bash
catmux_create_session /external/catmux/online_fast_livo2_hesai.yaml
```

## Calibration Requirements

`humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` is the Livox MID360 default. It contains bootstrapping camera values and the initial MID360 LiDAR-to-IMU translation from `fastlio-go2w`. Replace or verify these before collecting meaningful odometry:

- `camera.cam_fx`, `camera.cam_fy`, `camera.cam_cx`, `camera.cam_cy`
- `camera.cam_d0` through `camera.cam_d4`
- `extrin_calib.extrinsic_T` currently `[-0.011, -0.02329, 0.04412]`
- `extrin_calib.extrinsic_R`
- `extrin_calib.Pcl` currently `[-0.089, 0.0, 0.1508]`
- `extrin_calib.Rcl`
- `time_offset.imu_time_offset`
- `time_offset.img_time_offset`
- `time_offset.lidar_time_offset`

Interpretation:

- `extrinsic_T` and `extrinsic_R`: Livox LiDAR to Livox IMU transform used by the LIO state.
- `Pcl` and `Rcl`: LiDAR to RealSense color camera transform used by visual projection.

The current `Pcl`/`Rcl` values are provisional. They are derived from a temporary
`base_link -> camera_color_optical_frame` offset of `[0.30, 0.0, 0.05, 0.0, 0.0,
0.0]` and the MID360 mount reference in `config/sensor/go2w_mid360_calibration.yaml`.

Do not use unverified camera intrinsics or camera extrinsics in the default config for evaluation or field operation.

## Useful Commands

Check launch arguments:

```bash
source humble_ws/install/setup.bash
ros2 launch fast_livo2 go2w_fast_livo2.launch.py --show-args
```

Check expected topics:

```bash
ros2 topic list
ros2 topic echo /livox/imu --once
ros2 topic hz /livox/lidar
ros2 topic hz /camera/camera/color/image_raw
```

If `/livox/lidar` or `/livox/imu` is missing, check the `livox_mid360` catmux
window. The expected launch path is `bash /external/scripts/run_livox_mid360_points.sh`.
The wrapper runs `livox_ros_driver2_node` directly and uses
`config/livox/MID360_config.json` by default. It also falls back to
`humble_ws/src/fast-livo2/config/MID360_config.json` if the installed
`fast_livo2` share directory is stale, and finally to the `livox_ros_driver2`
package config when the FAST-LIVO2 copy is missing.
If the MID360 host IP or LiDAR IP differs from the default, update
`config/livox/MID360_config.json` or run with
`LIVOX_MID360_CONFIG=/path/to/MID360_config.json`.

If the Livox driver prints `bind failed`, the MID360 config usually names a
host IP that is not assigned to the robot, or another Livox process is already
using the UDP ports. The wrapper checks both before launching the driver. To
inspect manually inside the robot container:

```bash
ip -o -4 addr show
ss -H -lunp | grep -E ':(56101|56201|56301|56401|56501)\b' || true
pkill -f livox_ros_driver2_node
```

If the FAST-LIVO2 MID360 config is missing after pulling the repository, update
the submodules before rebuilding:

```bash
git submodule update --init --recursive humble_ws/src/fast-livo2
```

If `colcon build` reports duplicate `livox_ros_driver2` packages, only one
driver source can remain visible to colcon. This commonly happens when both
`humble_ws/src/livox_ros_driver2` and
`humble_ws/src/ws_livox/src/livox_ros_driver2` exist. The normal build script
now ignores duplicates automatically. To fix an already-failed workspace
manually:

```bash
touch /external/humble_ws/src/ws_livox/src/livox_ros_driver2/COLCON_IGNORE
cd /external
bash scripts/build_humble_ws.sh
source humble_ws/install/setup.bash
ros2 pkg prefix livox_ros_driver2
```

Rebuild only FAST-LIVO2:

```bash
cd humble_ws
colcon build --symlink-install --packages-select fast_livo2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## License

This workspace contains a ROS 2 Humble port of FAST-LIVO2 plus integration with GO2-W sensor packages. Keep upstream FAST-LIVO2 attribution and licensing when redistributing `fast_livo2`, and keep third-party source licenses for `realsense-ros`, `librealsense`, Unitree ROS 2 interfaces, and the Hesai driver.
