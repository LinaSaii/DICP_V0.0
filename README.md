```markdown
# Doppler ICP Stitcher (V0.0)

[![C++](https://img.shields.io/badge/lang-C%2B%2B-00599C)]() [![ROS2](https://img.shields.io/badge/ros2-supported-blue)]() [![License](https://img.shields.io/badge/license-TODO-lightgrey)](./LICENSE)

Professional, production-minded README for the Doppler ICP Stitcher project — a ROS 2/C++ node that performs point cloud stitching using Doppler information and ICP optimizations with Open3D and optional CUDA acceleration.

This README is organized with an interactive Table of Contents: click any entry to jump to that section.

---

Table of contents
- [Project overview](#project-overview)
- [Key features](#key-features)
- [Repository layout](#repository-layout)
- [Supported environment & prerequisites](#supported-environment--prerequisites)
- [Install system dependencies](#install-system-dependencies)
- [Clone the repository](#clone-the-repository)
- [Build the package (step-by-step)](#build-the-package-step-by-step)
- [Run the node (launch & direct)](#run-the-node-launch--direct)
- [Foxglove Studio visualization (step-by-step)](#foxglove-studio-visualization-step-by-step)
  - [Start foxglove_bridge](#start-foxglove_bridge)
  - [Import the provided Foxglove layout](#import-the-provided-foxglove-layout)
  - [Recommended panels & mapping](#recommended-panels--mapping)
- [Demo video & assets (how to include)](#demo-video--assets-how-to-include)
- [Configuration & parameters (config/param.yaml)](#configuration--parameters-configparamyaml)
- [Topics, services & interfaces](#topics-services--interfaces)
- [Testing, debugging & CI suggestions](#testing-debugging--ci-suggestions)
- [Troubleshooting & common errors](#troubleshooting--common-errors)
- [Development & contribution guidelines](#development--contribution-guidelines)
- [License & authors](#license--authors)
- [Contact & support](#contact--support)

---

Project overview
----------------
Doppler ICP Stitcher is a ROS 2 ament_cmake C++ package (C++17) that stitches sequential LiDAR frames by combining geometric ICP (point‑to‑plane) with measured Doppler radial velocities. It uses Open3D for point cloud processing, provides optional CUDA acceleration for heavy kernels, includes temporal persistence filtering to remove transient points, and publishes results via standard ROS 2 topics.

The package provides an executable named `stitch_node` (configured in `CMakeLists.txt`).

Key features
------------
- Doppler‑aware ICP: integrates velocity residuals with geometric point‑to‑plane ICP.
- Temporal persistence filter: voxel-based frame persistence to remove transient/moving objects.
- CUDA acceleration: optional GPU kernels for nearest neighbors / linear algebra.
- Asynchronous preprocessing pipeline to overlap IO and compute.
- ROS 2 integration: publishes merged clouds, poses, trajectory; supports MCAP/rosbag2 recording.
- Foxglove Studio layout support for quick visualization.

Repository layout
-----------------
- CMakeLists.txt — build definitions and targets
- package.xml — ROS 2 package metadata and dependencies
- include/doppler_icp_stitcher_open3d_pro2/ — public headers (API)
- src/ — C++ implementation
  - src/main.cpp
  - src/doppler_icp_stitcher.cpp
  - src/temporal_persistence_filter.cpp
  - src/async_preprocessor.cpp
  - src/utils.cpp
  - src/cuda_acceleration.cu / src/cuda_acceleration.cpp
- config/param.yaml — runtime parameters
- launch/stitcher_launch.py — example ROS 2 launch file
- foxglove/ — optional Foxglove Studio layout(s) (recommended: foxglove_layout.json)
- README.md — this file

---

Supported environment & prerequisites
------------------------------------
Recommended base system:
- Ubuntu 20.04 / 22.04 (20.04 works with ROS 2 Humble; 22.04 works with Humble/Ion/most newer ROS 2 distros). Use the ROS 2 distribution that matches your environment.
- NVIDIA GPU + drivers (if using CUDA acceleration)

Minimum toolchain:
- ROS 2 (Humble or Iron recommended) — install following official ROS 2 docs for your distro.
- colcon build tool (colcon-core)
- CMake >= 3.8
- GCC or Clang with C++17 support
- CUDA Toolkit (only if enabling CUDA build)
- Open3D (C++ library) — required for the point cloud pipeline
- Eigen3 (libeigen3-dev)
- Python 3 (for helper scripts and Foxglove bridge)

Notes on Open3D:
- For C++ usage, prefer installing the system package (if available) or building Open3D from source and pointing CMake to Open3D's install location using `-DOpen3D_DIR=/path/to/Open3D`.

---

Install system dependencies
---------------------------
The commands below are examples for Ubuntu. Adjust for your distribution.

1) Update and install common build deps:
```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip python3-venv \
                    libeigen3-dev pkg-config
```

2) ROS 2 (follow official install instructions for your ROS 2 distro). Example for ROS 2 Humble:
- Follow: https://docs.ros.org/en/humble/Installation.html

3) Install colcon and Python helpers:
```bash
python3 -m pip install -U colcon-common-extensions setuptools
```

4) Open3D:
- Option A (if system package available):
  ```bash
  sudo apt install -y libopen3d-dev   # may exist on some distros
  ```
- Option B (recommended if no system package): install from source or use a packaged binary and set Open3D_DIR.
  See Open3D: https://www.open3d.org (or https://github.com/intel-isl/Open3D)

5) CUDA (optional):
- Install the CUDA Toolkit from NVIDIA if you want GPU acceleration.
- Ensure `nvcc` and libraries are on PATH/LD_LIBRARY_PATH.
- Example: follow https://developer.nvidia.com/cuda-downloads

6) Foxglove Bridge & Studio:
- Foxglove Studio (desktop or web) — download from https://foxglove.dev/studio
- foxglove_ros2_bridge may be available as a ROS 2 package or built from source:
  - Try installing via apt (if maintained for your distro) or build from source.
  - Example run command: `ros2 run foxglove_bridge foxglove_bridge` (the bridge package must be installed in your ROS 2 environment).

---

Clone the repository
--------------------
Clone into a ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/LinaSaii/-v0.nokpis.git
cd ~/ros2_ws
```

If you prefer SSH:
```bash
git clone git@github.com:LinaSaii/-v0.nokpis.git
```

---

Build the package (step-by-step)
--------------------------------
1) Ensure system deps and ROS 2 are installed and sourced:
```bash
source /opt/ros/<ros-distro>/setup.bash
```

2) If Open3D is installed to a non-standard location, export or pass `Open3D_DIR` to CMake:
```bash
export Open3D_DIR=/path/to/Open3D/lib/cmake/Open3D
```

3) Build with colcon:
```bash
cd ~/ros2_ws
colcon build --packages-select doppler_icp_stitcher_open3d_pro2
```

4) Source the overlay:
```bash
source install/setup.bash
```

Build hints:
- If CMake fails with "Open3D not found", either install Open3D system package or build Open3D and set `Open3D_DIR`.
- If building CUDA code, ensure CUDA Toolkit and drivers are correctly installed and that `CUDA_ARCHITECTURES` in `CMakeLists.txt` matches your GPU.

---

Run the node (launch & direct)
------------------------------
Recommended: use the provided launch file:
```bash
ros2 launch doppler_icp_stitcher_open3d_pro2 stitcher_launch.py
```

Run directly (less recommended for full parameter loading):
```bash
ros2 run doppler_icp_stitcher_open3d_pro2 stitch_node
```

Example full workflow (build + start bridge + launch):
1. Open terminal A (source environment):
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch doppler_icp_stitcher_open3d_pro2 stitcher_launch.py
```
2. Open terminal B (start Foxglove bridge, see [Foxglove section](#foxglove-studio-visualization-step-by-step)):
```bash
ros2 run foxglove_bridge foxglove_bridge
```

---

Foxglove Studio visualization (step-by-step)
--------------------------------------------
This repo supports visualization through Foxglove Studio. Place a JSON layout under `foxglove/foxglove_layout.json` (recommended) and import it.

Start foxglove_bridge
- Launch the bridge:
  ```bash
  ros2 run foxglove_bridge foxglove_bridge
  ```
- The bridge prints the WebSocket URL (commonly `ws://localhost:8765`). Copy that URL.

Connect Foxglove Studio
- Open Foxglove Studio (desktop or web).
- Add a new connection using the WebSocket address output by the bridge.
- After connecting, you should see ROS 2 topics available.

Import the provided Foxglove layout
- File (example): `foxglove/foxglove_layout.json`
- In Foxglove Studio: Layouts → Import layout → select `foxglove/foxglove_layout.json`.
- The layout should configure:
  - 3D panel for `sensor_msgs/PointCloud2` (topic: `/stitch/merged_cloud` or whichever `output_topic` you set)
  - TF panel (for `/tf` and `/tf_static`)
  - Time-series plots for IMU channels (angular velocity, linear acceleration) if IMU topics are present
  - Diagnostic panels and logs

Recommended panels & mapping
- 3D point cloud: `/stitch/merged_cloud` (sensor_msgs/PointCloud2)
- TF viewer: `/tf`, `/tf_static`
- Pose topic: `/icp_pose` (geometry_msgs/PoseStamped)
- Trajectory: `/icp_trajectory` (nav_msgs/Path or CSV export)
- IMU channels (if present): `/imu` (sensor_msgs/Imu) → plot angular_velocity.x/y/z and linear_acceleration.x/y/z

Tips:
- Ensure PointCloud2 `header.frame_id` exists in the TF tree and is broadcasted by the node or another node.
- If clouds look misaligned in Foxglove, confirm TF connectivity and timestamps.

---

Demo video & assets (how to include)
-----------------------------------
Recommended: host a demo on YouTube (unlisted option) and link it in README for easy sharing. For inline preview in README, add a thumbnail in `assets/demo_thumb.png` and link:

```markdown
[![Demo video](assets/demo_thumb.png)](https://youtu.be/<your_video_id>)
```

Alternatives:
- Add a small animated GIF (keep it small).
- Attach the video as a release asset on GitHub (mind size limits).

Suggested video contents:
- Building the workspace
- Running `stitch_node`
- Starting Foxglove bridge and loading layout
- Visualizing a rosbag playback showing stitched cloud and IMU plots

---

Configuration & parameters (config/param.yaml)
----------------------------------------------
Edit `config/param.yaml` to tune runtime behaviour (the node loads this file via the launch file). Example parameters to include and tune:

- input_mode: "live" | "csv"        # choose live ROS topics or CSV frames directory
- frames_directory: /path/to/csvs   # CSV frames (x,y,z,radial_velocity) for batch mode
- input_topic: /input/pointcloud
- output_topic: /stitch/merged_cloud
- frame_id: map
- use_cuda: true | false
- voxel_size: 0.2
- downsample: true
- max_icp_iterations: 50
- max_corr_distance: 2.0
- velocity_threshold: 0.1
- lambda_doppler_start: 0.5
- lambda_doppler_end: 0.05
- lambda_schedule_iters: 10
- persistence_voxel_size: 0.5
- min_persistence_frames: 3
- enable_recording: true
- recording_directory: /path/to/recordings

Make sure to set sensible defaults in the file and document units (meters, seconds).

---

Topics, services & interfaces
-----------------------------
Suggested topic conventions used by the node:

Subscribed:
- `/input/pointcloud` (sensor_msgs/PointCloud2) — or CSV frames input when `input_mode: csv`
- TF (via tf2)

Published:
- `/stitch/merged_cloud` (sensor_msgs/PointCloud2) — final stitched cloud
- `/icp_pose` (geometry_msgs/PoseStamped) — current estimated pose
- `/icp_trajectory` (nav_msgs/Path) — accumulated trajectory or saved to CSV
- `/stitch/info` (std_msgs/String) — status / diagnostics

Services / params:
- The node should expose parameters from `config/param.yaml` via ROS 2 parameter interface.
- If helpful, add a service to save the current map or to toggle recording.

Document any custom messages or services here if you add them.

---

Testing, debugging & CI suggestions
----------------------------------
- Use `ros2 topic list`, `ros2 topic echo` and Foxglove Studio to monitor data.
- Use `ros2 bag play <bagfile>` to replay recorded inputs.
- Create unit tests with GTest for core algorithm pieces (ICP residuals, persistence filter).
- Add GitHub Actions to run linters and unit tests on PRs:
  - ament_lint, colcon build test, and unit test invocations.
- Instrument critical sections with informative logging (rclcpp logging) instead of printf.

---

Troubleshooting & common errors
-------------------------------
1. CMake cannot find Open3D:
   - Install a system `libopen3d-dev` package if available or build Open3D from source and set `Open3D_DIR` to the Open3D CMake config folder.
   - Example: `cmake -DOpen3D_DIR=/opt/Open3D/lib/cmake/Open3D ..`

2. CUDA compile/link errors:
   - Verify CUDA Toolkit version is compatible with your driver.
   - Ensure `nvcc` is on PATH and `CUDA_HOME`/`CUDA_TOOLKIT_ROOT_DIR` are set if needed.
   - Adjust `CUDA_ARCHITECTURES` in `CMakeLists.txt` to match your card (e.g., "75", "80", "86").

3. Foxglove layout shows no topics:
   - Confirm foxglove_bridge is running and connected.
   - Confirm topic names match layout; fix layout JSON or topic remapping.
   - Check firewall blocking WebSocket port.

4. Point cloud appears in wrong location:
   - Verify `header.frame_id` of published PointCloud2 and that a TF is published from that frame to `frame_id` set in params (e.g., `map`).
   - Check message timestamps and TF timesync.

5. Performance / memory:
   - Tune voxel_size and persistence settings; use sliding window to manage memory.
   - Enable CUDA if available and validated.

---

Development & contribution guidelines
------------------------------------
- Use C++17 and follow ROS 2 node/parameter best practices.
- Create a branch per change: `feature/<short-desc>` or `fix/<short-desc>`.
- Open PRs with:
  - Clear description of the change
  - Reference issue (if any)
  - Test plan / instructions and expected outputs
- Add unit tests for algorithmic changes and integration tests for launch behaviors.
- Keep `README.md`, `config/param.yaml`, and `launch/` files up to date.

Suggested CI:
- GitHub Actions job matrix:
  - Build on Ubuntu 20.04 and 22.04
  - Run `colcon build` and unit tests
  - Linting (ament_lint_auto)

---

License & authors
-----------------
- License: 
- Maintainer: 
- Repository owner: 

---

Contact & support
-----------------
For issues and contributions, open an issue or PR:
h