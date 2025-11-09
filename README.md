# Doppler ICP Stitcher (v0.0)

[![C++](https://img.shields.io/badge/lang-C%2B%2B-00599C)]() [![ROS2](https://img.shields.io/badge/ros2-supported-blue)]() [![License](https://img.shields.io/badge/license-TODO-lightgrey)]()

A ROS 2 C++ node that fuses Doppler radial velocities with geometric ICP to produce robust stitched point clouds from sequential LiDAR frames. This repository provides the node, configuration examples, a launch file, and an optional Foxglove layout for visualization.

This document is written for the client and maintainers: it highlights purpose, system requirements, quick start commands, configuration guidance, topics and troubleshooting tips.

---

## Table of contents

- [Project overview](#project-overview)  
- [Key features](#key-features)  
- [Requirements](#requirements)  
- [Quick start (build & run)](#quick-start-build--run)  
- [Configuration & parameters](#configuration--parameters)  
- [ROS interfaces (topics / services)](#ros-interfaces-topics--services)  
- [Visualization - Foxglove](#visualization-foxglove) 
- [Troubleshooting & tips](#troubleshooting--tips)  
- [Development & contribution guidelines](#development--contribution-guidelines)  
- [License & maintainers](#license--maintainers)  
- [Contact & support](#contact--support)

---

## Project overview

Doppler ICP Stitcher combines measured radial velocities (Doppler) with point-to-plane ICP to improve registration of sequential LiDAR frames, especially for moving-object robustness. It is implemented as a ROS 2 node (ament_cmake, C++17) and exposes parameters for runtime tuning, optional CUDA acceleration, and a Foxglove layout for quick visualization.

The primary executable: `stitch_node` (configured in CMakeLists).

---

## Key features

- Doppler-aware ICP: integrates velocity residuals with geometric point-to-plane ICP.  
- Temporal persistence filter: removes transient objects via voxel persistence.  
- Optional CUDA acceleration for nearest neighbors / linear algebra.  
- Asynchronous preprocessing pipeline to overlap I/O and compute.  
- ROS 2 friendly: parameters, launch files, and publish/subscribe topics.  
- Foxglove Studio layout provided for fast visualization.

---

## Requirements

Supported base system:
- Ubuntu 20.04 or 22.04 (choose ROS 2 distro accordingly: Humble for 20.04, Humble/Iron/rolling for 22.04).  
- ROS 2 (Humble or newer recommended).  
- C++17-compatible compiler (GCC or Clang).  
- colcon build tool.  
- Open3D (C++), Eigen3.  
- NVIDIA GPU + CUDA toolkit (optional; only if enabling GPU acceleration).  
- Python 3 for helper scripts and Foxglove bridge.

Recommended minimum packages (Ubuntu examples):
```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip python3-venv libeigen3-dev pkg-config
python3 -m pip install -U colcon-common-extensions setuptools
```

Open3D:
- Prefer system package `libopen3d-dev` if available or build Open3D from source and point CMake to `Open3D_DIR`.

---

## Quick start (build & run)

1) Clone into a ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/LinaSaii/DICP_V0.0.git
cd ~/ros2_ws
```

2) Source ROS 2 and build:
```bash
source /opt/ros/<ros-distro>/setup.bash
# If Open3D is in a custom location:
# export Open3D_DIR=/path/to/Open3D/lib/cmake/Open3D
colcon build --packages-select doppler_icp_stitcher_open3d_pro2
source install/setup.bash
```

3) Launch the node (recommended):
```bash
ros2 launch doppler_icp_stitcher_open3d_pro2 stitcher_launch.py
```

Or run executable directly:
```bash
ros2 run doppler_icp_stitcher_open3d_pro2 stitch_node
```

If you use CUDA, ensure the CUDA toolkit and drivers are installed and `CUDA_ARCHITECTURES` is set in `CMakeLists.txt` for your GPU.

---

## Configuration & parameters

Parameters are loaded from `config/param.yaml` by the example launch file. Key parameters (example names and descriptions):

- input_mode: "live" | "csv" : choose between live ROS topics or batch CSV frames.  
- frames_directory: path to CSV frames when using CSV mode.  
- input_topic: ROS topic for input point clouds (sensor_msgs/PointCloud2).  
- output_topic: topic for the stitched cloud (default: `/stitch/merged_cloud`).  
- frame_id: global map frame id (e.g., `map`).  
- use_cuda: true | false : enable CUDA optimized code paths.  
- voxel_size: float (meters) : voxel size for downsampling and persistence.  
- persistence_voxel_size: float (meters) : voxel size for persistence filter.  
- min_persistence_frames: int : minimum frames to consider a voxel persistent.  
- max_icp_iterations: int : maximum ICP iterations.  
- max_corr_distance: float (meters) : maximum correspondence distance.  
- velocity_threshold: float (m/s) : threshold to classify moving points (using Doppler).  
- lambda_doppler_start / lambda_doppler_end : weighting schedule for Doppler residuals.

Example minimal `config/param.yaml` snippet:
```yaml
stitch_node:
  ros__parameters:
    input_mode: "live"
    input_topic: "/input/pointcloud"
    output_topic: "/stitch/merged_cloud"
    frame_id: "map"
    use_cuda: false
    voxel_size: 0.2
    persistence_voxel_size: 0.5
    min_persistence_frames: 3
    max_icp_iterations: 50
    max_corr_distance: 2.0
    velocity_threshold: 0.1
```

Document units clearly: distances in meters, time in seconds, velocities in meters/second.

---

## ROS interfaces (topics & services)

Subscribed:
- `/input/pointcloud` (sensor_msgs/PointCloud2) — incoming LiDAR frames.  
- TF (via tf2) — required frames for visualization and alignment.

Published:
- `/stitch/merged_cloud` (sensor_msgs/PointCloud2) — merged/stitched point cloud.  
- `/icp_pose` (geometry_msgs/PoseStamped) — current estimated pose from ICP.  
- `/icp_trajectory` (nav_msgs/Path) — accumulated trajectory.

Parameters are exposed via the ROS 2 parameter API. Consider adding a service to save the current map or toggle recording.

---

## Visualization (Foxglove Studio)

A recommended Foxglove layout is provided in `foxglove/foxglove_layout.json`.

To visualize:
1. Run the node:
```bash
ros2 launch doppler_icp_stitcher_open3d_pro2 stitcher_launch.py
```
2. Start Foxglove bridge:
```bash
ros2 run foxglove_bridge foxglove_bridge
```
3. Open Foxglove Studio, connect to the bridge WebSocket (commonly `ws://localhost:8765`), import `foxglove/foxglove_layout.json`, and map:
- 3D panel → `/stitch/merged_cloud` (sensor_msgs/PointCloud2)  
- TF viewer → `/tf`, `/tf_static`  
- Pose → `/icp_pose`  
- Trajectory → `/icp_trajectory`

Ensure `header.frame_id` is broadcast and TF is available; mismatched frames or missing TF will misplace clouds.

---

## Troubleshooting & tips

- CMake cannot find Open3D: set `Open3D_DIR` to the Open3D installation CMake config directory or install the system package.  
- CUDA errors: check CUDA toolkit / driver compatibility; ensure `nvcc` is on PATH and `CUDA_ARCHITECTURES` is set.  
- Foxglove shows no topics: verify `foxglove_bridge` is running and the WebSocket connection is established.  
- Misaligned clouds: verify TF connectivity, `header.frame_id`, and message timestamps.  

Testing:
- Use `ros2 topic list` and `ros2 topic echo` for quick checks.  
- Replay bags with `ros2 bag play` for reproducible tests.  
- Add GTest unit tests for algorithmic components (ICP residuals, filters).

---

## Development & contribution guidelines

- Follow C++17 and ROS 2 best practices (rclcpp, parameter usage).  
- Branch naming: `feature/<short-desc>` or `fix/<short-desc>`.  
- PR checklist: description, linked issue (if any), test plan, and expected outcomes.  
- Add unit tests for algorithmic changes and CI for build & test.

Suggested CI (GitHub Actions):
- Matrix: Ubuntu 20.04 and 22.04.  
- Steps: install dependencies, colcon build, run unit tests, run linters (ament_lint_auto).

---

## License & maintainers

- License: 
- Maintainer:  
- Repository owner: 

---

## Contact & support

For issues and contributions, please open GitHub Issues or Pull Requests in this repository. For urgent support, contact the maintainer .

---
