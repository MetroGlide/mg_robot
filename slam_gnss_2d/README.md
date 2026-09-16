# slam_gnss_2d

2D LiDAR + Odometry + GNSS constraints SLAM implementation written in C++.

## Features

- **Point-to-Line ICP with Huber / Cauchy robust kernels**
- **2D Multi-resolution NDT (Normal Distributions Transform)**
- **Correlative Scan Matching (CSM)**
- **Scan-to-Scan and Sliding-Window Local Map Reference Providers**
- **Loop Closure Detection & Submap Matching with streak fallbacks & crossing angle filtering**
- **Incremental Factor Graph Optimization via GTSAM iSAM2**
- **Batch Re-optimization via GTSAM Levenberg-Marquardt**
- **WGS84 ↔ UTM Projection & Initial Heading Estimation via GeographicLib**
- **High-speed Occupancy Grid Mapping (Overwrite & Hit/Miss Ratio Counting Renderers)**
- **Standalone Docker & Makefile workflow**

## Packages

- `slam_gnss_2d_msgs`: Custom message and service definitions (`PoseGraphDiff`, `GetPoseGraph`)
- `slam_gnss_2d`: C++ SLAM core library, executables, and launch files

## Quick Start (Docker)

```bash
# Build dev container
make build

# Run unit tests
make test

# Run offline processing on rosbag2
make offline BAG=/path/to/bag
```
