# syntax=docker/dockerfile:1
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS dev

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    ros-${ROS_DISTRO}-gtsam \
    libnanoflann-dev \
    libgeographic-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    ros-${ROS_DISTRO}-ublox-msgs \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_data && ln -s /root/ros2_data /data

WORKDIR /app
COPY . /app

WORKDIR /root/ros2_ws/src
RUN ln -s /app/slam_gnss_2d ./slam_gnss_2d && \
    ln -s /app/slam_gnss_2d_msgs ./slam_gnss_2d_msgs

WORKDIR /root/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
  && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

FROM dev AS runtime
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "slam_gnss_2d", "bringup_slam_gnss_2d.launch.py"]
