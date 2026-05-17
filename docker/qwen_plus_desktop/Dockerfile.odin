FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-tf2-tools \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace/src/odin_rescue

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
      --packages-select \
      odin_ai \
      odin_bringup \
      odin_coordinator \
      odin_detection \
      odin_exploration \
      odin_gazebo \
      odin_interfaces \
      odin_map_merge \
      odin_navigation \
      odin_slam

COPY docker/qwen_plus_desktop/scripts/ /opt/odin/scripts/
RUN chmod +x /opt/odin/scripts/*.sh

WORKDIR /workspace
ENTRYPOINT ["/opt/odin/scripts/entrypoint.sh"]
CMD ["/opt/odin/scripts/run_odin_full.sh"]
