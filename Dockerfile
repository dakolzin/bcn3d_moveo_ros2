FROM osrf/ros:humble-desktop-full

EXPOSE 8500/udp 

# install ros package
RUN apt-get update && apt-get install -y \
      build-essential \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-rviz-visual-tools \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

COPY . /ros2_bcn3d_moveo/src/
WORKDIR /ros2_bcn3d_moveo

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash; colcon build'
RUN echo ". install/setup.bash" >> ~/.bashrc


WORKDIR /ros2_bcn3d_moveo/src

ENTRYPOINT [ "/bin/bash","./runlaunch.sh" ]
