FROM ros:noetic-ros-base

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies: usb_cam, rviz, rqt_graph, build tools
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt-image-view \
    ros-noetic-usb-cam \
    ros-noetic-rviz \
    ros-noetic-rqt-graph \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-catkin-tools \
    python3-pip \
    git \
    build-essential && \
    rosdep update

# Create workspace
RUN mkdir -p /ros_ws/src

WORKDIR /ros_ws/src

# Copy launch file
COPY launch /ros_ws/launch

# Clone aruco_ros and checkout the correct branch for ROS Noetic
RUN git clone https://github.com/pal-robotics/aruco_ros.git && \
    cd aruco_ros && \
    git checkout noetic-devel

# Go back to workspace root and install ROS dependencies
WORKDIR /ros_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Copy your calibration YAML file into the container
# Assuming your build context has a folder: ./camera_calibration/my_webcam.yaml
COPY camera_calibration/my_webcam.yaml /ros_ws/calibrations/my_webcam.yaml

# Source workspace setup in all container shells
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /ros_ws

CMD ["bash"]
