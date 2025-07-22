FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# --- Dependências de ROS e do sistema ---
RUN apt-get update && apt-get install -y \
    ros-noetic-rqt-image-view \
    ros-noetic-usb-cam \
    ros-noetic-rviz \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-topic \
    python3-rosdep \
    python3-catkin-tools \
    python3-pip \
    git \
    build-essential \
    ros-noetic-fiducial-msgs \ 
    ros-noetic-tf \
    && rosdep update

# --- Cria o workspace catkin ---
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src

# --- Clona o aruco_ros ---
RUN git clone https://github.com/pal-robotics/aruco_ros.git && \
    cd aruco_ros && git checkout noetic-devel

# --- Cria o pacote aruco_bridge (para o script Python) ---
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /ros_ws/src && \
    catkin_create_pkg aruco_bridge rospy fiducial_msgs tf geometry_msgs std_msgs"

# --- Copia script python para o pacote ---
COPY aruco_to_fiducial_publisher.py /ros_ws/src/aruco_bridge/scripts/aruco_to_fiducial_publisher.py

# --- Permissão de execução ---
RUN chmod +x /ros_ws/src/aruco_bridge/scripts/aruco_to_fiducial_publisher.py

# --- Copia arquivos de calibração e launch ---
COPY camera_calibration/my_webcam.yaml /ros_ws/calibrations/my_webcam.yaml
COPY launch /ros_ws/launch

# --- Instala dependências e compila ---
WORKDIR /ros_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# --- Sourcing automático ---
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /ros_ws
CMD ["bash"]

