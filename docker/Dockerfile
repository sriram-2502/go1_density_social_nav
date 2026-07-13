# Reconstructed Jetson perception image for Go1/ZED2i experiments.
# Build on Jetson AGX Xavier with JetPack 5.1.2 / L4T R35.4.1.
FROM stereolabs/zed:5.1.2-tools-devel-jetson-jp5.1.2

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]
WORKDIR /home

# The original object_detection image had manual layers and no recovered source
# Dockerfile. This installs the ROS Noetic pieces needed by camera_node.py and
# keeps the custom experiment nodes in /home, matching the lab run command.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    && if [ ! -f /etc/apt/sources.list.d/ros1-latest.list ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
      echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros1-latest.list; \
    fi && \
    apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-numpy \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-rosbag \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-vision-msgs \
    ros-noetic-tf \
    ros-noetic-catkin \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    && rm -rf /var/lib/apt/lists/*

# camera_node.py publishes autoware_msgs/DetectedObjectArray. The recovered
# container had autoware_ai_messages under /home/autoware/src.
ARG AUTOWARE_AI_MESSAGES_REPO=https://github.com/autowarefoundation/autoware_ai_messages.git
ARG AUTOWARE_AI_MESSAGES_REF=master
RUN mkdir -p /home/autoware/src && \
    cd /home/autoware/src && \
    git clone --depth 1 --branch "${AUTOWARE_AI_MESSAGES_REF}" "${AUTOWARE_AI_MESSAGES_REPO}" autoware_ai_messages && \
    ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /home/autoware/src/CMakeLists.txt && \
    source /opt/ros/noetic/setup.bash && \
    cd /home/autoware && \
    catkin_make

COPY jetson_nodes/ /home/
RUN mkdir -p /home/bags && \
    chmod +x /home/*.py && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /home/autoware/devel/setup.bash' >> /root/.bashrc

ENV ROS_DISTRO=noetic
ENV ROS_PACKAGE_PATH=/home/autoware/src:/opt/ros/noetic/share
ENV PYTHONPATH=/home/autoware/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
ENV LD_LIBRARY_PATH=/home/autoware/devel/lib:/opt/ros/noetic/lib:/usr/local/cuda/lib64

CMD ["/bin/bash"]
