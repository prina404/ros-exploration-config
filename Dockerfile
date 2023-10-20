# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential nano \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
   	python3-catkin-tools \
   	python3-pip \
   	python3-rospy \
    apt-utils \
    xvfb wmctrl x11-utils &&\
    rm -rf /var/lib/apt/lists/* && \
    /bin/bash -c 'pip3 install matplotlib opencv-python'

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-gmapping \
    ros-noetic-stage-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
    
RUN mkdir -p /root/catkin_ws/src
COPY src /root/catkin_ws/src

RUN	echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
	/bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make -DCMAKE_BUILD_TYPE=Release;' && \
	echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
    

ENV DISPLAY=:1
ENV screen=${XVFB_SCREEN:-0}
ENV resolution=${XVFB_RESOLUTION:-800x600x24}
RUN echo "Xvfb ${DISPLAY} -screen ${screen} ${resolution} &" >> /root/.bashrc

COPY launch.sh /root/
WORKDIR /root/catkin_ws/src/my_navigation_configs/
ENTRYPOINT ["bash", "-i", "/root/launch.sh"]



