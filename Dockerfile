FROM osrf/ros:jazzy-desktop-full

RUN mkdir -p /igvc/igvc_ws/src
RUN mkdir -p /igvc/setup
RUN mkdir -p /igvc/deps

COPY igvc_ws/src /igvc/igvc_ws/src
# COPY setup /igvc/setup
# COPY vectorsecrets.txt /igvc/setup/vectorsecrets.txt
# COPY deps /igvc/deps

WORKDIR /igvc/setup
RUN /bin/bash -c "./setup.sh"

WORKDIR /igvc/igvc_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build"