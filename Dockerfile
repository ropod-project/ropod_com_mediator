FROM ropod/ropod_common:latest

## Install MongoCXX driver, mongocxx and ftsm
WORKDIR /workspace
RUN wget https://github.com/mongodb/mongo-c-driver/releases/download/1.13.0/mongo-c-driver-1.13.0.tar.gz \
    && tar xzf mongo-c-driver-1.13.0.tar.gz \
    && cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF mongo-c-driver-1.13.0/ \
    && make \
    && make install \
    && rm -rf * \
    && curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.4.0.tar.gz \
    && tar -xzf r3.4.0.tar.gz \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local mongo-cxx-driver-r3.4.0/ \
    && make EP_mnmlstc_core \
    && make \
    && make install \
    && rm -rf * \
    && git clone https://github.com/ropod-project/ftsm.git \
    && mv ftsm /opt/ropod/ \
    && rm -rf *

# Install bare-bones ROS
WORKDIR /
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-kinetic-ros-base \
    && rosdep init \
    && rosdep update \
    && apt install -y --no-install-recommends python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools \
    && pip install catkin_pkg empy \
    && rm -rf /var/lib/apt/lists/*

# Setup a catkin workspace
WORKDIR /ropod_com_mediator_ws
RUN catkin init \
    #&& catkin config --install --install-space /opt/ropod/ros/ \
    && catkin config --cmake-args -DPYTHON_VERSION=3.5 \
    && catkin config --extend /opt/ros/kinetic/

# Clone the ropod_ros_msgs repository
WORKDIR src/ropod_ros_msgs
RUN git clone https://github.com/ropod-project/ropod_ros_msgs.git

# Copy the com_mediator source code
WORKDIR /ropod_com_mediator_ws/src/ropod_com_mediator
COPY . /ropod_com_mediator_ws/src/ropod_com_mediator/

# Build the catkin workspace
WORKDIR /ropod_com_mediator_ws
RUN sh /opt/ros/kinetic/setup.sh && \
    apt update -qq && \
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build && \
    echo "source /ropod_com_mediator_ws/devel/setup.bash" >> ~/.bashrc

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
