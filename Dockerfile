# FROM git.ropod.org:4567/ropod/docker/ropod-base:kinetic-msgs-python3 AS ropod_msgs
FROM git.ropod.org:4567/ropod/docker/ropod-base:kinetic AS kinetic
FROM git.ropod.org:4567/ropod/ropod_common:latest

## Get MongoCXX driver, mongocxx and ftsm
WORKDIR /workspace
RUN wget https://github.com/mongodb/mongo-c-driver/releases/download/1.13.0/mongo-c-driver-1.13.0.tar.gz
RUN tar xzf mongo-c-driver-1.13.0.tar.gz
RUN curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.4.0.tar.gz
RUN tar -xzf r3.4.0.tar.gz
RUN git clone https://github.com/ropod-project/ftsm.git
RUN mv ftsm /opt/ropod/

## Install MongoCXX driver
## Instructions adapted from here: http://mongocxx.org/mongocxx-v3/installation/
# libmongoc
WORKDIR /workspace/mongo-c-driver-1.13.0/cmake-build
RUN cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
RUN make
RUN make install

# install mongocxx
WORKDIR /workspace/mongo-cxx-driver-r3.4.0/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
RUN make EP_mnmlstc_core
RUN make && make install

# WORKDIR /workspace
# RUN git clone https://github.com/ropod-project/ftsm.git
# RUN mv ftsm /opt/ropod/

WORKDIR /
COPY ros_entrypoint.sh /
# Setup ROS
COPY --from=kinetic / /

WORKDIR /ropod_com_mediator_ws
RUN catkin init \
    && catkin config --install --install-space /opt/ropod/ros/ \
    && catkin config --cmake-args -DPYTHON_VERSION=3.5 \
    && catkin config --extend /opt/ros/kinetic/

WORKDIR src/ropod_ros_msgs
RUN git clone https://github.com/ropod-project/ropod_ros_msgs.git

WORKDIR /ropod_com_mediator_ws/src/ropod_com_mediator
COPY . /ropod_com_mediator_ws/src/ropod_com_mediator/

WORKDIR /ropod_com_mediator_ws
RUN sh /opt/ros/kinetic/setup.sh && \
    apt update -qq && \
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    catkin build

ENTRYPOINT ["/ros_entrypoint.sh"]
