ARG UBUNTU_DISTRO=xenial
ARG ROS_DISTRO=kinetic

# Stage 1: ROS and basic tools
####################################################################################################################
FROM ros:$ROS_DISTRO-ros-base-$UBUNTU_DISTRO AS ros-builder
ARG UBUNTU_DISTRO
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# basic tools
RUN apt update && apt install --no-install-recommends -q -y \
    apt-utils lsb-release sudo unzip wget \
    git vim ssh gnupg2 lsb-release mlocate \
    net-tools iputils-ping curl ca-certificates\
    python-catkin-tools python-wstool mayavi2 \
    && rm -rf /var/lib/apt/lists/*

# OpenRAVE dependencies
RUN mkdir -p ~/git && cd ~/git    \
    && wget -q https://github.com/crigroup/openrave-installation/archive/0.9.0.zip -O openrave-installation.zip  \
    && unzip -q openrave-installation.zip -d ~/git    \
    && cd openrave-installation-0.9.0 && sudo ./install-dependencies.sh \
    && apt-get clean -qq \
    && rm -rf /var/lib/apt/lists/*


# Stage 2: Install OpenRAVE
####################################################################################################################
FROM ros-builder AS openrave-builder
ENV DEBIAN_FRONTEND noninteractive

# OpenSceneGraph
ENV OSG_COMMIT 1f89e6eb1087add6cd9c743ab07a5bce53b2f480
RUN mkdir -p /usr/src && cd /usr/src \
    && wget -q https://github.com/openscenegraph/OpenSceneGraph/archive/${OSG_COMMIT}.zip -O OpenSceneGraph.zip \
    && unzip -q OpenSceneGraph.zip -d /usr/src \
    && cd /usr/src/OpenSceneGraph-${OSG_COMMIT} && mkdir build && cd build \
    && cmake .. -DDESIRED_QT_VERSION=4 && make -j 4 && make install   \
    && rm -rf /usr/src/OpenSceneGraph-${OSG_COMMIT}/build

# FCL
RUN cd /usr/src \
    && wget -q https://github.com/flexible-collision-library/fcl/archive/0.5.0.zip -O fcl.zip \
    && unzip -q fcl.zip -d /usr/src \
    && cd /usr/src/fcl-0.5.0 && mkdir build && cd build \
    && cmake .. && make -j 4 && make install  \
    && rm -rf /usr/src/fcl-0.5.0/build

# OpenRAVE
ENV RAVE_COMMIT 7c5f5e27eec2b2ef10aa63fbc519a998c276f908
RUN pip install --upgrade sympy==0.7.1
RUN cd /usr/src \
    && wget -q https://github.com/rdiankov/openrave/archive/${RAVE_COMMIT}.zip -O openrave.zip \
    && unzip -q openrave.zip -d /usr/src \
    && cd /usr/src/openrave-${RAVE_COMMIT} && mkdir build && cd build \
    && cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ .. \
    && make -j 4 && make install  \
    && rm -rf /usr/src/openrave-${RAVE_COMMIT}/build


# Stage 3: ROS add-ons
####################################################################################################################
FROM openrave-builder AS final_build
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND noninteractive
ENV QT_X11_NO_MITSHM 1

RUN echo -e "\n[Installing essentials]"
RUN rm -rf /var/lib/apt/lists/* && apt-get update -q \
	&& apt-get install -qq -y --no-install-recommends \
	apt-utils python-catkin-tools ssh vim

RUN echo -e "\n[Installing base ROS dependencies]"
RUN apt-get install -qq -y --no-install-recommends \
	ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-transmission-interface \
	python-termcolor ros-$ROS_DISTRO-diagnostic-updater \
	ros-$ROS_DISTRO-rqt-runtime-monitor ros-$ROS_DISTRO-position-controllers \
	ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-baldor \
	ros-$ROS_DISTRO-control-msgs ros-$ROS_DISTRO-robot-state-publisher \
	ros-$ROS_DISTRO-bcap ros-$ROS_DISTRO-image-geometry \
	ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-diagnostic-aggregator \
	ros-$ROS_DISTRO-controller-manager-msgs ros-$ROS_DISTRO-realtime-tools \
	ros-$ROS_DISTRO-tf python-sklearn ros-$ROS_DISTRO-joint-state-controller \
	ros-$ROS_DISTRO-joint-trajectory-controller ros-$ROS_DISTRO-python-orocos-kdl \
	ros-$ROS_DISTRO-tf-conversions \
	ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-joint-limits-interface \
	ros-$ROS_DISTRO-joint-state-publisher \
	python-tabulate ros-$ROS_DISTRO-rqt-robot-monitor\
	ros-$ROS_DISTRO-effort-controllers

RUN echo -e "\n[Installing image-related ROS dependencies]"
RUN apt-get install -qq -y --no-install-recommends \
	ros-$ROS_DISTRO-camera-calibration-parsers ros-$ROS_DISTRO-cv-bridge\
	ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-view \
	ros-$ROS_DISTRO-camera-calibration ros-$ROS_DISTRO-image-proc ros-$ROS_DISTRO-roslint

# END