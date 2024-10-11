FROM osrf/ros:melodic-desktop-bionic
ENV CATKIN_WS=/root/cmd_ws

# install ros packages
RUN apt-get update && apt-get install -y \
                      wget \
                      ros-melodic-desktop-full \
                      ros-melodic-serial \
                      ros-melodic-catkin-simple \
					#   ros-melodic-pcl-ros\
                      software-properties-common\
                      python3-catkin-tools \
                      python3-wstool \
                      vim \
                      sudo \
					  net-tools\
                      autoconf \
                      mesa-utils\
	        	      gedit \
                      gdb \
                      cgdb \
                      && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y

# -------------------------------------------
# Libraries for building COVINS
# -------------------------------------------
RUN apt-get update && apt-get install -y \
                      cmake \
					  doxygen \
                      libgoogle-glog-dev \
                      libatlas-base-dev \
                      libsuitesparse-dev \
					  libyaml-cpp-dev \
					  libvtk6-dev \
					  libv4l-dev \
                      libomp-dev \
                      libglew-dev

COPY run.sh /run.sh
COPY ../../cmd_slam /cmd_slam
# 赋予 run.sh 文件执行权限
RUN chmod +x /run.sh

# 运行 run.sh 脚本
RUN /run.sh