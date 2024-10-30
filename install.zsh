# 获取当前的工作路径
CMD_WS=$(pwd)/../../
echo $CMD_WS
cd $CMD_WS

# 安装 worksapce 依赖
cd $CMD_WS/src
wstool init
wstool merge -t . cmd_slam/dependencies.rosinstall
wstool up

# Sophus 依赖于 fmt
sudo apt install libfmt-dev

# 安装 catkin build 
sudo apt install -y  python3-pip
sudo pip3 install -U catkin_tools

# 安装 Sophus
cd $CMD_WS/src/Sophus
git config --global --add safe.directory $CMD_WS/src/Sophus
git checkout 1.22.10
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
cd build 
cmake ..
make -j
sudo make install 

# pcl 依赖
sudo apt install libflann-dev
# 安装 pcl
cd $CMD_WS/src/pcl
git config --global --add safe.directory $CMD_WS/src/pcl
git checkout pcl-1.13.0
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
cd build 
cmake ..
make -j
sudo make install

# 安装 glog
cd $CMD_WS/src/glog
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
git config --global --add safe.directory $CMD_WS/src/glog
git checkout v0.5.0
cd build 
cmake ..
make -j
sudo make install

# 安装 ceres 
cd $CMD_WS/src/ceres-solver
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
git config --global --add safe.directory $CMD_WS/src/ceres-solver
git checkout 1.14.0
cd build 
cmake ..
make -j
sudo make install

# 拉取代码子模块
git config --global --add safe.directory $CMD_WS/src/cmd_slam/cmd_comm/thrid-party/mysylar
git config --global --add safe.directory $CMD_WS/src/cmd_slam/cmd_comm/thrid-party/cereal
git submodule update --init --recursive

cd $CMD_WS/src
chmod +x cmd_slam/fix_eigen_deps.sh
./cmd_slam/fix_eigen_deps.sh

# 处理 sylar 和 cereal
# 安装 boost 
sudo apt -y install libboost-all-dev
cd $CMD_WS/src/cmd_slam
cd cmd_comm/thrid-party/mysylar
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
cd build
cmake ..
make -j

# 安装 vtk
sudo apt -y install libvtk7-dev
sudo apt -y install libglew-dev
sudo apt -y install libgflags-dev
# 安装 pangolin 
cd $CMD_WS/src/pangolin
if [ -d "build" ]; then
    echo "存在名为 'build' 的文件夹。"
else 
    mkdir build 
fi
git config --global --add safe.directory $CMD_WS/src/pangolin
cd build
cmake ..
make -j
sudo make install


cd $CMD_WS/src
source /opt/ros/melodic/setup.bash
catkin build cmd_comm
catkin build cmd_backend