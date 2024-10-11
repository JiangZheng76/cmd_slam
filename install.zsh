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
# cd $CMD_WS/src/fmt
# mkdir build 
# cd build 
# cmake ..
# make -j
# sudo make install 

# 安装 Sophus
cd $CMD_WS/src/Sophus
git checkout 1.22.10
mkdir build 
cd build 
cmake ..
make -j
sudo make install 

# 安装 pcl
cd $CMD_WS/src/pcl
git checkout pcl-1.13.0
mkdir build 
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
cd $CMD_WS/src/cmd_slam
cd cmd_comm/thrid-party/mysylar
mkdir build
cd build
cmake ..
make -j

cd $CMD_WS/src
source /opt/ros/melodic/setup.bash
catkin build cmd_comm
catkin build cmd_backend