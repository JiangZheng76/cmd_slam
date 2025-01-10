# 定义一个函数
create_build_dir() {
    if [ -d "build" ]; then
        echo "存在名为 'build' 的文件夹。"
    else 
        mkdir build
        echo "创建了名为 'build' 的文件夹。"
    fi
}
WS=/root/cmd_ws
echo $WS

# 更新 cmake 
cd $WS/src
cd cmake-3.22.0
./bootstrap


# wstool
cd $WS/src
wstool init
wstool merge -t . cmd_slam/dependencies.rosinstall
wstool up

# Sophus 
cd $WS/src/Sophus
create_build_dir && cd build  && cmake .. && make -j && sudo make install 

# pcl 
cd $WS/src/pcl
git checkout pcl-1.13.0
create_build_dir && cd build  && cmake .. && make -j && sudo make install

# 安装 glog
cd $WS/src/glog
git checkout v0.5.0
create_build_dir && cd build  && cmake .. && make -j && sudo make install


# 安装 ceres 
cd $WS/src/ceres-solver
git checkout 1.14.0
create_build_dir && cd build  && cmake .. && make -j && sudo make install

# pangolin
cd $WS/src/pangolin
create_build_dir && cd build  && cmake .. && make -j && sudo make install

# mysylar
cd $WS/src/cmd_slam/cmd_comm/thrid-party/mysylar
create_build_dir && cd build  && cmake .. && make -j

