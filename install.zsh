cd /home/sysu/cmd_ws/src/cmd-slam/cmd_comm/thrid-party/mysylar/build 
cmake .. 
make -j8 
cd /home/sysu/cmd_ws/src/cmd-slam/cmd_backend/thrid-party/Kimera-RPGO/build
cmake ..
make -j10

cd /home/sysu/cmd_ws/src/cmd-slam
catkin build cmd_comm 
catkin build cmd_backend
# catkin build cmd_frontend
# catkin build direct_stereo_slam
source /home/sysu/cmd_ws/devel/setup.zsh