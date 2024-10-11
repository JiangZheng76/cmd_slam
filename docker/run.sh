echo "cmd_slam"
CATKIN_WS=/root/cmd_ws
cd ~
mkdir $CATKIN_WS/src
mv /cmd_slam $CATKIN_WS/src
cd $CATKIN_WS/src/cmd_slam

sh install.zsh
# tail -f /dev/null