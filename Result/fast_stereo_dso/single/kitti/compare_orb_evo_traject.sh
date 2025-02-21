#! /bin/bash
export PATH="/home/sysu/.local/bin:$PATH"
echo "角度绝对 rmse"
seq="08"
cp /home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti/${seq}/Traject.txt '/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti/'${seq}'/'${seq}'_Ours.txt'
cp /home/sysu/cmd_ws/src/cmd-slam/Result/orb_slam2/single/kitti/${seq}/Traject.txt '/home/sysu/cmd_ws/src/cmd-slam/Result/orb_slam2/single/kitti/'${seq}'/'${seq}'_orbslam2.txt'
evo_traj tum --ref='/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/kitti/'${seq}'.txt'\
     '/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti/'${seq}'/'${seq}'_Ours.txt'  \
     '/home/sysu/cmd_ws/src/cmd-slam/Result/orb_slam2/single/kitti/'${seq}'/'${seq}'_orbslam2.txt' --plot --plot_mode xz -va