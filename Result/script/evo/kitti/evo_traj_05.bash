
seqnum="05"
vo="cmd_stereo_dso"
export PATH="/home/sysu/.local/bin:$PATH"
cd /home/sysu/cmd_ws/src/cmd-slam/Result/$vo/kitti/$seqnum/tum/
evo_traj tum --ref=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/kitti_tum/$seqnum/groundtruth.txt \
    1.txt 2.txt 3.txt \
    -va --plot --plot_mode xz