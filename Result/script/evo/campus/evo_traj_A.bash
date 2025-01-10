vo="cmd_stereo_dso"
export PATH="/home/sysu/.local/bin:$PATH"
cd /home/sysu/cmd_ws/src/cmd-slam/Result/$vo/campus/tum/
evo_traj tum --ref=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/campus/gt_A.txt \
    3.txt \
    -va --plot --plot_mode xz