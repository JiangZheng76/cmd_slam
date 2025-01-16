
seqnum="10"
vo="cmd_stereo_dso"
export PATH="/home/sysu/.local/bin:$PATH"
cd /home/sysu/cmd_ws/src/cmd-slam/Result/$vo/malaga/$seqnum/tum/
evo_traj tum --ref=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/malaga/$seqnum.txt \
    1.txt 2.txt 3.txt \
    -va --plot --plot_mode xz