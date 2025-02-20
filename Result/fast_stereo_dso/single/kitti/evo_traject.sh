#! /bin/bash
seq_num="09"
gt=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/kitti
result=/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti

export PATH="/home/sysu/.local/bin:$PATH"
echo "角度绝对 rmse"
# evo_ape tum -r angle_deg ${gt}/${seq_num}.txt \
#      ${result}/${seq_num}/Traject.txt \
#     -va --plot --plot_mode xyz 

# echo "位移绝对 rmse"
# evo_ape tum -r trans_part ${gt}/${seq_num}.txt \
#      ${result}/${seq_num}/Traject.txt \
#     -va --plot --plot_mode xyz 

# echo "全部绝对 rmse"
# evo_ape tum ${gt}/${seq_num}.txt \
#      ${result}/${seq_num}/Traject.txt \
#     -va --plot --plot_mode xyz 

# echo "相对 rmse"
# evo_rpe tum ${gt}/${seq_num}.txt \
#      ${result}/${seq_num}/Traject.txt \
#     -va --plot --plot_mode xyz 

echo "画轨迹"
evo_traj tum --ref=${gt}/${seq_num}.txt \
     ${result}/${seq_num}/Traject.txt \
    -va --plot --plot_mode xz 