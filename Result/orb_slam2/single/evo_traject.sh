#! /bin/bash




















seq_num="10"
gt=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/kitti
result=/home/sysu/cmd_ws/src/cmd-slam/Result/orb_slam2/single/kitti

# export PATH="/home/sysu/.local/bin:$PATH"
# echo "角度绝对 rmse"
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

# echo "画轨迹"
# evo_traj tum --ref=${gt}/${seq_num}.txt \
#      ${result}/${seq_num}/Traject.txt \
#     -va --plot --plot_mode xz 

#! /bin/bash
gt=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/malaga
result=/home/sysu/cmd_ws/src/cmd-slam/Result/orb_slam2/single/malaga

export PATH="/home/sysu/.local/bin:$PATH"

seq_nums=("05" "06" "07" "10")
for i in "${!seq_nums[@]}"; do
    seq_num=${seq_nums[$i]}
#     echo "=========${seq_num} ARE==========="
#     evo_ape tum ${gt}/${seq_num}.txt \
     # ${result}/${seq_num}/Traject.txt  --plot -va
     
     evo_traj tum ${result}/${seq_num}/Traject.txt  --plot
    #  evo_traj tum ${gt}/${seq_num}.txt  --plot

    # echo "=========${seq_num} ATE==========="
    # evo_ape tum ${gt}/${seq_num}.txt \
    #  ${result}/${seq_num}/odom_Traject1.txt 
done