#! /bin/bash
# seq_num="00"
gt=/home/sysu/cmd_ws/src/cmd-slam/Result/groundtruth/kitti
result=/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti

export PATH="/home/sysu/.local/bin:$PATH"

# seq_nums=("00" "01" "02" "03" "04" "05" "06" "07" "08" "09" "10")
for i in "${!seq_nums[@]}"; do
    seq_num=${seq_nums[$i]}
    echo "=========${seq_num} ARE==========="
    evo_ape tum -r angle_deg ${gt}/${seq_num}.txt \
     ${result}/${seq_num}/Traject.txt  --plot

    # echo "=========${seq_num} ATE==========="
    # evo_ape tum ${gt}/${seq_num}.txt \
    #  ${result}/${seq_num}/odom_Traject1.txt 
done