#!/bin/bash
dataset="kitti360"
seq_nums=("09")
agent_nums=(4)

result_base_path="/home/sysu/cmd_ws/src/cmd-slam/Result/cmd_stereo_dso/multi/"
result_tmp_path="/home/sysu/cmd_ws/src/cmd-slam/Result/cmd_stereo_dso/multi/tmp"
source ~/cmd_ws/devel/setup.bash
mkdir -p ${result_tmp_path}

for((i=0; i<${#seq_nums[@]}; i++)); do
    seq_num=${seq_nums[$i]}
    result_path="${result_base_path}/${dataset}/${seq_num}"
    echo "result_path ${result_path}"
    # BACKEND_PID=$! 
    sleep 1

    for((n=0; n<${agent_nums[${i}]}; n++));
    do
        echo "roslaunch cmd_stereo_dso agent.launch seqnum:=${seq_num} save_path:=${result_tmp_path} time:=1 ag_n:=${n} dataset:=${dataset}"
        roslaunch cmd_stereo_dso agent.launch seqnum:=${seq_num} \
            save_path:=${result_tmp_path} time:=1 ag_n:=${n} dataset:=${dataset}&
        sleep 1
    done
    
    echo "Press Enter to save data..."
    read -s  # 阻塞等待用户按下 Enter 键

    # 复制结果文件到指定目录
    mkdir -p "${result_path}"
    # odom
    mv ${result_tmp_path}/* ${result_path}
    # server
    mv /home/sysu/.ros/Trajectory/* "${result_path}/"
    echo "Results for sequence ${seq_num} saved to ${result_path}."
    echo "Press restart backend...."
    read -s
done
echo "Processing completed."

