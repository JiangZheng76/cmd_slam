#!/bin/bash
seq_nums=("00" "02" "05" "08")
# seq_nums=("02" "05" "08")
agent_nums=(3 2 3 2)
# agent_nums=(2 3 2)

result_base_path="/home/sysu/cmd_ws/src/cmd-slam/Result/cmd_stereo_dso/multi/"
source ~/cmd_ws/devel/setup.bash
mkdir ${result_base_path}

for i in "${!seq_nums[@]}"; do
    seq_num=${seq_nums[$i]}
    result_path="${result_base_path}/${seq_num}"

    # roslaunch cmd_backend cmd_backend_node.launch&
    # BACKEND_PID=$! 
    sleep 1

    for((i=0; i<${agent_nums[$i]}; i++))
    do
        roslaunch cmd_stereo_dso agent.launch seqnum:=${seq_num} \
            save_path:=${result_base_path} time:=1 ag_n:=${i}&
        sleep 1
    done
    
    echo "Press Enter to save data..."
    read -s  # 阻塞等待用户按下 Enter 键

    # 检查是否成功生成结果文件
    mkdir -p "${result_path}"
    # 复制结果文件到指定目录
    # odom
    mv ${result_base_path}/* ${result_path}
    # server
    mv /home/sysu/.ros/Trajectory/* "${result_path}/"
    echo "Results for sequence ${seq_num} saved to ${result_path}."
    # kill -9 ${BACKEND_PID}
    echo "Press restart backend...."
    read -s
done
echo "Processing completed."

