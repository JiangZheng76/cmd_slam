#!/bin/bash
#usage: python xxx.py file_name

# seq_nums=("05" "06" "07" "08" "10")
seq_nums=("05" "08")


result_base_path="/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/malaga"
mkdir -p ${result_base_path}
for i in "${!seq_nums[@]}"; do
    seq_num=${seq_nums[$i]}
    result_path="${result_base_path}/${seq_num}"

    roslaunch fast_stereo_dso malaga.launch seqnum:=${seq_num} save_path:=${result_base_path} time:=2

    # 检查是否成功生成结果文件
    if [ -f ${result_base_path}/"Traject.txt" ] && [ -f ${result_base_path}/"TimeResult.txt" ]; then
        mkdir -p "${result_path}"
        # 复制结果文件到指定目录
        mv ${result_base_path}/"Traject.txt" "${result_path}/odom_Traject.txt"
        mv ${result_base_path}/TimeResult.txt "${result_path}/TimeResult.txt"
        mv /home/sysu/.ros/Trajectory/tum/1.txt "${result_path}/Traject.txt"
        mv /home/sysu/.ros/backend_result.txt "${result_path}/backend_result.txt"
        echo "Results for sequence ${seq_num} saved to ${result_path}."
    else
        echo "Error: Results for sequence ${seq_num} were not generated."
    fi
    sleep 1
done
echo "Processing completed."

