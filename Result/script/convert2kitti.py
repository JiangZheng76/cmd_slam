import os

def convert2kitti(input_path,output_path):
    # 读取4 * 4 
    data = []
    with open(input_path, 'r') as input_file:
        for line in input_file:
            data.append(line)
    
    output_data = []
    for pose in data:
        pose = pose.split(' ')
        output_data.append(pose[0:12])
    print(output_data)
    
    with open(output_path,'w') as output_file:
        for pose in output_data:
            for i in range(12):
                output_file.write(pose[i])
                if(i != 11):
                    output_file.write(" ")
            output_file.write("\n")
    
input = "/home/sysu/cmd_ws/src/cmd-slam/Result/stereo_dso/kitti/00/Trajectory_mutli/6.txt"
output = "/home/sysu/cmd_ws/src/cmd-slam/Result/stereo_dso/kitti/00/Trajectory_mutli/kitti_6.txt"
convert2kitti(input,output)