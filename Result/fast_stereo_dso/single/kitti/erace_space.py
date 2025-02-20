import os

def erace_sapce(line):
    datas = line.split(' ')
    result = []
    for data in datas:
        if data != '':
            result.append(data)
    return result
def process_file(input_file_path, output_file_path):
    """
    读取整个文件内容，将每行数据的多余空格替换为单个空格，然后写入输出文件。
    """
    content = []
    with open(input_file_path, 'r') as infile:
        # 读取整个文件内容
        for line in infile:
            content.append(line)
    
    with open(output_file_path, 'a') as outfile:
        for c in content:
            datas = erace_sapce(c)
            for i in range(len(datas)):
                outfile.write(datas[i])
                if i != len(datas) - 1:
                    outfile.write(' ')
    


# 设置输入和输出文件夹路径
seq_num=["00",  "01",  "02",  "03",  "04",  "05",  "06",  "07",  "08",  "09",  "10"]
for seq in seq_num:
    input_folder = "/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti/"+seq+"/odom_Traject.txt"  # 替换为你的输入文件夹路径
    output_folder = "/home/sysu/cmd_ws/src/cmd-slam/Result/fast_stereo_dso/single/kitti/"+seq+"/odom_Traject1.txt"  # 替换为你的输出文件夹路径
    process_file(input_folder, output_folder)
