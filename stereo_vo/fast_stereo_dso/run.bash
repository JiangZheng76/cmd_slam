#!/bin/bash
#usage: python xxx.py file_name
PATH=/home/sysu/cmd_ws/devel/lib/cmd_stereo_dso
cd ${PATH}
file_path=/media/sysu/200GB_Other/Dataset/Slam/malaga/malaga-urban-dataset-extract-10/malaga-urban-dataset-extract-10_rectified_1024x768_Images
    ./dso_dataset \
  files=${file_path} \
  calib=/home/sysu/cmd_ws/src/cmd-slam/stereo_vo/cmd_stereo_dso/calib/malaga/cam.txt \
  preset=0 mode=1 \
  quiet=1 nomt=1


