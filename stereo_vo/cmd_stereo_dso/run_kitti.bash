#!/bin/bash
#usage: python xxx.py file_name
PATH=/home/sysu/cmd_ws/src/cmd-slam/stereo_vo/cmd_stereo_dso
cd $PATH
for((i=00;i<=00;i++))
  do
    {
     a=$((100+$i))
     seqnum=${a:1}
     echo "running seqence ${seqnum}"
     echo "./${usePriorPoses}/${seqnum}.txt"
    
    # run dso
     ./build/bin/dso_dataset \
 	  files=/media/sysu/Samsung_T5/JZ_CODSV/${seqnum} \
 	  calib=./calib/kitti/${seqnum}.txt \
	  groundtruth=./groundTruthPose/kitti/${seqnum}.txt \
 	  preset=0 mode=1 \
	  quiet=1 nomt=1
    }&
  done


