#!/bin/bash

echo -e "\ntest: $1"
echo -e "\npost-processor\n"

docker run --name post_processor -it --rm -e INPUT_BAG_PATH=/data/t_wt$1 -e OUTPUT_BAG_PATH=/data/processed_$1.bag -e ROS_MASTER_URI=http://YOUR_ROS_MASTER_IP:11311 -v PATH_TO_POSE_ESTIMATION_EVAL/$1:/data duckietown/post-processor:daffy-amd64

wait
sleep 1
echo -e "\ngraph-optimizer\n"

docker run --rm -e ATMSGS_BAG=/data/processed_$1.bag -e OUTPUT_DIR=/data -e ROS_MASTER=duckietown-1 -e ROS_MASTER_IP=YOUR_ROS_MASTER_IP --name graph_optimizer -v PATH_TO_POSE_ESTIMATION_EVAL/$1:/data -e DUCKIETOWN_WORLD_FORK=duckietown -e MAP_NAME=ETH_large_loop duckietown/cslam-graphoptimizer:daffy-amd64

wait
sleep 1

echo -e "\ncreate txt files\n"
cd $1
rostopic echo -b t_db$1.bag -p /$2/sensor_fusion_node/fusion_lane_pose > SF_data$1.txt
wait

rostopic echo -b t_db$1.bag -p /$2/lane_filter_node/lane_pose > cam_data$1.txt
wait
rostopic echo -b t_db$1.bag -p /$2/sensor_fusion_node/encoder_pose > enc_data$1.txt
cd ..
wait
sleep 0.5

echo -e "\nevaluation\n"

python3 eval_curve_3_straight.py $1

echo "test $1 evaluation done!"