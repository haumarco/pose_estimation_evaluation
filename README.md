# curve, then 3 straight tiles (experiment 1)

This experiment was performed on the duckietown map ETH_large_loop. Place the Duckiebot in the beginning as stated in the thesis (bottom of map). Start the ROS-Master and acquisition-bridge on the watchtowers as described in the duckietown documentation. Do not start it on the Duckiebot. Before running the experiment start recording the bag files. Stop them after the Duckiebot has reached the end of the straight tiles.

ROS-Master Computer:

    $ dts cli
    $ rosbag record -a -O t_wtX.bag
Duckiebot:

    $ dts duckiebot demo --demo_name base --duckiebot_name DB_NAME
    $ cd /data/logs
    $ rosbag record /DB_NAME/sensor_fusion_node/fusion_lane_pose /DB_NAME/lane_filter_node/lane_pose -O t_dbX.bag

Where X is the experiment number (1,2,3,4,...). Download the bag file of the Duckiebot in the Browser: DB_NAME.local:8082 in the logs folder.
Create a folder named X (the experiment number) in the pose_estimation_evaluation folder. Run the following command in the pose_estimation_evaluation directory:

    $ complete_evaluation.sh X DB_NAME

The code calculates the mean and standard deviation of the experiment and writes it into the file evaluation_results.txt. The course of the each data stream can be plotted with plot_curve_3_straight.py. Just take a look into the code to plot what's desired.

# 3x3 loop experiment (appendix)

For this experiment please use the ETH_small_loop_3 map. Same procedure as above with the exception of the changing two parts in complete_evaluation.sh: eval_curve_3_straight.py to eval_3x3_loop.py and ETH_large_loop to ETH_small_loop_3.

The measurements of the localization system are getting transformed to match the coordinate system orientation of the lane. For each straight tile a mean and stdev gets calculated. The values for the whole experiment is a weighted average over all straight tiles. Each mean and stdev gets the weighting n/N. Where n is the number of localization system measurements in this specific straight tile and N is the total number of localization system measurements during all straight tiles together.
 

