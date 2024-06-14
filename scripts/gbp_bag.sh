#!/bin/bash

read bag_name

rosbag record -O $bag_name /tf /tf_static /rmf_obelix/ground_truth/odometry /errt_execution_time /command_path /octomap_point_cloud_centers /vis/ref_path /rmf_obelix/velodyne_points     
