#!/bin/bash

read bag_name

rosbag record -O $bag_name /tf /tf_static /hummingbird/ground_truth/odometry /errt_execution_time /command_path /octomap_point_cloud_centers /vis/ref_path /hummingbird/velodyne_points /dsp_path     
