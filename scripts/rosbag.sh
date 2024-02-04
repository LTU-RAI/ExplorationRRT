#!/bin/bash

read bag_name

rosbag record -o $bag_name /tf /tf_static /ufomap_mapping_server_node/map_depth_3 /ufomap_mapping_server_node/map_depth_2 /ufomap_mapping_server_node/map_depth_1 /ufomap_mapping_server_node/map /tracked_trajectory /hummingbird/velodyne_points /hummingbird/reference /hummingbird/ground_truth/odometry /RRT_PATHS /RRT_NODES /RRT_GOALS /RESERVED_GOALS /POSITION /PATH_TAKEN /Internal_ufo_map /HITS /CHOSEN_RRT_PATH_VISUALIZATION /CHOSEN_RRT_PATH_VISUALIZATION_array /chosen_path /errt_execution_time

 
