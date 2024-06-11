# ExplorationRRT
A Tree-based Next-best-trajectory Method for 3D UAV Exploration

![image(14)](https://github.com/LTU-RAI/ExplorationRRT/assets/49238097/98865471-765b-4a34-9b82-17dca53e53b4)

The ERRT framework is a combined Exploration-Planning algorithm for the exploration of unknown and unstructured 3D environments - in this version set up for rotorcraft UAVs. In the framework we leverage RRT tree-expansion for rapid path generation to multiple sampled candidate goals, combined with iterative path improvement modules and a NMPC to generate dynamic paths. Candidate RRT branches are then evaluated for distance, model-based actuation, and 3D LiDAR sensor based information gain along the candidate branches - not just at the end-goals or frontiers. The fundamental goal of ERRT is to find the momentary local "Next-best-trajectory" for continued and efficient exploration of unknown environments.

The framework heavily relies on the UFOmap occupancy mapper library and requires an UFOmap topic as input, together with the robot localization/odometry. As output the user can select to use a continously updated pose reference or the full trajectory to track.  

The public version of the framework is provided as a docker with all dependencies already configured including but not limited to: the UFOmap framework (https://github.com/UnknownFreeOccupied/ufomap), the RotorS UAV simulator and trajectory tracker (https://github.com/ethz-asl/rotors_simulator), and the NMPC optimizer (https://alphaville.github.io/optimization-engine/). Instructions for installing the docker are provided below, and further down are detailed instruction on running the framework from the installed docker. 

The docker is pre-configured to load the DARPA SubT Cave World consisting of wide interconnected tunnels and caves, and the ERRT and UFOmap tuning is specified for this environment - and as such optimal performance in other environments might require parameter tuning. The provided version relies also on a full-tracjectory tracking controller provided by RotorS, for the UAV to track the generated and selected path. 

When using the framework in academic publications, please cite our published work as: 


# Installation 

*** DOCKER INSTRUCTIONS TO-DO ***

# Fundamentals & Critical launch parameters
This section will detail some of the critical launch parameters of interest for the user - focusing on baseline configuration params, and those that can have a large impact on using ERRT different environments. The relevant launch files launch/errt.launch, and launch/server.launch (for UFOmap) has more details for every configuration parameter. 

## ROS Topics

The Following ROS topic configurations can be found in the launch/errt.launch file. 

**remap from="ODOMETRY_IN_" to="/hummingbird/ground_truth/odometry"** - The robot odometry topic 

**remap from="UFOMAP_IN_" to="ufomap_mapping_server_node/map"** - The UFOmap topic. Maps at different depths can also be used.

**remap from="REFERENCE_OUT_" to="/hummingbird/reference"** - Momentary pose references along the trajectory. These are updated by the condition of the robot position being closer to the current reference than PATH_UPDATE_DIST_. 

**remap from="PATH_OUT_" to="/hummingbird/command/trajectory"** - Path topic as a MultiDOFJointTrajectory message - Set up with the message type to be synced with the RotorS trajectory tracking controller.

There are also many visualization topics included in the ERRT program such as the RRT (whole tree), GOALS, HITS (predicted exploration), PATHS (all paths), and SELECTED_PATH. These are set up in the rviz/errt.rviz.

## Tuning Parameters

This section will detail a number of tuning parameters in launch/errt.launch and launch/server.launch (UFOmap). The related launch files detail all possible tuning and configuration parameters. 

**resolution** - Found in server.launch. The resolution of the UFOmap. 

**PLANNING_DEPTH_** and **INFO_GAIN_DEPTH_** - The depth of the Octree in UFOmap practically means merging voxels into larger ones. This significantly speeds up various volumetric occupancy checks. ERRT is configured to use a small **resolution** of ~0.05-0.15m but performing computationally demanding actions such as information gain calculations at a set depth in the Octree. 

**ROBOT_SIZE_** - The approximate size-radius of the robot. This is used in volumetric collision checks to ensure robot-safe trajectories.  

**V_LOCAL_** - The side length of the bounding box that defines the local sampling space for generating the RRT.

**NUMBER_OF_NODES_** - The size of the RRT as the exit condition to stop tree expansion. 

**NUMBER_OF_GOALS_** - The number of candidate goals to be generated and consequently the number of candidate trajectories that will be investigated. Large effect on computation time. 

**SENSOR_RANGE_** - The range of the LiDAR model used to compute predicted information gain along candidate branches. Must be set lower than the **max_range** in server.launch. Recommended to use a much smaller range than the real range of the sensor for both parameters - both for computation effort and to guarantee useful, dense, and consistent data integration into the UFOmap. 

**SENSOR_VERTICAL_** - The vertical cutoff angle in radians for the LiDAR model as half the vertical field of view. Should match the onboard LiDAR on the robot. Ex. Ouster 32-beam 45deg FoV -> **SENSOR_VERTICAL** = 0.393

**INFO_CALC_DIST_** - The distance between nodes in candidate branches where information gain calculations will be performed. Too small values lead to big overlap between checks, and a significant increase in computation time. 

**K_Dist, K_Info, K_u** - Gains related to the distance cost, information gain, and actuation cost along candidate branches. Changing these will change which types of trajectories will be favored. Note: the information gain is calculated from the *number* of unknown voxels in sensor view so **K_Info** might need to be edited with significant changes to **resolution** or **INFO_GAIN_DEPTH_**. 

**INITIAL_POINT_** - *true/false* indicates if the UAV should travel to a specified initial coordinate before ERRT takes over navigation, set by subsequent *x,y,z*-coordinates. Can be useful to provide an initial direction of exploration or to "hot-start" ERRT with a small map and not just with scans from the ground. 

# Running the framework

*** TO-DO WITH DOCKER CONFIG ***
```
