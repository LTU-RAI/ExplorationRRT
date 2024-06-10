# ExplorationRRT

The ERRT framework is a combined Exploration-Planning algorithm for the exploration of unknown and unstructured 3D environments - in this version set up for rotorcraft UAVs. In the framework we leverage RRT tree-expansion for rapid path generation to multiple sampled candidate goals, combined with iterative path improvement modules and a NMPC to generate dynamic paths. Candidate RRT branches are then evaluated for distance, model-based actuation, and 3D LiDAR sensor based information gain along the candidate branches - not just at the end-goals or frontiers. The framework heavily relies on the UFOmap occupancy mapper library and requires an UFOmap topic as input, together with the robot localization/odometry. As output the user can select to use a continously updated pose reference or the full trajectory to track.  

The public version of the framework is provided as a docker with all dependencies already configured including but not limited to: the UFOmap framework (https://github.com/UnknownFreeOccupied/ufomap), the RotorS UAV simulator and trajectory tracker (https://github.com/ethz-asl/rotors_simulator), and the NMPC optimizer (https://alphaville.github.io/optimization-engine/). Instructions for installing the docker are provided below, and further down are detailed instruction on running the framework from the installed docker. 

The docker is pre-configured to load the DARPA SubT Cave World consisting of wide interconnected tunnels and caves, and the ERRT and UFOmap tuning is specified for this environment - and as such optimal performance in other environments might require parameter tuning. The provided version relies also on a full-tracjectory tracking controller provided by RotorS, for the UAV to track the generated and selected path. 

When using the framework in academic publications, please cite our published work as: 


# Installation 

*** DOCKER INSTRUCTIONS TO-DO ***

# Fundamentals & Critical launch parameters
This section will detail some of the critical launch parameters of interest for the user - focusing on baseline configuration params, and those that can have a large impact on using ERRT different environments. The relevant launch files launch/errt.launch, and launch/server.launch (for UFOmap) has more details for every configuration parameter. 

The ROS topics in ERRT that can be configured can be found in errt.launch:

\it{remap from="ODOMETRY_IN_" to="/hummingbird/ground_truth/odometry"} - The robot odometry topic 

remap from="UFOMAP_IN_" to="ufomap_mapping_server_node/map" - The UFOmap topic. Maps at different depths can also be used.

remap from="REFERENCE_OUT_" to="/hummingbird/reference" - Momentary pose references along the trajectory. These are updated by the condition of the robot position being closer to the current reference than PATH_UPDATE_DIST_. 

remap from="PATH_OUT_" to="/hummingbird/command/trajectory" Path topic as a MultiDOFJointTrajectory message - Set up with the message type to be synced with the RotorS trajectory tracking controller.

There are also many visualization topics included in the ERRT program such as the RRT (whole tree), GOALS, HITS (predicted exploration), PATHS (all paths), and SELECTED_PATH. These are set up in the rviz/errt.rviz.




# Running the framework

*** TO-DO WITH DOCKER CONFIG ***
```
