# ExplorationRRT

A Tree-based Next-best-trajectory Method for 3D UAV Exploration

![image(14)](https://github.com/LTU-RAI/ExplorationRRT/assets/49238097/98865471-765b-4a34-9b82-17dca53e53b4)

##

The ERRT framework is a combined Exploration-Planning algorithm for the exploration of unknown and unstructured 3D environments - in this version set up for rotorcraft UAVs. In the framework we leverage RRT tree-expansion for rapid path generation to multiple sampled candidate goals, combined with iterative path improvement modules and a NMPC to generate dynamic paths. Candidate RRT branches are then evaluated for distance, model-based actuation, and 3D LiDAR sensor based information gain along the candidate branches - not just at the end-goals or frontiers. The fundamental goal of ERRT is to find the momentary local "Next-best-trajectory" for continued and efficient exploration of unknown environments.

##

The framework heavily relies on the UFOmap occupancy mapper library and requires an UFOmap topic as input, together with the robot localization/odometry. As output the user can select to use a continously updated pose reference or the full trajectory to track.  

The public version of the framework is provided as a docker with all dependencies already configured including but not limited to: the UFOmap framework (https://github.com/UnknownFreeOccupied/ufomap), the RotorS UAV simulator and trajectory tracker (https://github.com/ethz-asl/rotors_simulator), and the NMPC optimizer (https://alphaville.github.io/optimization-engine/). Instructions for installing the docker are provided below, and further down are detailed instruction on running the framework from the installed docker. 

The docker is pre-configured to load the DARPA SubT Cave World consisting of wide interconnected tunnels and caves, and the ERRT and UFOmap tuning is specified for this environment - and as such optimal performance in other environments might require parameter tuning. The provided version relies also on a full-tracjectory tracking controller provided by RotorS, for the UAV to track the generated and selected path. 


![image(15)](https://github.com/LTU-RAI/ExplorationRRT/assets/49238097/ed52dec3-6133-4387-a20a-fe6a104cbb18)


# Installation 

## Requirements

The ERRT repository only requires docker. If you do not have docker installed, please follow the docker installation instructions. If you already have docker installed, you can skip this step.

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin python3-pip
pip install gdown

```
The repository contains a Dockerfile that allows the user to build a docker image containing packages for exploration, planning, control and simulation environment. 

## Cloning the repository and building the docker

Clone the ERRT project

```bash
  git clone https://github.com/LTU-RAI/ExplorationRRT.git

```
Go to the ERRT directory and download cave models for gazebo. (This speeds up the world loading process in Gazebo)

```bash
  cd ExplorationRRT/docker
  gdown --id 1TDbXF9He_LXYY57Xo3tOxvEVYH3kPtS8
  gdown --id 1y7BDt0tjK9Ml7MlTxUwOTG8ZygO_dpI0
  unzip models.zip 
  unzip ignition_models.zip 
```

Build the docker image with following command. The build process might take some time when building for first time. 

```bash
  sudo docker build --build-arg USERNAME=$(whoami) -t errt_test . 

```


# Fundamentals & Critical launch parameters
This section will detail some of the critical launch parameters of interest for the user - focusing on baseline configuration params, and those that can have a large impact on using ERRT different environments. The relevant launch files launch/errt.launch, and launch/server.launch (for UFOmap) has more details for every configuration parameter. 

## ROS Topics

The Following ROS topic configurations can be found in the launch/errt.launch file. 

**remap from="odometry_in_" to="/hummingbird/ground_truth/odometry"** - The robot odometry topic 

**remap from="ufomap_in_" to="ufomap_mapping_server_node/map"** - The UFOmap topic. Maps at different depths can also be used.

**remap from="reference_out_" to="/hummingbird/reference"** - Momentary pose references along the trajectory. These are updated by the condition of the robot position being closer to the current reference than **path_update_dist**. 

**remap from="path_out_" to="/hummingbird/command/trajectory"** - Path topic as a MultiDOFJointTrajectory message - Set up with the message type to be synced with the RotorS trajectory tracking controller.

There are also many visualization topics included in the ERRT program such as the tree_expansion (whole tree), candidate_goals, predicted_info_gain (predicted exploration), candidate_branches (all paths), and selected_trajectory. These are set up in the rviz/errt.rviz.

## Tuning Parameters

This section will detail a number of tuning parameters in config/errt.yaml and launch/server.launch (UFOmap). The related launch files detail all possible tuning and configuration parameters. 

**resolution** - Found in server.launch. The resolution of the UFOmap. 

**planning_depth** and **info_gain_depth** - The depth of the Octree in UFOmap practically means merging voxels into larger ones. This significantly speeds up various volumetric occupancy checks. ERRT is configured to use a small **resolution** of ~0.05-0.15m but performing computationally demanding actions such as information gain calculations at a set depth in the Octree. 

**robot_size** - The approximate size-radius of the robot. This is used in volumetric collision checks to ensure robot-safe trajectories.  

**v_local** - The side length of the bounding box that defines the local sampling space for generating the RRT.

**number_of_nodes** - The size of the RRT as the exit condition to stop tree expansion. 

**number_of_goals** - The number of candidate goals to be generated and consequently the number of candidate trajectories that will be investigated. Large effect on computation time. 

**sensor_range** - The range of the LiDAR model used to compute predicted information gain along candidate branches. Must be set lower than the **max_range** in server.launch. Recommended to use a much smaller range than the real range of the sensor for both parameters - both for computation effort and to guarantee useful, dense, and consistent data integration into the UFOmap. 

**sensor_vertical_fov** - The vertical cutoff angle in radians for the LiDAR model as half the vertical field of view. Should match the onboard LiDAR on the robot. Ex. Ouster 32-beam 45deg FoV -> **SENSOR_VERTICAL** = 0.393

**info_calc_dist** - The distance between nodes in candidate branches where information gain calculations will be performed. Too small values lead to big overlap between checks, and a significant increase in computation time. 

**k_dist, k_info, k_u** - Gains related to the distance cost, information gain, and actuation cost along candidate branches. Changing these will change which types of trajectories will be favored. Note: the information gain is calculated from the *number* of unknown voxels in sensor view so **K_Info** might need to be edited with significant changes to **resolution** or **INFO_GAIN_DEPTH_**. 

**start_from_waypoint** - *true/false* indicates if the UAV should travel to a specified initial coordinate before ERRT takes over navigation, set by subsequent *x,y,z*-coordinates. Can be useful to provide an initial direction of exploration or to "hot-start" ERRT with a small map and not just with scans from the ground. 

## Test ERRT in a docker container
![errt_gif-ezgif com-speed](https://github.com/LTU-RAI/ExplorationRRT/assets/49238097/957df250-dddc-4bd1-b7e9-841269cb16f2)

Run the docker container with NVIDIA flags. In the ERRT directory:


```bash
    ./start_errt_gpu.sh
``` 

If you do not have NVIDIA GPU :

```bash
    ./start_errt_no_gpu.sh
```
Once you are inside the docker container, please run the following command to start the ERRT tmux session.
This session will launch the ERRT sub-modules and a Rviz window to visualize the drone exploring the cave environment. 

```bash
tmuxinator errt
```

## Acknowledgement

If you find the E-RRT framework useful in your research, please consider citing the E-RRT article.

```bibtex
@ARTICLE{10582913,
  author={Lindqvist, Björn and Patel, Akash and Löfgren, Kalle and Nikolakopoulos, George},
  journal={IEEE Transactions on Robotics}, 
  title={A Tree-based Next-best-trajectory Method for 3D UAV Exploration}, 
  year={2024},
  volume={},
  number={},
  pages={1-18},
  keywords={Robots;Robot sensing systems;Costs;Collision avoidance;Autonomous aerial vehicles;Trajectory;Three-dimensional displays;Tree-based Exploration;RRT;Subterranean Exploration;Field Robotics;Unmanned Aerial Vehicles},
  doi={10.1109/TRO.2024.3422052}}
}

```
