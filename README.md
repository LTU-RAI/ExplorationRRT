# Exploratory rapidly-exploring random tree (ERRT)

The ERRT framework consists of ERRT module and the ufomap mapper, both of which depend on the ufomap software. The errt folder contains the implemented ERRT module, the ufomap_mapping folder contains the modified ufomap mapper and the Ufomap folder contains the ufomap as it existed as this project came to a close.

# Installation 

It is assumed that the Noetic version of robot operating system (ROS) has been installed; if not follow the guide at the following link:

http://wiki.ros.org/noetic/Installation/Ubuntu

After ROS Noetic has been sucessfully installed, follow the steps below:
Install the Intel(R) Threading Building Blocks 2018:
```
sudo apt install libtbb-dev
```

Prerequisites:
```
sudo apt install python3-catkin-tools python3-osrf-pycommon
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init # Init Catkin workspace
catkin config --extend /opt/ros/noetic  # exchange noetic for your ros distro if necessary
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # To enable release mode compiler optimzations
```
It will be assumed that your Catkin workspace is in ~/catkin_ws.

1. Install system dependencies
```
sudo apt install libtbb-dev
```

2. Move into your Catkin workspace
```
cd ~/catkin_ws/src
```

3. Clone the framework using either SSH or HTTPS
```
git clone https://github.com/Decending/ExplorationRRT.git
```
or
```
git clone git@github.com:Decending/ExplorationRRT.git
```


4. Move into your the errt folder
```
cd errt
```

5. Build the optimizer
```
python3 rrt_costgen.py
```

6.
```
# Build your workspace
catkin build
# Move out of errt folder
cd ~/catkin_ws
# Source your workspace
source devel/setup.bash
```

# Fundamentals

To work with and understand the tuning found here one needs to fully understand how a path is evaluated. In short, the ERRT framework tries to minimize the cost for finding new information, which it naturally tries to maximize. During path evaluation there are three variables of interest:

*Actuation cost* - This denotes the effort required for the drone to get from point **a** to point **b**
*Distance cost* - This denotes the distance the drone needs to travel to get from point **a** to point **b**
*Information gain* - This denotes the new information which the drone expects to get on the path from point **a** to point **b**

The evaluation of a single path follows the equation:

*Total cost* = *Actuation cost* + *Distance cost* - *Information gain*

To affect the weights of the different variables against each other you can use the scalers found in errt.launch.

# Getting started

To get the framework going there are a few variables which needs to be set correctly first.

**frame_id & robot_frame_id** - Found in the server.cfg file. These variables are the frame of the published maps and the frame of the robot respectively.

**remapping of cloud in** - Found in the server.launch file. This remaping decides from which topic the mapper will receive the point cloud data from. Without this data there will be no ufomap.

**MAP_FRAME_ID** - Found in the errt.launch file. This variables set the frame for a number of visualization messages.

**Sensor parameters** - Found in the errt.launch file. These variables are to be set according to the sensor ranges for your particular lidar system to accurately simulate the results.

# Tuning guide

**Insert depth** - Found in the server.cfg file. This variable affects at which depth data is inserted, where a lower level than the ufomap depth level decreases the popping between the different states for a voxel, but at the cost of slower integration into the map.

**Initial point parameters** - Found in the errt.launch file. These variables set whether or not the drone is to travel to a specific point before starting the rest of the algorithm. Important to change for restarts in cases where the map is persistent, in a simulation for example.

**Tuning parameters** - Found in the errt.launch file. These variables control how a path is evaluated and acts as the most significant tuning found in the framework.

**RRT-tree parameters** - Found in th errt.launch file. These variables decides how the tree is generated and can be of interest if there's a significant change in environment for the exploratory drone.

**NMPC parameters** - Found in the errt.launch file. These variables are for tuning the nonlinear model predictive controller, which heavily affects the final path and the actuation cost.

# Running the framework

The errt requires the ufomap from the mapper to function properly, so to run the framework one has to launch the mapper with:
```
roslaunch ufomap_mapper server.launch
```
followed by the errt module by running:
```
roslaunch errt errt.launch
```
