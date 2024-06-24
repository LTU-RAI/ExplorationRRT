FROM osrf/ros:noetic-desktop-full-focal

ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV LD_LIBRARY_PATH=/usr/lib/nvidia:$LD_LIBRARY_PATH

# Use build argument for the username
ARG USERNAME
ENV USR_NAME=$USERNAME

RUN echo "Current working directory after usermod:" && pwd
# Install some handy tools.
RUN set -x \
    && apt-get update \
    && apt-get --with-new-pkgs upgrade -y \
    && apt-get install -y mesa-utils \
    && apt-get install -y libgl1-mesa-glx \
    && apt-get install -y vim \
    && apt-get install -y tmuxinator \
    && apt-get install -y python3-catkin-tools \
    && apt-get install -y python3-osrf-pycommon \
    && apt-get install -y python3-pip \
    && pip3 install opengen \
    && pip3 install gdown \
    && apt-get install -y libtbb-dev \
    && apt-get install -y ros-noetic-octomap-server \
    && apt-get install -y ros-noetic-octomap-ros \
    && apt-get install -y ros-noetic-octomap-rviz-plugins \
    && apt-get install -y ros-noetic-octomap-mapping \
    && apt-get install -y libtool \
    && apt-get install -y libgoogle-glog-dev \
    && apt-get install -y libnlopt-dev \
    && apt-get install -y libsuitesparse-dev \
    && apt-get install -y ros-noetic-nlopt \
    && apt-get install -y liblapacke-dev \
    && apt-get install -y ros-noetic-gtsam \
    && apt-get install -y ros-noetic-rosmon \
    && apt-get install -y iputils-ping \
    && apt-get install -y apt-transport-https ca-certificates \
    && apt-get install -y openssh-server python3-pip exuberant-ctags \
    && apt-get install -y git vim tmux nano htop sudo curl wget gnupg2 \
    && apt-get install -y bash-completion \
    && apt-get install -y libcanberra-gtk3-0 \
    && apt-get install -y ros-noetic-gmapping ros-noetic-slam-gmapping ros-noetic-openslam-gmapping \
    && apt-get install -y ros-noetic-joy \
    && apt-get install -y ros-noetic-twist-mux \
    && apt-get install -y ros-noetic-interactive-marker-twist-server \
    && apt-get install -y ros-noetic-fath-pivot-mount-description \
    && apt-get install -y ros-noetic-flir-camera-description \
    && apt-get install -y ros-noetic-realsense2-description \
    && apt-get install -y ros-noetic-lms1xx \
    && apt-get install -y ros-noetic-robot-localization \
    && apt-get install -y ros-noetic-teleop-twist-keyboard \
    && apt-get install -y ros-noetic-teleop-twist-joy \
    && apt-get install -y ros-noetic-rviz-imu-plugin \
    && apt-get install -y ros-noetic-gmapping \
    && apt-get install -y ros-noetic-mavros-msgs \
    && rm -rf /var/lib/apt/lists/* 
    
RUN set -x \
    && useradd -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" 

# The OSRF container didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python

USER $root
WORKDIR /home/$USERNAME

# Install Rust using rustup
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Add Rust binaries to the PATH
RUN echo "source $HOME/.cargo/env" >> /home/$USER_NAME/.bashrc

RUN sudo usermod -a -G video $USERNAME

RUN rosdep update \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

# Clone the necessary packages for simulation. 
RUN mkdir -p catkin_ws/src 

WORKDIR /home/$USERNAME/catkin_ws/src 

RUN catkin init
RUN catkin config --extend /opt/ros/noetic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN  git clone https://github.com/ethz-asl/eigen_checks.git
RUN  git clone https://github.com/catkin/catkin_simple.git
RUN  git clone https://github.com/ethz-asl/eigen_catkin.git
RUN  git clone https://github.com/ntnu-arl/lidar_simulator.git
RUN  git clone https://github.com/ethz-asl/mav_comm.git
RUN  git clone https://github.com/ros-planning/navigation_msgs.git
RUN  git clone https://github.com/ethz-asl/numpy_eigen.git
RUN  git clone --branch melodic-devel https://github.com/ros-perception/perception_pcl.git
RUN  git clone https://github.com/ros/xacro.git
RUN  git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git

RUN  git clone https://github.com/LTU-RAI/darpa_subt_worlds.git
RUN  git clone https://github.com/LTU-RAI/rotors_simulator.git && cd rotors_simulator && git pull
RUN  git clone https://github.com/LTU-RAI/ufomap.git

RUN git clone https://github.com/LTU-RAI/geometry2.git
RUN  git clone https://github.com/aakapatel/mav_control_rw.git
USER root

WORKDIR /home/$USERNAME/catkin_ws/
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN catkin init
RUN catkin config --extend /opt/ros/noetic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# Build all packages 

# Add Qt dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-tf2-sensor-msgs \
    python3-catkin-tools \
    python3-osrf-pycommon \
    libtbb-dev \
    qtbase5-dev \
    qtdeclarative5-dev \
    libqt5x11extras5-dev \
    && rm -rf /var/lib/apt/lists/*

# ENV QT_X11_NO_MITSHM 1

RUN sudo catkin build 

#Clone Exploration package 
WORKDIR /home/$USERNAME/catkin_ws/src/

RUN git clone https://github.com/LTU-RAI/ExplorationRRT.git

WORKDIR /home/$USERNAME/catkin_ws/src/ExplorationRRT/

USER root

# Build the cost gen for RRT solver. 
RUN /bin/bash -c 'source $HOME/.cargo/env; cd /home/$USERNAME/catkin_ws/src/ExplorationRRT; python3 rrt_costgen.py'

WORKDIR /home/$USERNAME/catkin_ws/

RUN sudo catkin build errt  

USER $USERNAME
WORKDIR /home/$USERNAME
RUN mkdir -p /home/$USERNAME/.config/tmuxinator

RUN git clone https://github.com/jimeh/tmux-themepack.git /home/$USERNAME/.tmux-themepack \
    && git clone https://github.com/tmux-plugins/tmux-resurrect /home/$USERNAME/.tmux-resurrect
COPY --chown=$USERNAME:$USERNAME ./.tmux.conf /home/$USERNAME/.tmux.conf
COPY --chown=$USERNAME:$USERNAME ./statusline.conf /home/$USERNAME/statusline.conf
COPY --chown=$USERNAME:$USERNAME ./utility.conf /home/$USERNAME/utility.conf
COPY --chown=$USERNAME:$USERNAME ./errt.yml /home/$USERNAME/.config/tmuxinator/errt.yml

# Set some decent colors if the container needs to be accessed via /bin/bash.
RUN echo LS_COLORS=$LS_COLORS:\'di=1\;33:ln=36\' >> ~/.bashrc \
    && echo export LS_COLORS >> ~/.bashrc \
    && echo 'alias tmux="tmux -2"' >> ~/.bashrc \
    && echo 'PATH=~/bin:$PATH' >> ~/.bashrc \
    && touch ~/.sudo_as_admin_successful # To surpress the sudo message at run.

# some automatic sourcing for convenience.
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash --extend" >> /home/$USERNAME/.bashrc
RUN echo "export PS1='\u@\h:\W\$ '" >> /home/$USERNAME/.bashrc 
RUN echo "alias cbe=\"sudo catkin build errt\" " >> /home/$USERNAME/.bashrc
RUN echo "alias tmkill=\"tmux kill-session\" " >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/.gazebo/
RUN mkdir -p models
COPY models /home/$USERNAME/.gazebo/models

WORKDIR /home/$USERNAME/ 
RUN mkdir -p .ignition
COPY ignition_models /home/$USERNAME/.ignition/

WORKDIR /home/$USERNAME/catkin_ws/

# Add Rust binaries to the PATH
RUN echo "source $HOME/.cargo/env" >> /home/$USERNAME/.bashrc
STOPSIGNAL SIGTERM

# RUN groupadd -r $USERNAME && useradd -r -g $USERNAME $USERNAME
CMD sudo service ssh start && /bin/bash

