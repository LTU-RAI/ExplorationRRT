FROM osrf/ros:noetic-desktop-full-focal
# The OSRF ROS Noetic containers use the root user.
# Therefore, the following commands are executed as root up until the
# USER user statement.

# We love UTF :) 
ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV LD_LIBRARY_PATH=/usr/lib/nvidia:$LD_LIBRARY_PATH


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
	&& pip3 install powerline-shell dearpygui \
	&& rm -rf /var/lib/apt/lists/* \
	# && useradd -ms /bin/bash aakapatel \
	# && echo "aakapatel:aakapatel" | chpasswd && adduser aakapatel sudo \
	# && echo "aakapatel ALL=(ALL) NOPASSWD: ALL " >> /etc/sudoers
  && useradd -m -s /bin/bash aakapatel \
    && echo 'aakapatel ALL=(ALL:ALL) NOPASSWD:ALL' >> /etc/sudoers

# The OSRF contianer didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python



USER aakapatel
WORKDIR /home/aakapatel

RUN sudo usermod -a -G video aakapatel


RUN rosdep update \
        && echo "source /opt/ros/noetic/setup.bash" >> /home/aakapatel/.bashrc


RUN git clone https://github.com/jimeh/tmux-themepack.git ~/.tmux-themepack  \
        && git clone https://github.com/tmux-plugins/tmux-resurrect ~/.tmux-resurrect
COPY --chown=user:user ./.tmux.conf /home/aakapatel/.tmux.conf
COPY --chown=user:user ./.powerline.sh /home/aakapatel/.powerline.sh
COPY --chown=user:user ./errt.yml /home/aakapatel/.config/tmuxinator/errt.yml


# Install Rust using rustup
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Add Rust binaries to the PATH
ENV PATH="/root/.cargo/bin:${PATH}"


#
# Set some decent colors if the container needs to be accessed via /bin/bash.
RUN echo LS_COLORS=$LS_COLORS:\'di=1\;33:ln=36\' >> ~/.bashrc \
        && echo export LS_COLORS >> ~/.bashrc \
        && echo 'source ~/.powerline.sh' >> ~/.bashrc \
        && echo 'alias tmux="tmux -2"' >> ~/.bashrc \
        && echo 'PATH=~/bin:$PATH' >> ~/.bashrc \
        && touch ~/.sudo_as_admin_successful # To surpress the sudo message at run.



RUN echo "source /home/aakapatel/catkin_workspaces/exploration_ws/devel/setup.bash --extend" >> /home/aakapatel/.bashrc

RUN echo "source $HOME/.cargo/env " >> /home/aakapatel/.bashrc
RUN echo "alias err=\"sudo catkin build errt\" " >> /home/aakapatel/.bashrc
RUN echo "alias tmkill=\"tmux kill-session\" " >> /home/aakapatel/.bashrc
RUN echo "alias cde=\"cd catkin_workspaces/exploration_ws \" " >> /home/aakapatel/.bashrc

WORKDIR /home/aakapatel/.gazebo/

RUN mkdir -p models

COPY models /home/aakapatel/.gazebo/models

WORKDIR /home/aakapatel/ 

RUN mkdir -p .ignition

COPY ignition_models /home/aakapatel/.ignition/

WORKDIR /home/aakapatel/catkin_workspaces/exploration_ws/

STOPSIGNAL SIGTERM

CMD sudo service ssh start && /bin/bash
