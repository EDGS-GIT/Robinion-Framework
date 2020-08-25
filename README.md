# Robinion User Guide

## Install Ubuntu and ROS

Robinion software framework used Ubuntu 20.04 with ROS Noetic Ninjemys. Ubuntu 20.04 and ROS Noetic Ninjemys installation guide refer to these links:

* https://help.ubuntu.com/lts/installation-guide/index.html
* http://wiki.ros.org/noetic/Installation/Ubuntu

It is highly recomended to install **ros-noetic-desktop-full** to run the Robinion software framework.


## Install Ubuntu Packages

```bash
sudo apt update && sudo apt install -y git
```

## Setup Virtual Environment

Robinion software framework was developed using Python version 3. We prefer conda virtual environment to use the framework. For creating conda virtual environment please follow this guide.


### 1. Install Anaconda

```bash
wget https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh
chmod +x Anaconda3-2020.07-Linux-x86_64.sh
./Anaconda3-2020.07-Linux-x86_64.sh
source ~/.bashrc
```

### 2. Create Conda Virtual Environment

```bash
git clone https://github.com/EDGS-GIT/conda_environment.git
cd conda_environment
conda env create -f robinion-ros-env.yml
```

### 3. Post Installation

Set default conda environment

```bash
echo "conda activate ros" >> ~/.bashrc
source ~/.bashrc
```

## Install Dependencies

### 1. PyKDL

PyKDL module is Python version of liborocos-kdl which used for calculating kinematics of the robot. We prefer build the PyKDL from the sources. Please follow this link for building the PyKDL module.

https://github.com/EDGS-GIT/python_orocos_kdl

## Setup Robinion ROS Framework

Robinion Framework

```bash
mkdir ~/robinion_ws && cd ~/robinion_ws
git clone https://github.com/EDGS-GIT/Robinion-Framework.git
mv Robinion-Framework src
```

DynamixelSDK

```bash
cd ~/robinion_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK 
git checkout noetic-devel
```

Robotis Framework

```bash
cd ~/robinion_ws/src
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
```

Robotis Framework Msgs

```bash
cd ~/robinion_ws/src
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
```

Robotis-Math

```bash
cd ~/robinion_ws/src
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
```

kdl_parser

```bash
cd ~/robinion_ws/src
wget https://github.com/ros/kdl_parser/archive/1.13.2.tar.gz
tar -xvzf 1.13.2.tar.gz
rm -rf 1.13.2.tar.gz
```

urdf_parser_py

```bash
cd ~/robinion_ws/src
wget https://github.com/ros/urdf_parser_py/archive/0.4.3.tar.gz
tar -xvzf 0.4.3.tar.gz
rm -rf 0.4.3.tar.gz
```

**Compile Robinion Framework**

```bash
cd ~/robinion_ws/src/robinion_manager
mkdir include
cd ~/robinion_ws/
catkin_make -j12
source devel/setup.bash
```

**TODO**
1. Add new servo device
2. Fix include folder error

## Testing Robinion Framework

### Simulation Mode

#### 1. Testing walking_control_module

Launch robinion_walking_module

```bash
roslaunch robinion_walking_module test_walking_module.launch
```

Open Gazebo Client

```bash
gzclient
```

Publish topic to robinion_walking_module

**Initialize**

This command will make the robot go to initial pose

```bash
rostopic pub /robinion/walking_module/state robinion_msgs/WalkingState "state: 'init'"
```

**Start Walking**

This command will make the robot start walking (stepping in place)

```bash
rostopic pub /robinion/walking_module/state robinion_msgs/WalkingState "state: 'start
```

**Send Walking Command**

This command will make the robot walking forward. For changing direction, just change value of $cmd_x$, $cmd_y$, and $cmd_alpha$.

```bash
rostopic pub /robinion/walking_module/command robinion_msgs/WalkingCommand "cmd_x: 0.05
cmd_y: 0.0 
cmd_alpha: 0.0"
```

**Stop Walking**

This command will make the robot stop walking.

```bash
rostopic pub /robinion/walking_module/state robinion_msgs/WalkingState "state: 'stop'"
```

### Robot Mode

**TO DO**
