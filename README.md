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

## Install Conda Virtual Environment

Robinion software framework was developed using Python version 3. We prefer conda virtual environment to use the framework. For creating conda virtual environment please follow this guide.


### 1. Install Anaconda

```bash
wget https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh
chmod +x Anaconda3-2020.07-Linux-x86_64.sh
./Anaconda3-2020.07-Linux-x86_64.sh
source ~/.bashrc
```

### 2. Create Virtual Environment

```bash
git clone https://github.com/ekorudiawan/conda-environment.git
cd conda-environment
conda env create -f robinion-ros-env.yml
```

### 3. Post Installation

Set default conda environment

```bash
echo "conda activate ros" >> ~/.bashrc
source ~/.bashrc
```

## Install Dependencies

### 1. python3-pykdl

python3-pykdl module is used for calculating kinematics of the robot. We prefer build the python-pykdl from the sources, rather than installing from rosdep. Please follow this guide for building the python-pykdl module.

**Deactivate conda environment**

```bash
conda deactivate
```

Install python-pykdl from source

```bash
wget https://github.com/orocos/orocos_kinematics_dynamics/archive/v1.4.0.tar.gz
tar -xvzf v1.4.0.tar.gz
cd orocos_kinematics_dynamics-1.4.0/python_orocos_kdl/
mkdir build && cd build
cmake -DMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.8.2 ..
make -j12
```

Copy module to conda environment

```bash
cp PyKDL.so ~/anaconda3/envs/ros/lib/python3.8/site-packages/
```