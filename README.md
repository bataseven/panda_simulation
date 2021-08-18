# Pick and Place Simulation In Gazebo Using The Panda Robot From Franka Emika

## Introduction
This repository contains the necessary packages to perform a simple pick and place task using ROS, Moveit! motion planning framework and Gazebo Physics Engine. Simulation is performed on a machine that natively runs Ubuntu 20.04.2 LTS. Noetic distribution of ROS is used. Note that the content in this repository is built on top of [this](https://github.com/erdalpekel/panda_simulation) repository.

![Panda in Gazebo](https://github.com/bataseven/panda_simulation/blob/master/assets/tables_and_the_robot.jpg?raw=true "Panda in Gazebo")

## How to get started
Install the necessary ROS packages of Panda robot:
```
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```

Run the following commands to create the workspace and install additional packages:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/bataseven/panda_simulation.git
git clone https://github.com/bataseven/panda_moveit_config.git
git clone --branch simulation https://github.com/bataseven/franka_ros.git
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
catkin_make
```
After building the workspace, source the *setup.bash*. To avoid sourcing it whenever a new terminal opens, add it to the *bashrc*. If the workspace is located in the $HOME folder, run the following command:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## How to use
To run the simulation enter the command:
```
roslaunch panda_simulation simulation.launch
```
Start the demo by running the node in a seperate thread
```
rosrun panda_simulation grasping_demo.py
```
## Simulation in action
![Pick & Place](https://github.com/bataseven/panda_simulation/blob/master/assets/simulation.gif?raw=true)
