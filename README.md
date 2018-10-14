# Traversability Mapping and Motion Planning

This repository contains code for a traversability mapping and motion plannign system for ROS compatible UGVs. The system takes in point cloud from a Velodyne VLP-16 Lidar and outputs a traversability map for autonomous navigation in real-time. A demonstration of the system can be found [here](https://www.youtube.com/watch?v=bQt2OMlQ8tY) -> https://www.youtube.com/watch?v=bQt2OMlQ8tY


## Get Started

- Install [ROS](http://www.ros.org/install/).

- Install [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).

- Install [ROS Navigation stack](http://wiki.ros.org/navigation). You can install it by running ```sudo apt-get install ros-indigo-navigation```. If you are using other versions of ROS, replace indigo in the command with your ROS version.


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/Traversability_Mapping.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## Run the System (in simulation)

1. Run the launch file:
```
roslaunch traversability_mapping offline.launch
```

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
Notes: our system only needs /velodyne_points for input from bag files. However, a 3D SLAM method usually needs /imu/data.

## Run the System (with real robot)

Run the launch file:
```
roslaunch traversability_mapping online.launch
```

## Cite *Traversability_Mapping*

Thank you for citing our paper if you use any of this code: 
```
@inproceedings{traversability2018,
  title={Bayesian Generalized Kernel Inference for Terrain Traversability Mapping},
  author={Tixiao Shan, Kevin Doherty, Jinkun Wang and Brendan Englot},
  booktitle={In Proceedings of the 2nd Annual Conference on Robot Learning},
  year={2018}
}
```
