# Traversability Mapping and Motion Planning

This repository contains code for a traversability mapping and motion plannign system for ROS compatible UGVs. The system takes in point cloud from a Velodyne VLP-16 Lidar and outputs a traversability map for autonomous navigation in real-time. A demonstration of the system can be found [here](https://www.youtube.com/watch?v=bQt2OMlQ8tY) -> https://www.youtube.com/watch?v=bQt2OMlQ8tY


## Get Started

- Install [ROS](http://www.ros.org/install/).

- The system needs an external source to offer transformation between ```/map``` and ```/base_link```. Any SLAM methods that can work with Velodyne lidar should do the trick, i.e. LOAM and Google Cartographer. Many SLAM methods have been tested with our system. Here we **assume** you have LOAM installed since it offers the best localization performance. The LOAM package can be found by searching "loam_velodyne" on GitHub. Once you download the loam_velodyne package, remember to add these two lines to its launch file, which is called ```loam_velodyne.launch``` in ```/loam_velodyne/launch``` folder. The reason we need these two lines is because the world and the robot frame in loam_vedloyne is called ```/camera_init``` and ```/camera``` respectively. These two lines of code will establish two static transformations that connect ```/map``` and ```/base_link```. If you are using Google Cartographer, then you don't need these two lines because it offers transformation between ```/map``` and ```/base_link``` by default.
```
<node pkg="tf" type="static_transform_publisher" name="camera_init_to_map" args="0 0 0.3 1.570795 0 1.570795 /map /camera_init 10" />
<node pkg="tf" type="static_transform_publisher" name="base_link_to_camera" args="0 0 0 -1.570795 -1.570795 0 /camera /base_link 10" />
```
- ROS navigation stack also needs to be installed for autonomous navigation. You can install it by running ```sudo apt-get install ros-indigo-navigation```. If you are using other versions of ROS, replace indigo in the command with your ROS version.

- Download some ROS bag files if you don't have a robot right now. Some bag files can be found [here](https://drive.google.com/drive/folders/0B0hyPClc-TrQdHdwQXhMa3R5WGs?usp=sharing).

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/Traversability_Mapping.git
cd ..
catkin_make
```
Note that sometimes elevation_msgs package is not compiled before traversability_mapping package and an error will occur. You just need to run ```catkin_make``` again to finish compiling.

## Run the System (in simulation)

1. Run the launch file:
```
roslaunch traversability_mapping traversability_mapping_offline.launch
```
Notes: this launch file will launch all the essential packages for traversability mapping and motion planning. Note that it assumes you are using LOAM to give robot position. If you are using other SLAM methods, please replace the corresponding launch file for it.

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
Notes: our system only needs /velodyne_points for input from bag files. However, a 3D SLAM method usually also needs /imu/data.

## Run the System (with real robot)

Run the launch file:
```
roslaunch traversability_mapping traversability_mapping_online.launch
```

## Use the System

You can just simply drag the blue goal flag, which is an interactive marker in Rviz, to a mapped position. A path will be generated and displayed. If you are using a real robot, the robot will be driven by ROS navigation stack to follow the path.