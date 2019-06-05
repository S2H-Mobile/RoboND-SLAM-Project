[//]: # (Image References)

[screenshot_3D]: ./writeup/images/3D_point_cloud_kitchen_dining_settings_default.jpg

# RoboND-SLAM-Project, *Map my world*
This project is part of the Robotics Nanodegree. A mobile robot performs SLAM in two simulated indoor environments. It generates 2D and 3D maps.

![3D map of the kitchen dining world.][screenshot_3D]

## Contents
- The [writeup report](https://github.com/S2H-Mobile/RoboND-SLAM-Project/blob/master/writeup/writeup_map_my_world.pdf).
- The ROS package `slam_rover` contains the URDF description of a mobile rover equipped with a Kinect RGB-D camera and a Hokuyo laser range finder, the SDF definitions of two Gazebo worlds, and launch files for localization, mapping, visualization and teleoperating the rover.

## Setup
### Ubuntu 16.04
1. Install ROS.
2. Install [rtabmap_ros](https://github.com/introlab/rtabmap_ros).

### Jetson TX2
1. Install ROS.

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
Note: Skip this step if ROS is already installed.

2. Install dependencies.
```bash
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
$ sudo apt-get remove ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
```

3. Install RTAB-Map.
```bash
$ cd ~
$ git clone https://github.com/introlab/rtabmap.git rtabmap
$ cd rtabmap/build
$ cmake ..
$ make
$ sudo make install
```
4. Install [rtabmap_ros](https://github.com/introlab/rtabmap_ros).
5. Create `.gazebo` folder: Open gazebo and then close it.

6. Add model collision adjustments.
```bash
$ curl -L https://s3-us-west-1.amazonaws.com/udacity-robotics/Term+2+Resources/P3+Resources/models.tar.gz | tar zx -C ~/.gazebo/
```

## Usage
1. Create a catkin workspace. See [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
2. Clone this repo into the `src` folder.
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/S2H-Mobile/RoboND-SLAM-Project.git
```
3. Build the catkin workspace.
```bash
$ cd ~/catkin_ws/
$ catkin_make
```
4. Source the bash setup file.
```bash
$ source ~/catkin_ws/devel/setup.bash
```
5. Run the main launch file.
```bash
$ roslaunch slam_rover rtab_run.launch
```
This will start four ROS nodes,
- the Gazebo simulation,
- the mapping node,
- the RViz window,
- and the teleoperation node for the rover.

When running the project on the Jetson TX2, it is recommended to use the maximum clock speed via the `jetson_clocks.sh` script. 

The nodes can also be launched individually in separate terminals.
```bash
$ roslaunch slam_rover world.launch
```
```bash
$ roslaunch slam_rover mapping.launch
```
```bash
$ roslaunch slam_rover rviz.launch
```
```bash
$ roslaunch slam_rover teleop.launch
```

Alternatively, run the bash script `./rtab_run` and select either the `kitchen_dining.world` or the `cafe.world`.

5. Navigate the rover to perform SLAM in the specified environment.
6. Evaluate the resulting map database with the RTAB-Map Database Viewer.
```bash
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```
