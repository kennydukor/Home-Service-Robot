# Home-Service-Robot

Udacity Robotics Software Engineer Nanodegree Program [Project 5]


**Project Goals**

The goal of this project was to design a robot's environment in gazebo and program the home-service robot that will map it's environment and autonomously navigate to pre-specified pickup and drop-off locations. For this one needed to:

* Design robot's environment with the Building Editor in Gazebo.
* Teleoperate the robot and manually test SLAM.
* Use the ROS navigation stack and manually command the robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
* Write a pick_objects node that commands the robot to move to the desired pickup and drop off zones.
* Write an add_markers node that subscribes to the robot odometry and publishes pick-up and drop-off markers to rviz.
* modify pick_objects node and add_markers node to establish communication between them, to complete desired home service robot implementation

### Prerequisites
(since I have not tested on multiple platforms, and versions, I am listing only the configuration I used)

* Ubuntu 16.04 OS with default make (>=4.1) and g++/gcc (>=5.4) packages
* Gazebo >= 7.0
* ROS Kinetic
* following ROS packages were used and the process of obtaining them is detailed below:
	* [gmapping](http://wiki.ros.org/gmapping)
	* [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
	* [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
	* [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

### Clone and Build

Since the folder presented here comprises only of ROS package, one needs to first create a catkin workspace and initialize it. Also, note that the official ROS packaged are already included here, but their dependencies need to be installed; steps for this are given below.

Within your `home` directory, execute the following:

```
mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
```

Within `~/catkin_ws/src/` download or clone folders of this repository:

```
$ cd catkin_ws/src/
$ git clone https://github.com/kennydukor/Home-Service-Robot.git
```

Install dependencies:

```
$ rosdep -i install gmapping -y
$ rosdep -i install turtlebot_teleop -y
$ rosdep -i install turtlebot_rviz_launchers -y
$ rosdep -i install turtlebot_gazebo -y
```

`NOTE`: If any of the official packages give error, I recommed you delete associated folder and clone with src folder using appropriate line from here:

```
$ git clone https://github.com/ros-perception/slam_gmapping.git  
$ git clone https://github.com/turtlebot/turtlebot.git  
$ git clone https://github.com/turtlebot/turtlebot_interactions.git  
$ git clone https://github.com/turtlebot/turtlebot_simulator.git
```

Go back to catkin workspace and build it

```
$ cd catkin_ws/
$ catkin_make
```

### Launch specific application and visualize

From the `catkin_ws/` directory run the following commands:  

#### Testing SLAM

``` bash
$ ./src/script/test_slam.sh
```  
<p align="center"><img src="screenshots/slam_5.gif"></p>

#### Testing Navigation

``` bash
$ ./src/script/test_navigation.sh
```  
<p align="center"><img src="./screenshots/image_1.gif"></p> 

#### Pick_objects

``` bash
$ ./src/script/pick_objects.sh
```  
<p align="center"><img src="./screenshots/image_1.gif"></p> 

#### Add Markers

``` bash
$ ./src/script/add_marker.sh
```  
<p align="center"><img src="./screenshots/image_1.gif"></p>

#### Home Service

```bash
$ ./src/script/home_service.sh
```
<p align="center"><img src="./screenshots/image_1.gif"></p>
