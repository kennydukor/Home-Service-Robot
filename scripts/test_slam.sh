#!/bin/sh

# Launch turtlebot in my custom world
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/robot_world.world" &

sleep 10

# Launch gmapping_demo.launch to perform mapping task
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo gmapping_demo.launch " & 

sleep 5

# Launch rviz for visualization
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" 

sleep 5

# Launch teleops for navigation
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch"
