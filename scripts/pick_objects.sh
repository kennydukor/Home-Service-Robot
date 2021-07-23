#!/bin/sh

# Launch turtlebot in my custom world
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/robot_world.world" &

sleep 15

# Launch gmapping_demo.launch to perform mapping task
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/robot_world_map.yaml" & 

sleep 10

# Launch rviz for visualization
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Launch pick_objects node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../config/virtual_loc_config.yaml;
rosrun pick_objects pick_objects "
