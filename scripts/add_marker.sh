#!/bin/sh

# Launch turtlebot in my custom world
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 0.171998 -y 0.682663 -z -0.000247 -R 0.000778 -P -0.014417 -Y 1.603426';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/robot_world.world" &

sleep 10

# Launch gmapping_demo.launch to perform mapping task
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/robot_world_map.yaml" & 

sleep 5

# Launch rviz for visualization
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Launch add_marker node
# I didnt use virtual_loc_config.yaml
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../config/virtual_loc_config.yaml;
rosrun add_makers add_markers"
