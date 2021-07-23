#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
	
  // ***
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // *** Creating publisher to broadcast if robot is at goal marker
  ros::Publisher goal_reach_pub = n.advertise<std_msgs::UInt8>("/goal_reached", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // pick-up action
  move_base_msgs::MoveBaseGoal goal;		

  // goal reach status
  std_msgs::UInt8 status_msg;  

  // set up the frame parameters
  // pick-goal setup
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  // n.getParam("/pick_up_loc/tx", goal.target_pose.pose.position.x);
  // n.getParam("/pick_up_loc/ty", goal.target_pose.pose.position.y);
  // n.getParam("/pick_up_loc/tz", goal.target_pose.pose.position.z);
  // n.getParam("/pick_up_loc/qx", goal.target_pose.pose.orientation.x);
  // n.getParam("/pick_up_loc/qy", goal.target_pose.pose.orientation.y);
  // n.getParam("/pick_up_loc/qz", goal.target_pose.pose.orientation.z);
  // n.getParam("/pick_up_loc/qw", goal.target_pose.pose.orientation.w);
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.y = -2;
  goal.target_pose.pose.orientation.w = 1;

  // Send the goal position and orientation for the robot for pick-up location
  ROS_INFO("Sending pick-up goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Pick-up location reached");
    ROS_INFO("Now picking up package");
    // publish goal-reach status
    status_msg.data = 1;
    // ROS_INFO("The pick-up goal-reach status is %d", status_msg.data);
    goal_reach_pub.publish(status_msg);

    ros::Duration(5).sleep();

      // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // pick-up action
    move_base_msgs::MoveBaseGoal goal_2;		

    // goal reach status
    std_msgs::UInt8 status_msg;  

    // set up the frame parameters
    // pick-goal setup
    goal_2.target_pose.header.frame_id = "map";
    goal_2.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal_2.target_pose.pose.position.x = -2;
    goal_2.target_pose.pose.position.y = -2;
    goal_2.target_pose.pose.orientation.w = 1;

    // Send the goal position and orientation for the robot for pick-up location
    ROS_INFO("Sending drop-off goal");
    ac.sendGoal(goal_2);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Drop-off location reached");}
    else {
      ROS_INFO("Failed to reach drop-off location");
    }
  }
  else {
    ROS_INFO("Failed to reach pick-up location");
  }

  return 0;
}
