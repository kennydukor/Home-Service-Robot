#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Start = 0; Pickup = 1; Drop = 2 NOTE data is published for add_marker use
int robot_stage = 0;
std_msgs::Int32 msgs;

/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // *** Creating publisher to broadcast if robot is at goal marker
  ros::Publisher robot_status_pub = n.advertise<std_msgs::Int32>("/goal_status", 10);
  
  msgs.data = robot_stage;
  robot_status_pub.publish(msgs);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // pick-up action
  move_base_msgs::MoveBaseGoal goal;		

  // goal reach status
  //std_msgs::UInt8 status_msg;  

  // set up the frame parameters
  // pick-goal setup
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.y = 0;
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
    robot_stage = 1;
    msgs.data = robot_stage;
    robot_status_pub.publish(msgs);

    ros::Duration(5).sleep();

      // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // pick-up action
    move_base_msgs::MoveBaseGoal goal_2;		

    // set up the frame parameters
    // pick-goal setup
    goal_2.target_pose.header.frame_id = "map";
    goal_2.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal_2.target_pose.pose.position.x = 0;
    goal_2.target_pose.pose.position.y = 2;
    goal_2.target_pose.pose.orientation.w = 1;

    // Send the goal position and orientation for the robot for pick-up location
    ROS_INFO("Sending drop-off goal");
    ac.sendGoal(goal_2);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Drop-off location reached");
      robot_stage = 2;
      msgs.data = robot_stage;
      robot_status_pub.publish(msgs);
    }
    else {
      ROS_INFO("Failed to reach drop-off location");
    }
  }
  else {
    ROS_INFO("Failed to reach pick-up location");
  }

  return 0;
}
