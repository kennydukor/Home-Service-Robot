#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

int marker_stage = 0;

/* main function */
int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_virtual_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// Set our initial shape type to be a cube
	int shape = visualization_msgs::Marker::CUBE;

	ROS_INFO("Subscribed to desired goal-position");
  while (ros::ok()) {
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_virtual_markers";
    marker.id = 0;

    // Set the marker type. 
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    // threshold = marker.scale.x;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    switch (marker_stage)
    {
      // publish pick-up marker
      case 0:
        {
          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;
  	      marker.pose.position.x = 1;
  	      marker.pose.position.y = 0;
  	      marker.pose.orientation.w = 1;
          marker.lifetime = ros::Duration();
          ROS_INFO("Showing marker at pickup location");
          ros::Duration(5).sleep();
          marker_stage = 1;
          break;
        } 
        case 1:
          {
            // Remove marker to simulate pickup 
            sleep(2);
            marker.action = visualization_msgs::Marker::DELETE;
            ROS_INFO("Moving marker to drop-off location");
            marker_stage = 2;
            break;
          }

        case 2: 
          {
            ros::Duration(5).sleep();
            ROS_INFO("Marker at drop-off location");
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 2;
            marker.pose.orientation.w = 1;
            marker.lifetime = ros::Duration();
            break;
          }
    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    //Ensure subscriber receives the message
    ros::spinOnce();

    r.sleep();
  } 

}
