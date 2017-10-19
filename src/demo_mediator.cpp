#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "ropod_ros_msgs/sem_waypoint_cmd.h"
#include "ropod_ros_msgs/ropod_sem_waypoint_list.h"


ros::Publisher control_pub;
ros::Subscriber sem_waypoint_sub;

// read the list of received semantic waypoints, query for their metric resolution, and send the metric waypoints to controller
void NewCmdReceived(const ropod_ros_msgs::ropod_sem_waypoint_list::ConstPtr& msg)
{
	ropod_ros_msgs::sem_waypoint_cmd cmd;
	ropod_ros_msgs::ropod_control_primitive prim;
	geometry_msgs::Point pt;
	
	cmd.header = msg->header;
	cmd.sem_waypoint = "demo";
	
	// read the list of received semantic waypoints
	for(std::vector<ropod_ros_msgs::ropod_sem_waypoint>::const_iterator it = msg->sem_waypoint.begin(); it != msg->sem_waypoint.end(); ++it) {
		
		//ignore header for now
		
		// set behaviour used to reach the waypoint
		prim.behaviour = "test_behaviour";
		
		// set sequence of metric points to reach semantic waypoint
		//add for loop over query result
		pt.x = 0;
		pt.y = 0;
		pt.z = 0;
		prim.point.push_back(pt);
		cmd.primitive.push_back(prim);

	}
	
	// send new metric waypoints to controller
	control_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "WM_mediator");
  ros::NodeHandle node;
//  ros::Rate loop_rate(10);
  
  ros::Publisher control_pub = node.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 100);
  ros::Subscriber sem_waypoint_sub = node.subscribe("ropod_commands", 100, NewCmdReceived);
  
  ROS_INFO("Ready.");
  ros::spin();
/* while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }*/

  return 0;
}