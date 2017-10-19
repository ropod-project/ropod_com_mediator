#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/* ROPOD ROS messages */
#include "ropod_ros_msgs/sem_waypoint_cmd.h"
#include "ropod_ros_msgs/ropod_sem_waypoint_list.h"

ros::Publisher control_pub;
ros::Subscriber sem_waypoint_sub;

void groundSemanticWaypoints(const ropod_ros_msgs::ropod_sem_waypoint sem_pt, ropod_ros_msgs::ropod_control_primitive& control_primitive)
{
	geometry_msgs::Point pt;

	// needs to be repaced wth querying the WM
	if (sem_pt.command == "GOTO"){
		if (sem_pt.location == "START"){
			// set behaviour used to reach the waypoint
			control_primitive.behaviour = "GOTO";
			pt.x = 0;
			pt.y = 0;
			pt.z = 0;
			control_primitive.point.push_back(pt);
		} else if (sem_pt.location == "MOBIDIK"){
			control_primitive.behaviour = "GOTO";
			pt.x = 1;
			pt.y = 0;
			pt.z = 0;
			control_primitive.point.push_back(pt);
		} else if (sem_pt.location == "ELEVATOR"){
			control_primitive.behaviour = "GOTO";
			pt.x = 1;
			pt.y = 1;
			pt.z = 0;
			control_primitive.point.push_back(pt);
			pt.x = 2;
			pt.y = 2;
			pt.z = 0;
			control_primitive.point.push_back(pt);
		} else {
			ROS_WARN("Unknown command!");
		}
	} else if (sem_pt.command == "ENTER_ELEVATOR"){
		ROS_INFO("ENTER_ELEVATOR not implemented yet");
	} else if (sem_pt.command == "EXIT_ELEVATOR"){
		ROS_INFO("EXIT_ELEVATOR not implemented yet");
	} else if (sem_pt.command == "PAUSE"){
		ROS_INFO("PAUSE not implemented yet");
	} else if (sem_pt.command == "RESUME"){
		ROS_INFO("RESUME not implemented yet");
	} else {
		ROS_WARN("Unknown command!");
	}
}

// read the list of received semantic waypoints, query for their metric resolution, and send the metric waypoints to controller
void NewCmdReceived(const ropod_ros_msgs::ropod_sem_waypoint_list::ConstPtr& msg)
{
	ROS_INFO("New command received.");
	//control_pub = node.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 100);
	
	ropod_ros_msgs::sem_waypoint_cmd cmd;
	
	cmd.header = msg->header;
	cmd.sem_waypoint = "demo";
	
	// read the list of received semantic waypoints
	for(std::vector<ropod_ros_msgs::ropod_sem_waypoint>::const_iterator it = msg->sem_waypoint.begin(); it != msg->sem_waypoint.end(); ++it) {
		
		ropod_ros_msgs::ropod_control_primitive prim;
		groundSemanticWaypoints(*it, prim);
		cmd.primitive.push_back(prim);
	}
	
	std::cout  << "[DEBUG]   msg = "  << cmd << std::endl;
	// send new metric waypoints to controller
	control_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "WM_mediator");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);
  
  control_pub= node.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 100);
  sem_waypoint_sub = node.subscribe("ropod_commands", 100, NewCmdReceived);
  
  ROS_INFO("Ready.");
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}