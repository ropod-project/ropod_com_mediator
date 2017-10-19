#include <iostream>
#include <fstream>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/ropod_sem_waypoint_list.h>

// This node sends out a hard coded sem_waypoint_list as a test case for the december demo
int main(int argc, char **argv)
{

	/* Initialize ROS framework */
	ros::init(argc, argv, "test_publisher_for_ropod_sem_waypoint_list");
	ros::NodeHandle node;
	ros::Rate loop_rate(1);

	ros::Publisher zyreToRosCommandsPuplisher = node.advertise<ropod_ros_msgs::ropod_sem_waypoint_list>("ropod_commands", 100);
	
	// fill the msg
	ropod_ros_msgs::ropod_sem_waypoint_list rosMsg;
	ropod_ros_msgs::ropod_sem_waypoint sem_waypoint;
	sem_waypoint.command = "GOTO";
	sem_waypoint.location = "START";
	rosMsg.sem_waypoint.push_back(sem_waypoint);
	sem_waypoint.command = "GOTO";
	sem_waypoint.location = "MOBIDIK";
	rosMsg.sem_waypoint.push_back(sem_waypoint);
	sem_waypoint.command = "GOTO";
	sem_waypoint.location = "ELEVATOR";
	rosMsg.sem_waypoint.push_back(sem_waypoint);
	
	ROS_INFO("Ready.");
	while (ros::ok())
	{
		zyreToRosCommandsPuplisher.publish(rosMsg);
		ROS_INFO("Published cmd.");
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
