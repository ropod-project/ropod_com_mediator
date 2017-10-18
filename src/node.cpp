#include <iostream>
#include <fstream>

/* ROS includes */
#include <ros/ros.h>

int main(int argc, char **argv)
{

	/* Initialize ROS framework */
	ros::init(argc, argv, "ropod_com_mediator");
	ros::NodeHandle node;

	/* parameters */



	ROS_INFO("Ready.");
	ros::spin();

	return 0;
}
