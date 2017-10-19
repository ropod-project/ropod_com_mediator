#include "ros/ros.h"
#include "ropod_ros_msgs/sem_waypoint_cmd.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "WM_mediator");
  ros::NodeHandle n;
  ros::Publisher control_pub = n.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 1000);
  ros::Rate loop_rate(10);
  
  int count = 0;
  while (ros::ok())
  {
    ropod_ros_msgs::sem_waypoint_cmd cmd;
	ropod_ros_msgs::ropod_control_primitive prim;
	geometry_msgs::Point pt;
	
	prim.behaviour = "test_behaviour";
	pt.x = 0;
	pt.y = 0;
	pt.z = 0;
	prim.point.push_back(pt);
	
	cmd.sem_waypoint = "test";
	cmd.primitive.push_back(prim);

    //ROS_INFO("%s", msg.data.c_str());

    control_pub.publish(cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}