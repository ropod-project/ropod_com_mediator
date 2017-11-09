#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

/* ROPOD ROS messages */
#include "ropod_ros_msgs/sem_waypoint_cmd.h"
#include "ropod_ros_msgs/ropod_sem_waypoint_list.h"

ros::Publisher control_pub;
ros::Subscriber sem_waypoint_sub;
std::map<std::string, geometry_msgs::PoseStamped> waypoints;

std::map<std::string, geometry_msgs::PoseStamped> readWaypoints(std::string waypoint_file)
{
    std::map<std::string, geometry_msgs::PoseStamped> waypoints;
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(waypoint_file);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
        return waypoints;
    }

    for (YAML::const_iterator it=root.begin(); it != root.end(); ++it)
    {
        std::string name = it->begin()->first.as<std::string>();
        YAML::Node node = it->begin()->second;

        std::vector<double> position = node["position"].as<std::vector<double>>();
        std::vector<double> orientation = node["orientation"].as<std::vector<double>>();

        geometry_msgs::PoseStamped point;
        point.pose.position.x = position[0];
        point.pose.position.y = position[1];
        point.pose.position.z = position[2];
        point.pose.orientation.x = orientation[0];
        point.pose.orientation.y = orientation[1];
        point.pose.orientation.z = orientation[2];
        point.pose.orientation.w = orientation[3];

        waypoints[name] = point;
    }

    return waypoints;
}

void groundSemanticWaypoints(const ropod_ros_msgs::ropod_sem_waypoint sem_pt, ropod_ros_msgs::ropod_control_primitive& control_primitive)
{
    geometry_msgs::PoseStamped pt;

    // needs to be repaced wth querying the WM
    if (sem_pt.command == "GOTO")
    {
        if (waypoints.find(sem_pt.location) != waypoints.end())
        {
            control_primitive.behaviour = "GOTO";
            control_primitive.poses.push_back(waypoints[sem_pt.location]);
        }
        else
        {
            ROS_WARN("Unknown command!");
        }
    }
    else if (sem_pt.command == "ENTER_ELEVATOR")
    {
        control_primitive.behaviour = "TAKE_ELEVATOR";
        control_primitive.poses.push_back(waypoints["INSIDE_ELEVATOR1"]);
        control_primitive.poses.push_back(waypoints["OUTSIDE_ELEVATOR"]);
    }
    else if (sem_pt.command == "EXIT_ELEVATOR")
    {
        control_primitive.behaviour = "GOTO";
        control_primitive.poses.push_back(waypoints["OUTSIDE_ELEVATOR"]);
    }
    else if (sem_pt.command == "PAUSE")
    {
        ROS_WARN("PAUSE not implemented yet");
    }
    else if (sem_pt.command == "RESUME")
    {
        ROS_WARN("RESUME not implemented yet");
    }
    else
    {
        ROS_WARN("Unknown command!");
    }
}

// read the list of received semantic waypoints, query for their metric resolution, and send the metric waypoints to controller
void newCmdReceived(const ropod_ros_msgs::ropod_sem_waypoint_list::ConstPtr& msg)
{
    ROS_INFO("New command received.");
    //control_pub = node.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 100);

    ropod_ros_msgs::sem_waypoint_cmd cmd;

    cmd.header = msg->header;
    cmd.sem_waypoint = "demo";

    // read the list of received semantic waypoints
    for(std::vector<ropod_ros_msgs::ropod_sem_waypoint>::const_iterator it = msg->sem_waypoint.begin(); it != msg->sem_waypoint.end(); ++it)
    {
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

    std::string semantic_waypoint_file;
    node.getParam("/semantic_waypoint_file", semantic_waypoint_file);
    waypoints = readWaypoints(semantic_waypoint_file);

    control_pub = node.advertise<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd_topic", 100);
    sem_waypoint_sub = node.subscribe("semantic_waypoint_topic", 100, newCmdReceived);

    ROS_INFO("Ready.");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
