#ifndef COM_MEDIATOR_H
#define COM_MEDIATOR_H

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h> //TF
#include <tf2_ros/transform_listener.h> //TF
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

/* Zyre + JONS includes */
#include "zyre.h"
#include <json/json.h>
#include <iostream>

#include <ropod_ros_msgs/ropod_demo_status_update.h>
#include <ropod_ros_msgs/Task.h>
#include "ZyreBaseCommunicator.h"

class ComMediator : ZyreBaseCommunicator
{
private:
    ros::NodeHandle nh;
    ros::Publisher ropod_commands_pub;
    ros::Subscriber progress_sub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    Json::CharReaderBuilder json_builder;

    double minSendDurationInSec;
    std::string tfFrameId;
    std::string tfFrameReferenceId;
    std::string robotName;
    std::string zyreGroupName;
    ros::Time lastSend;

public:
    ComMediator();
    virtual ~ComMediator();

    virtual void recvMsgCallback(ZyreMsgContent *msgContent);
    void progressCallback(const ropod_ros_msgs::ropod_demo_status_update::ConstPtr &ros_msg);
    void tfCallback();
};

#endif /* COM_MEDIATOR_H */
