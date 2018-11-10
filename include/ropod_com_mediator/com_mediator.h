#ifndef COM_MEDIATOR_H
#define COM_MEDIATOR_H

#include <iostream>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h> //TF
#include <tf2_ros/transform_listener.h> //TF
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

/* Zyre + JSON includes */
#include "zyre.h"
#include "ZyreBaseCommunicator.h"
#include <json/json.h>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <ropod_ros_msgs/Task.h>
#include <ropod_ros_msgs/Status.h>
#include <ropod_ros_msgs/ElevatorRequest.h>
#include <ropod_ros_msgs/ElevatorRequestReply.h>

/* Remote experiment messages */
#include <ropod_ros_msgs/ExecuteExperiment.h>
#include <ropod_ros_msgs/CommandFeedback.h>
#include <ropod_ros_msgs/ExperimentFeedback.h>

#include <ftsm_base.h>


using namespace ftsm;


class ComMediator : public FTSMBase, public ZyreBaseCommunicator
{
private:
    int argc;
    char **argv;
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<ros::Rate> rate;

    // task execution
    ros::Publisher ropod_task_pub;
    ros::Subscriber progress_goto_sub;
    ros::Subscriber progress_dock_sub;
    ros::Subscriber elevator_request_sub;
    ros::Publisher elevator_request_reply_pub;

    // remote experiments
    ros::Publisher experiment_pub;
    ros::Subscriber command_feedback_sub;
    ros::Subscriber experiment_feedback_sub;

    tf2_ros::Buffer tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    Json::CharReaderBuilder json_builder;

    double minSendDurationInSec;
    std::string tfFrameId;
    std::string tfFrameReferenceId;
    std::string robotName;
    std::string zyreGroupName;
    ros::Time lastSend;

    void parseAndPublishTaskMessage(const Json::Value &root);
    void parseAndPublishElevatorReply(const Json::Value &root);
    void parseAndPublishExperimentMessage(const Json::Value &root);

    void startNode();
    void createSubcribersPublishers();
    void stopNode();

public:
    ComMediator(int argc, char**argv);
    virtual ~ComMediator();

    virtual void recvMsgCallback(ZyreMsgContent *msgContent);

    // task execution
    void progressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::ConstPtr &ros_msg);
    void progressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::ConstPtr &ros_msg);
    void elevatorRequestCallback(const ropod_ros_msgs::ElevatorRequest::ConstPtr &ros_msg);

    // remote experiments
    void commandFeedbackCallback(const ropod_ros_msgs::CommandFeedback::ConstPtr &ros_msg);
    void experimentFeedbackCallback(const ropod_ros_msgs::ExperimentFeedback::ConstPtr &ros_msg);

    void tfCallback();


    std::string init();
    std::string configuring();
    std::string ready();
    std::string running();
    std::string recovering();
};

#endif /* COM_MEDIATOR_H */
