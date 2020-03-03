#ifndef COM_MEDIATOR_H
#define COM_MEDIATOR_H

#include <iostream>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>

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
#include <ropod_ros_msgs/TransitionList.h>

/* Remote experiment action */
#include <ropod_ros_msgs/ExecuteExperimentAction.h>

#include <ftsm_base.h>


using namespace ftsm;


class ComMediator : public FTSMBase, public ZyreBaseCommunicator
{
protected:
    int argc;
    char **argv;
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<ros::Rate> rate;
    bool debug_mode;

    // task execution
    ros::Publisher ropod_task_pub;
    ros::Subscriber progress_goto_sub;
    ros::Subscriber progress_dock_sub;
    ros::Subscriber elevator_request_sub;
    ros::Publisher elevator_request_reply_pub;
    ros::Publisher remote_command_pub;

    // remote monitoring
    ros::Subscriber experiment_transition_sub;

    std::unique_ptr<actionlib::SimpleActionClient<ropod_ros_msgs::ExecuteExperimentAction>> experiment_client;

    ros::Subscriber robot_pose_sub;
    ros::Subscriber robot_subarea_sub;

    Json::CharReaderBuilder json_builder;

    std::string robotName;
    std::string zyreGroupName;
    std::string robotSubAreaName;

    void parseTaskMessage(const Json::Value &root, ropod_ros_msgs::Task& task_msg);
    virtual void publishTaskMessage(const ropod_ros_msgs::Task& task_msg);
    void parseAndPublishElevatorReply(const Json::Value &root);
    void parseAndPublishExperimentMessage(const Json::Value &root);
    void parseAndPublishCommandMessage(const Json::Value &root);

    void setupTaskPublisher();
    void setupRemoteCommandPublisher();
    void setupGotoSubscriber();
    void setupDockSubscriber();
    void setupElevatorRequestPubSub();
    void setupRobotPoseSubscriber();
    void setupRobotSubareaSubscriber();
    void setupExecuteExperiementActionClient();
    void setupExperimentTransitionSubscriber();

public:
    ComMediator(int argc, char**argv, const std::string& robot_name, bool debug);
    virtual ~ComMediator();

    virtual void recvMsgCallback(ZyreMsgContent *msgContent);
    virtual void sendMessageStatus(const std::string &msgId, bool status);

    // task execution
    void progressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::ConstPtr &ros_msg);
    void progressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::ConstPtr &ros_msg);
    void elevatorRequestCallback(const ropod_ros_msgs::ElevatorRequest::ConstPtr &ros_msg);

    // remote experiments
    void experimentFeedbackCallback(const ropod_ros_msgs::ExecuteExperimentFeedbackConstPtr &ros_msg);
    void experimentResultCallback(const actionlib::SimpleClientGoalState& state,
                                  const ropod_ros_msgs::ExecuteExperimentResultConstPtr &ros_msg);
    void experimentTransitionCallback(const ropod_ros_msgs::TransitionList::ConstPtr &ros_msg);

    void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void robotSubAreaCallback(const std_msgs::String::ConstPtr &subarea_msg);

    virtual std::string init();
    virtual std::string configuring();
    virtual std::string ready();
    virtual std::string running();
    virtual std::string recovering();

    virtual void setupRos();
    virtual void tearDownRos();
};

#endif /* COM_MEDIATOR_H */
