#include <ropod_com_mediator/com_mediator.h>
#include <sstream>
#include <chrono>
#include <thread>
#include <csignal>

bool nodeKilled = false;

std::string getEnv(const std::string &var)
{
    char const* value = std::getenv(var.c_str());
    if (value == NULL)
    {
        std::cerr << "Warning: environment variable " << var << " not set!" << std::endl;
        return std::string();
    }
    else
    {
        return std::string(value);
    }
}

ComMediator::ComMediator(int argc, char **argv)
    : FTSMBase("com_mediator", {"roscore"},
               {{"heartbeat", {{"roscore", "ros/ros_master_monitor"}}}}),
    ZyreBaseCommunicator(getEnv("ROPOD_ID"),
                         false, "", true, false), // print msgs, network interface, acknowledge, startImmediately
    argc(argc),
    argv(argv)
{
    std::vector<std::string> sendAcknowledgementFor;
    sendAcknowledgementFor.push_back("TASK");
    sendAcknowledgementFor.push_back("ROBOT-EXPERIMENT-REQUEST");
    this->setSendAcknowledgementFor(sendAcknowledgementFor);

    std::vector<std::string> expectAcknowledgementFor;
    expectAcknowledgementFor.push_back("ROBOT-ELEVATOR-CALL-REQUEST");
    this->setExpectAcknowledgementFor(expectAcknowledgementFor);

    robotName = getEnv("ROPOD_ID");
    std::map<std::string, std::string> headers;
    headers["name"] = robotName + std::string("_com_mediator");
    this->setHeaders(headers);

    // start zyre node
    this->startZyreNode();
}

ComMediator::~ComMediator() { }

std::string ComMediator::init()
{
    this->setupRos();
    this->joinGroup(zyreGroupName);
    return FTSMTransitions::INITIALISED;
}

std::string ComMediator::configuring()
{
    return FTSMTransitions::DONE_CONFIGURING;
}
std::string ComMediator::ready()
{
    return FTSMTransitions::RUN;
}

void ComMediator::setupRos()
{
    ROS_INFO("Initialising ROS node");
    ros::init(argc, argv, "com_mediator");
    nh.reset(new ros::NodeHandle("~"));

    ROS_INFO("[com_mediator] Creating a task publisher");
    ropod_task_pub = nh->advertise<ropod_ros_msgs::Task>("task", 1);

    ROS_INFO("[com_mediator] Creating a ropod_task_feedback/goto subscriber");
    progress_goto_sub = nh->subscribe<ropod_ros_msgs::TaskProgressGOTO>("ropod_task_feedback/goto", 1,
                                        &ComMediator::progressGOTOCallback, this);

    ROS_INFO("[com_mediator] Creating a ropod_task_feedback/dock subscriber");
    progress_dock_sub = nh->subscribe<ropod_ros_msgs::TaskProgressDOCK>("ropod_task_feedback/dock", 1,
                                        &ComMediator::progressDOCKCallback, this);

    ROS_INFO("[com_mediator] Creating an elevator_request subscriber");
    elevator_request_sub = nh->subscribe<ropod_ros_msgs::ElevatorRequest>("elevator_request", 1,
                                        &ComMediator::elevatorRequestCallback, this);

    ROS_INFO("[com_mediator] Creating an elevator_request_reply publisher");
    elevator_request_reply_pub = nh->advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_request_reply", 1);

    ROS_INFO("[com_mediator] Creating a robot_pose subscriber");
    robot_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("robot_pose", 1, &ComMediator::robotPoseCallback, this);

    ROS_INFO("[com_mediator] Creating a /ropod/execute_experiment action client");
    this->experiment_client = std::unique_ptr<actionlib::SimpleActionClient<ropod_ros_msgs::ExecuteExperimentAction>>
                              (new actionlib::SimpleActionClient<ropod_ros_msgs::ExecuteExperimentAction>
                                  ("/ropod/execute_experiment", true));

    ROS_INFO("[com_mediator] Creating a /ropod/transition_list subscriber");
    this->experiment_transition_sub = nh->subscribe<ropod_ros_msgs::TransitionList>(
            "/ropod/transition_list", 1, &ComMediator::experimentTransitionCallback, this);


    ROS_INFO("[com_mediator] Reading ROS parameters");
    if (robotName.empty())
    {
        nh->param<std::string>("robotName", robotName, "ropod_1");
    }
    nh->param<std::string>("zyreGroupName", zyreGroupName, "ROPOD");
    double loop_rate;
    nh->param<double>("loop_rate", loop_rate, 10.0);
    rate.reset(new ros::Rate(loop_rate));
}

void ComMediator::tearDownRos()
{
    ropod_task_pub.shutdown();
    progress_goto_sub.shutdown();
    progress_dock_sub.shutdown();
    elevator_request_sub.shutdown();
    elevator_request_reply_pub.shutdown();
    experiment_client->cancelAllGoals();
}

std::string ComMediator::running()
{
    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(this->depend_statuses[DependMonitorTypes::HEARTBEAT]
                                                               ["roscore"]
                                                               ["ros/ros_master_monitor"].c_str(), root);
    bool master_available = root["status"].asBool();

    if(!zsys_interrupted && master_available)
    {
        ros::spinOnce();
        rate->sleep();
        return FTSMTransitions::CONTINUE;
    }
    else
    {
        return FTSMTransitions::RECOVER;
    }

}

std::string ComMediator::recovering()
{
    this->recoverFromPossibleDeadRosmaster();
    return FTSMTransitions::DONE_RECOVERING;
}

void ComMediator::recvMsgCallback(ZyreMsgContent *msgContent)
{
    if (msgContent->event == "SHOUT")
    {
        std::stringstream msg;
        msg << msgContent->message;

        Json::Value root;
        std::string errors;
        bool ok = Json::parseFromStream(json_builder, msg, &root, &errors);

        if (root.isMember("header"))
        {
            if (root["header"]["type"] == "TASK")
            {
                this->parseAndPublishTaskMessage(root);
            }
            else if (root["header"]["type"] == "ROBOT-ELEVATOR-CALL-REPLY")
            {
                this->parseAndPublishElevatorReply(root);
            }
            else if (root["header"]["type"] == "ROBOT-EXPERIMENT-REQUEST")
            {
                this->parseAndPublishExperimentMessage(root);
            }
        }
    }
}

void ComMediator::sendMessageStatus(const std::string &msgId, bool status)
{
    if (status)
        ROS_INFO_STREAM("Sending message: " << msgId << " succeeded");
    else
        ROS_ERROR_STREAM("Sending message: " << msgId << " failed");

    // TODO: what to do here if sending a message fails?
    // need to call some sort of recovery action
}

///////////////////////
// ROS to Zyre methods
///////////////////////
void ComMediator::progressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::ConstPtr &ros_msg)
{
    Json::Value msg;

    msg["header"]["type"] = "TASK-STATUS";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msgId"] = generateUUID();
    msg["header"]["timestamp"] = getTimeStamp();

    msg["payload"]["metamodel"] = "ropod-demo-progress-schema.json";
    msg["payload"]["taskId"] = ros_msg->task_id;
    // msg["payload"]["robotId"] = ros_msg->robot_id;
    msg["payload"]["robotId"] = robotName;
    // msg["payload"]["taskStatus"] = "ongoing";
    msg["payload"]["taskStatus"] = ros_msg->task_status.status_code;
    msg["payload"]["taskProgress"]["actionId"] = ros_msg->action_id;
    msg["payload"]["taskProgress"]["actionStatus"] = ros_msg->status.status_code;
    msg["payload"]["taskProgress"]["actionType"] = ros_msg->action_type;
    msg["payload"]["taskProgress"]["area"] = ros_msg->area_name;

    std::stringstream feedbackMsg("");
    feedbackMsg << msg;
    this->shout(feedbackMsg.str(), zyreGroupName);
}

void ComMediator::progressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::ConstPtr &ros_msg)
{
    Json::Value msg;

    msg["header"]["type"] = "TASK-STATUS";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    msg["header"]["msgId"] = uuid_str;
    zuuid_destroy (&uuid);
    //int64_t now = zclock_time();
    char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
    msg["header"]["timestamp"] = timestr;
    zstr_free(&timestr);

    msg["payload"]["metamodel"] = "ropod-demo-progress-schema.json";
    msg["payload"]["taskId"] = ros_msg->task_id;
    // msg["payload"]["robotId"] = ros_msg->robot_id;
    msg["payload"]["robotId"] = robotName;
    msg["payload"]["taskStatus"] = ros_msg->task_status.status_code;
    msg["payload"]["taskProgress"]["actionId"] = ros_msg->action_id;
    msg["payload"]["taskProgress"]["actionStatus"] = ros_msg->status.status_code;
    msg["payload"]["taskProgress"]["actionType"] = ros_msg->action_type;
    msg["payload"]["taskProgress"]["area"] = ros_msg->area_name;

    std::stringstream feedbackMsg("");
    feedbackMsg << msg;
    this->shout(feedbackMsg.str(), zyreGroupName);
}

void ComMediator::elevatorRequestCallback(const ropod_ros_msgs::ElevatorRequest::ConstPtr &ros_msg)
{
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-ELEVATOR-CALL-REQUEST";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    msg["header"]["msgId"] = uuid_str;
    zuuid_destroy (&uuid);
    //int64_t now = zclock_time();
    char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
    msg["header"]["timestamp"] = timestr;
    Json::Value receiverIds;
    receiverIds.append("resource_manager");
    msg["header"]["receiverIds"] = receiverIds;
    zstr_free(&timestr);


    msg["payload"]["metamodel"] = "ropod-elevator-cmd-schema.json";
    msg["payload"]["queryId"] = ros_msg->query_id;
    msg["payload"]["robotId"] = robotName;
    msg["payload"]["command"] = ros_msg->command;
    msg["payload"]["startFloor"] = ros_msg->start_floor;
    msg["payload"]["goalFloor"] = ros_msg->goal_floor;
    msg["payload"]["taskId"] = ros_msg->task_id;
    msg["payload"]["load"] = ros_msg->load;

    std::stringstream jsonMsg("");
    jsonMsg << msg;
    this->shout(jsonMsg.str(), zyreGroupName);
}

void ComMediator::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    double roll,yaw,pitch;
    tf2::Quaternion q;
    q.setX(pose_msg->pose.orientation.x); // there is certainly a more elegant way than this ...
    q.setY(pose_msg->pose.orientation.y);
    q.setZ(pose_msg->pose.orientation.z);
    q.setW(pose_msg->pose.orientation.w);
    tf2::Matrix3x3 m;
    m.setRotation(q);
    m.getRPY(roll,pitch, yaw);

    ROS_DEBUG("Sending Zyre message.");

    /* Convert to JSON Message */
    Json::Value msg;
//	{
//	  "header":{
//	    "type":"ROBOT-POSE-2D",
//	    "metamodel":"ropod-msg-schema.json",
//	    "msgId":"5073dcfb-4849-42cd-a17a-ef33fa7c7a69"
//	  },
//	  "payload":{
//	    "metamodel":"ropod-demo-robot-pose-2d-schema.json",
//	    "robotId":"ropod_0",
//	    "pose":{
//	      "referenceId":"basement_map",
//	      "x":10,
//	      "y":20,
//	      "theta":3.1415
//	    }
//	  }
//	}
    msg["header"]["type"] = "ROBOT-POSE-2D";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msgId"] = this->generateUUID();

    char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
    msg["header"]["timestamp"] = timestr;
    zstr_free(&timestr);

    msg["payload"]["metamodel"] = "ropod-demo-robot-pose-2d-schema.json";
    msg["payload"]["robotId"] = robotName;
    msg["payload"]["pose"]["referenceId"] = pose_msg->header.frame_id;
    msg["payload"]["pose"]["x"] = pose_msg->pose.position.x;
    msg["payload"]["pose"]["y"] = pose_msg->pose.position.y;
    msg["payload"]["pose"]["theta"] = yaw;

    std::stringstream poseMsg("");
    poseMsg << msg;
    this->shout(poseMsg.str());
}

void ComMediator::experimentFeedbackCallback(const ropod_ros_msgs::ExecuteExperimentFeedbackConstPtr &ros_msg)
{
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-COMMAND-FEEDBACK";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msgId"] = this->generateUUID();
    msg["header"]["timestamp"] = ros_msg->stamp.toSec();

    msg["payload"]["metamodel"] = "ropod-command-feedback-schema.json";
    msg["payload"]["robotId"] = this->robotName;
    msg["payload"]["command"] = ros_msg->command_name;
    msg["payload"]["state"] = ros_msg->state;

    std::stringstream jsonMsg("");
    jsonMsg << msg;
    this->shout(jsonMsg.str(), zyreGroupName);
}

void ComMediator::experimentResultCallback(const actionlib::SimpleClientGoalState& state,
                                           const ropod_ros_msgs::ExecuteExperimentResultConstPtr &ros_msg)
{
    if (!ros_msg)
    {
        ROS_WARN_STREAM("experimentResultCallback got NULL result");
        return;
    }
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-EXPERIMENT-FEEDBACK";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msgId"] = this->generateUUID();
    msg["header"]["timestamp"] = ros_msg->stamp.toSec();

    msg["payload"]["metamodel"] = "ropod-experiment-feedback-schema.json";
    msg["payload"]["robotId"] = this->robotName;
    msg["payload"]["experimentType"] = ros_msg->experiment_type;
    msg["payload"]["result"] = ros_msg->result;

    std::stringstream jsonMsg("");
    jsonMsg << msg;
    this->shout(jsonMsg.str(), zyreGroupName);
}

void ComMediator::experimentTransitionCallback(const ropod_ros_msgs::TransitionList::ConstPtr &ros_msg)
{
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-EXPERIMENT-SM";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msgId"] = this->generateUUID();
    msg["header"]["timestamp"] = ros::Time::now().toSec();

    msg["payload"]["metamodel"] = "ropod-experiment-transition-schema.json";
    msg["payload"]["robotId"] = this->robotName;
    Json::Value transitions;
    for (int i = 0; i < ros_msg->transitions.size(); i++){
        Json::Value transition;
        transition["source"] = ros_msg->transitions[i].source;
        transition["target"] = ros_msg->transitions[i].target;
        transition["name"] = ros_msg->transitions[i].name;
        transitions.append(transition);
    }
    msg["payload"]["transitions"] = transitions;

    std::stringstream jsonMsg("");
    jsonMsg << msg;
    this->shout(jsonMsg.str(), zyreGroupName);
}

///////////////////////
// Zyre to ROS methods
///////////////////////
void ComMediator::parseAndPublishTaskMessage(const Json::Value &root)
{
    ropod_ros_msgs::Task task;
    task.task_id = root["payload"]["id"].asString();
    ROS_INFO_STREAM("[com_mediator] Received task " << task.task_id);
    task.start_time = root["payload"]["start_time"].asDouble();
    task.finish_time = root["payload"]["finish_time"].asDouble();
    task.earliest_start_time = root["payload"]["earliest_start_time"].asDouble();
    task.latest_start_time = root["payload"]["latest_start_time"].asDouble();
    task.load_type = root["payload"]["loadType"].asString();
    task.load_id = root["payload"]["loadId"].asString();
    task.estimated_duration = root["payload"]["estimated_duration"].asDouble();
    task.priority = root["payload"]["priority"].asInt();
    for (auto robot_id : root["payload"]["team_robot_ids"])
    {
        std::string robot_id_str = robot_id.asString();
        task.team_robot_ids.push_back(robot_id_str);
    }
    if (!root["payload"]["robot_actions"].isMember(robotName))
    {
        ROS_INFO_STREAM("No actions for " << robotName << ". Ignoring task");
        return;
    }
    const Json::Value &action_list = root["payload"]["robot_actions"][robotName];
    ROS_INFO_STREAM("Task has " << action_list.size() << " actions for " << robotName);
    for (int i = 0; i< action_list.size(); i++)
    {
        ropod_ros_msgs::Action action;
        action.action_id = action_list[i]["id"].asString();
        action.type = action_list[i]["type"].asString();
        if (action.type == "GOTO" || action.type == "DOCK" || action.type == "UNDOCK")
        {
            action.execution_status = action_list[i]["execution_status"].asString();
            action.estimated_duration = action_list[i]["eta"].asFloat();
            const Json::Value &areas = action_list[i]["areas"];
            for (int j = 0; j < areas.size(); j++)
            {
                ropod_ros_msgs::Area area;
                area.id = areas[j]["id"].asString();
                area.name = areas[j]["name"].asString();
                area.type = areas[j]["type"].asString();
                area.floor_number = areas[j]["floorNumber"].asInt();
                const Json::Value &wp = areas[j]["subAreas"];
                for (int k = 0; k < wp.size(); k++)
                {
                    ropod_ros_msgs::SubArea sub_area;
                    sub_area.name = wp[k]["name"].asString();
                    sub_area.id = wp[k]["id"].asString();
                    sub_area.type = wp[k]["type"].asString();
                    // if capacity is not specified, it appears as an empty string
                    try
                    {
                        sub_area.capacity = wp[k]["capacity"].asInt();
                    }
                    catch (std::exception &e)
                    {
                    }
                    area.sub_areas.push_back(sub_area);
                }
                action.areas.push_back(area);
            }
        }
        else if (action.type == "REQUEST_ELEVATOR")
        {
            action.start_floor = action_list[i]["start_floor"].asInt();
            action.goal_floor = action_list[i]["goal_floor"].asInt();
            action.elevator.elevator_id = action_list[i]["elevator_id"].asInt();
        }
        task.robot_actions.push_back(action);
    }

    ropod_task_pub.publish(task);

}

void ComMediator::parseAndPublishElevatorReply(const Json::Value &root)
{
    ropod_ros_msgs::ElevatorRequestReply reply;
    reply.query_id = root["payload"]["queryId"].asString();
    ROS_INFO_STREAM("[com_mediator] Received elevator reply: " << reply.query_id);
    reply.query_success = root["payload"]["querySuccess"].asBool();
    reply.elevator_id = root["payload"]["elevatorId"].asInt();
    reply.elevator_door_id = root["payload"]["elevatorDoorId"].asInt();
    elevator_request_reply_pub.publish(reply);
}

void ComMediator::parseAndPublishExperimentMessage(const Json::Value &root)
{
    std::string experiment_type = root["payload"]["experimentType"].asString();
    ROS_INFO("[com_mediator] Received '%s' experiment request", experiment_type.c_str());

    ropod_ros_msgs::ExecuteExperimentGoal experiment_msg;
    experiment_msg.experiment_type = experiment_type;
    this->experiment_client->sendGoal(experiment_msg,
                                      boost::bind(&ComMediator::experimentResultCallback, this, _1, _2),
                                      actionlib::SimpleActionClient<ropod_ros_msgs::ExecuteExperimentAction>::SimpleActiveCallback(),
                                      boost::bind(&ComMediator::experimentFeedbackCallback, this, _1));
}

int main(int argc, char **argv)
{
    ComMediator com_mediator(argc, argv);
    com_mediator.run();
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (nodeKilled)
        {
            com_mediator.stop();
            break;
        }
    }
	return 0;
}
