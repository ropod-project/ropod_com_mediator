#include <ropod_com_mediator/com_mediator.h>
#include <sstream>

ComMediator::ComMediator()
    : ZyreBaseCommunicator("com_mediator",
                           std::vector<std::string>{std::string("ROPOD")},
                           std::vector<std::string>{std::string("TASK")}, false),
    nh("~"),
    tfListener(tfBuffer)
{
    nh.param<std::string>("tfFrameId", tfFrameId, "base_link");
    nh.param<std::string>("tfFrameReferenceId", tfFrameReferenceId, "map");
    nh.param<std::string>("robotName", robotName, "ropod_1");
    nh.param<double>("minSendDurationInSec", minSendDurationInSec, 1.0);
    nh.param<std::string>("zyreGroupName", zyreGroupName, "ROPOD");
    lastSend = ros::Time::now();
    tfBuffer._addTransformsChangedListener(boost::bind(&ComMediator::tfCallback, this)); // call on change

    ropod_task_pub = nh.advertise<ropod_ros_msgs::Task>("task", 1);
    progress_goto_sub = nh.subscribe<ropod_ros_msgs::TaskProgressGOTO>("ropod_task_feedback/goto", 1,
                                        &ComMediator::progressGOTOCallback, this);
    progress_dock_sub = nh.subscribe<ropod_ros_msgs::TaskProgressDOCK>("ropod_task_feedback/dock", 1,
                                        &ComMediator::progressDOCKCallback, this);

    elevator_request_sub = nh.subscribe<ropod_ros_msgs::ElevatorRequest>("elevator_request", 1,
                                        &ComMediator::elevatorRequestCallback, this);
    elevator_request_reply_pub = nh.advertise<ropod_ros_msgs::ElevatorRequestReply>("elevator_request_reply", 1);

    // remote experiments
    this->experiment_pub = nh.advertise<ropod_ros_msgs::ExecuteExperiment>("/ropod/execute_experiment", 1);
    this->command_feedback_sub = nh.subscribe<ropod_ros_msgs::CommandFeedback>("/ropod/cmd_feedback", 1,
                                        &ComMediator::commandFeedbackCallback, this);
    this->experiment_feedback_sub = nh.subscribe<ropod_ros_msgs::ExperimentFeedback>("/ropod/experiment_feedback", 1,
                                        &ComMediator::experimentFeedbackCallback, this);
}

ComMediator::~ComMediator() { }

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
            if (root["payload"]["robotId"] != robotName)
            {
                return;
            }

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

///////////////////////
// ROS to Zyre methods
///////////////////////
void ComMediator::progressGOTOCallback(const ropod_ros_msgs::TaskProgressGOTO::ConstPtr &ros_msg)
{
    Json::Value msg;

    msg["header"]["type"] = "TASK-PROGRESS";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msg_id"] = generateUUID();
    msg["header"]["timestamp"] = getTimeStamp();

    msg["payload"]["metamodel"] = "ropod-demo-progress-schema.json";
    msg["payload"]["taskId"] = ros_msg->task_id;
    // msg["payload"]["robotId"] = ros_msg->robot_id;
    msg["payload"]["robotId"] = robotName;
    msg["payload"]["actionId"] = ros_msg->action_id;
    msg["payload"]["actionType"] = ros_msg->action_type;
    msg["payload"]["status"]["areaName"] = ros_msg->area_name;
    msg["payload"]["status"]["actionStatus"] = ros_msg->status.status_code;
    msg["payload"]["status"]["taskStatus"] = "ongoing";
    msg["payload"]["status"]["sequenceNumber"] = ros_msg->sequenceNumber;
    msg["payload"]["status"]["totalNumber"] = ros_msg->totalNumber;
    // msg["payload"]["status"]["currentAction"] = ros_msg->currentAction;

    std::stringstream feedbackMsg("");
    feedbackMsg << msg;
    this->shout(feedbackMsg.str(), zyreGroupName);
}

void ComMediator::progressDOCKCallback(const ropod_ros_msgs::TaskProgressDOCK::ConstPtr &ros_msg)
{
    Json::Value msg;

    msg["header"]["type"] = "TASK-PROGRESS";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    msg["header"]["msg_id"] = uuid_str;
    zuuid_destroy (&uuid);
    //int64_t now = zclock_time();
    char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
    msg["header"]["timestamp"] = timestr;
    zstr_free(&timestr);


    msg["payload"]["metamodel"] = "ropod-demo-progress-schema.json";
    msg["payload"]["taskId"] = ros_msg->task_id;
    // msg["payload"]["robotId"] = ros_msg->robot_id;
    msg["payload"]["robotId"] = robotName;
    msg["payload"]["actionId"] = ros_msg->action_id;
    msg["payload"]["actionType"] = ros_msg->action_type;
    msg["payload"]["status"]["areaName"] = ros_msg->area_name;
    msg["payload"]["status"]["actionStatus"] = ros_msg->status.status_code;
    msg["payload"]["status"]["taskStatus"] = ros_msg->task_status.status_code;
    // msg["payload"]["status"]["currentAction"] = ros_msg->currentAction;

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
    msg["header"]["msg_id"] = uuid_str;
    zuuid_destroy (&uuid);
    //int64_t now = zclock_time();
    char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
    msg["header"]["timestamp"] = timestr;
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

void ComMediator::tfCallback()
{
    ros::Duration maxTFCacheDuration = ros::Duration(10.0); // [s]
    geometry_msgs::TransformStamped transform;
    double roll,yaw,pitch;
    try
    {
        transform = tfBuffer.lookupTransform(tfFrameReferenceId, tfFrameId, ros::Time(0), ros::Duration(3.0));
        tf2::Quaternion q;
        q.setX(transform.transform.rotation.x); // there is certainly a more elegant way than this ...
        q.setY(transform.transform.rotation.y);
        q.setZ(transform.transform.rotation.z);
        q.setW(transform.transform.rotation.w);
        tf2::Matrix3x3 m;
        m.setRotation(q);
        m.getRPY(roll,pitch, yaw);
    }
    catch (tf2::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    if ( (ros::Time::now() - transform.header.stamp) > maxTFCacheDuration )
    { //simply ignore outdated TF frames
        ROS_WARN("TF found for %s. But it is outdated. Skipping it.", tfFrameId.c_str());
        return;
    }
    ROS_DEBUG("TF found for %s.", tfFrameId.c_str());


    if (ros::Time::now() - lastSend > ros::Duration(minSendDurationInSec))
    { // throttld down pose messages
        ROS_DEBUG("Sending Zyre message.");

        /* Convert to JSON Message */
        Json::Value msg;
    //	{
    //	  "header":{
    //	    "type":"RobotPose2D",
    //	    "metamodel":"ropod-msg-schema.json",
    //	    "msg_id":"5073dcfb-4849-42cd-a17a-ef33fa7c7a69"
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
        msg["header"]["type"] = "RobotPose2D";
        msg["header"]["metamodel"] = "ropod-msg-schema.json";
        msg["header"]["msg_id"] = this->generateUUID();

        char *timestr = zclock_timestr (); // TODO: this is not ISO 8601
        msg["header"]["timestamp"] = timestr;
        zstr_free(&timestr);

        msg["payload"]["metamodel"] = "ropod-demo-robot-pose-2d-schema.json";
        msg["payload"]["robotId"] = robotName;
        msg["payload"]["pose"]["referenceId"] = tfFrameReferenceId;
        msg["payload"]["pose"]["x"] = transform.transform.translation.x;
        msg["payload"]["pose"]["y"] = transform.transform.translation.y;
        msg["payload"]["pose"]["theta"] = yaw;

        std::stringstream poseMsg("");
        poseMsg << msg;
        this->shout(poseMsg.str());
        lastSend = ros::Time::now();
    }
}

void ComMediator::commandFeedbackCallback(const ropod_ros_msgs::CommandFeedback::ConstPtr &ros_msg)
{
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-COMMAND-FEEDBACK";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msg_id"] = this->generateUUID();
    msg["header"]["timestamp"] = ros_msg->stamp.toSec();

    msg["payload"]["metamodel"] = "ropod-command-feedback-schema.json";
    msg["payload"]["robotId"] = this->robotName;
    msg["payload"]["command"] = ros_msg->command_name;
    msg["payload"]["state"] = ros_msg->state;

    std::stringstream jsonMsg("");
    jsonMsg << msg;
    this->shout(jsonMsg.str(), zyreGroupName);
}

void ComMediator::experimentFeedbackCallback(const ropod_ros_msgs::ExperimentFeedback::ConstPtr &ros_msg)
{
    Json::Value msg;
    msg["header"]["type"] = "ROBOT-EXPERIMENT-FEEDBACK";
    msg["header"]["metamodel"] = "ropod-msg-schema.json";
    msg["header"]["msg_id"] = this->generateUUID();
    msg["header"]["timestamp"] = ros_msg->stamp.toSec();

    msg["payload"]["metamodel"] = "ropod-experiment-feedback-schema.json";
    msg["payload"]["robotId"] = this->robotName;
    msg["payload"]["experimentType"] = ros_msg->experiment_type;
    msg["payload"]["result"] = ros_msg->result;

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
    task.task_id = root["payload"]["taskId"].asString();
    task.start_time = root["payload"]["earliest_start_time"].asDouble();
    task.start_time = root["payload"]["latest_start_time"].asDouble();
    task.cart_type = root["payload"]["cart_type"].asString();
    task.cart_id = root["payload"]["cart_id"].asString();
    task.cart_id = root["payload"]["estimated_duration"].asDouble();
    task.cart_id = root["payload"]["priority"].asInt();
    for (auto robot_id : root["payload"]["team_robot_ids"])
    {
        std::string robot_id_str = robot_id.asString();
        task.team_robot_ids.push_back(robot_id_str);
    }
    const Json::Value &action_list = root["payload"]["actions"];
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
                area.area_id = areas[j]["id"].asString();
                area.name = areas[j]["name"].asString();
                area.type = areas[j]["type"].asString();
                area.floor_number = areas[j]["floor_number"].asInt();
                const Json::Value &wp = areas[j]["waypoints"];
                for (int k = 0; k < wp.size(); k++)
                {
                    ropod_ros_msgs::Waypoint waypoint;
                    waypoint.semantic_id = wp[k]["semantic_id"].asString();
                    waypoint.area_id = wp[k]["area_id"].asString();
                    waypoint.floor_number = wp[k]["floor_number"].asInt();
                    waypoint.waypoint_pose.position.x = wp[k]["x"].asDouble();
                    waypoint.waypoint_pose.position.y = wp[k]["y"].asDouble();
                    waypoint.waypoint_pose.orientation.w = 1.0;
                    area.waypoints.push_back(waypoint);
                }
                action.areas.push_back(area);
            }
            const Json::Value &wp = action_list[i]["waypoints"];
            for (int k = 0; k < wp.size(); k++)
            {
                ropod_ros_msgs::Waypoint waypoint;
                waypoint.semantic_id = wp[k]["semantic_id"].asString();
                waypoint.area_id = wp[k]["area_id"].asString();
                waypoint.floor_number = wp[k]["floor_number"].asInt();
                waypoint.waypoint_pose.position.x = wp[k]["x"].asInt();
                waypoint.waypoint_pose.position.y = wp[k]["y"].asInt();
                waypoint.waypoint_pose.orientation.w = 1.0;
                action.waypoints.push_back(waypoint);
            }
        }
        else if (action.type == "REQUEST_ELEVATOR")
        {
            action.start_floor = action_list[i]["startFloor"].asInt();
            action.goal_floor = action_list[i]["goalFloor"].asInt();
            std::stringstream ss;
            ss << action_list[i] << std::endl;
        }
        task.robot_actions.push_back(action);
    }
    ropod_task_pub.publish(task);

}

void ComMediator::parseAndPublishElevatorReply(const Json::Value &root)
{
    ropod_ros_msgs::ElevatorRequestReply reply;
    reply.query_id = root["payload"]["queryId"].asString();
    reply.query_success = root["payload"]["querySuccess"].asBool();
    reply.elevator_id = root["payload"]["elevatorId"].asInt();
    reply.elevator_waypoint = root["payload"]["elevatorWaypoint"].asString();
    elevator_request_reply_pub.publish(reply);
}

void ComMediator::parseAndPublishExperimentMessage(const Json::Value &root)
{
    std::string experiment_type = root["payload"]["experimentType"].asString();
    ROS_INFO("[com_mediator] Received '%s' experiment request", experiment_type.c_str());
    ropod_ros_msgs::ExecuteExperiment experiment_msg;
    experiment_msg.experiment_type = experiment_type;
    this->experiment_pub.publish(experiment_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ropod_com_mediator");
    ComMediator com_mediator;

    ros::NodeHandle nh("~");
    double loop_rate = 10.0;
    nh.param<double>("loop_rate", loop_rate, 10.0);
    ros::Rate r(loop_rate);
    while (ros::ok() && !zsys_interrupted)
    {
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}
