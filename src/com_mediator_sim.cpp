#include <ropod_com_mediator/com_mediator_sim.h>
#include <chrono>
#include <thread>

std::string getArgValue(int argc, char **argv, const std::string& argName="debug_mode", const std::string& defaultArgVal="")
{
    std::string argVal = defaultArgVal;
    for(unsigned int i = 0; i < argc; i++)
    {
        std::string argument = std::string(argv[i]);
        std::size_t found = argument.find(argName + std::string("="));
        if (found != std::string::npos)
        {
            argVal = argument.substr(argument.find("=") + 1);
            break;
        }
    }
    return argVal;
}

bool getModeStatus(int argc, char **argv, const std::string& modeName="debug_mode")
{
    std::string argVal = getArgValue(argc, argv, modeName);
    return argVal == "true";
}

ComMediatorSim::ComMediatorSim(int argc, char **argv, const std::string& robot_name)
    : ComMediator(argc, argv, robot_name, true)
{
}

ComMediatorSim::~ComMediatorSim() 
{
}

void ComMediatorSim::setupRos()
{
    nh.reset(new ros::NodeHandle("~"));

    setupTaskPublisher();
    setupGotoSubscriber();

    /** Disable for simulations */
    // setupDockSubscriber();
    // setupElevatorRequestPubSub();
    // setupRobotPoseSubscriber();
    // setupRobotSubareaSubscriber();
    // setupExecuteExperiementActionClient();
    // setupExperimentTransitionSubscriber();

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

void ComMediatorSim::tearDownRos()
{
    ropod_task_pub.shutdown();
    progress_goto_sub.shutdown();

    /** Disable for simulations */
    // progress_dock_sub.shutdown();
    // elevator_request_sub.shutdown();
    // elevator_request_reply_pub.shutdown();
    // experiment_client->cancelAllGoals();
}


///////////////////////
// Zyre to ROS methods
///////////////////////

void ComMediatorSim::publishTaskMessage(const ropod_ros_msgs::Task& task_msg)
{
    if (!task_msg.robot_actions.empty())
    {
        ROS_INFO("[ComMediator] Filtering out all actions other than GOTO before publishing the task to task executor!");
        ropod_ros_msgs::Task msg_copy = task_msg;
        for (auto itr = msg_copy.robot_actions.begin(); itr != msg_copy.robot_actions.end(); )
        {
            if (itr->type != "GOTO")
            {
                ROS_INFO("\t[ComMediator] Removed Action (%s) of type (%s)", itr->action_id.c_str(), itr->type.c_str());
                itr = msg_copy.robot_actions.erase(itr);
            }
            else
                ++itr;
        }
        ComMediator::publishTaskMessage(msg_copy);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_mediator");
    bool debug = getModeStatus(argc, argv, "debug_mode");
    bool sim = getModeStatus(argc, argv, "sim_mode");
    std::string robot_name = getArgValue(argc, argv, "robot_name", "ropod_001");

    ComMediator* com_mediator = NULL;
    if (sim)
    {
        ROS_INFO("[ComMediator] Creating a ComMediator simulation object");
        com_mediator = new ComMediatorSim(argc, argv, robot_name);
    }
    else if (debug)
    {
        ROS_INFO("[ComMediator] Creating a ComMediator debug object");
        com_mediator = new ComMediator(argc, argv, robot_name, true);
    }
    else
    {
        ROS_INFO("[ComMediator] Creating a ComMediator object");
        com_mediator = new ComMediator(argc, argv, robot_name, false);
    }

    if (com_mediator != NULL)
        ROS_INFO("[ComMediator] Ready.");
    else
        ROS_ERROR("[ComMediator] Could not create a ComMediator");

    com_mediator->run();
    while(ros::ok())
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    com_mediator->stop();

    delete com_mediator;
    com_mediator = NULL;
    return 0;
}
