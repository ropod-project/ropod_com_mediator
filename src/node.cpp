#include <iostream>
#include <fstream>
#include <sstream>

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

/* ROPOD ROS messages */
#include <ropod_ros_msgs/ropod_sem_waypoint_list.h>

ros::Publisher zyreToRosPuplisher;
ros::Publisher zyreToRosCommandsPuplisher;

/* TF */
tf2_ros::Buffer tfListener;
tf2_ros::TransformListener* tfUpdateListener;
ros::Time lastSend;

/* Parameters */
std::string tfFrameId = "base_link";
std::string tfFrameReferenceId = "map";
std::string robotName = "ropod_0";
std::string zyreGroupName = "ROPOD";
double minSendDurationInSec = 1.0;

static void
chat_actor (zsock_t *pipe, void *args)
{
    zyre_t *node = zyre_new ((char *) args);
    if (!node)
        return;                 //  Could not create new node

    zyre_start (node);
    zyre_join (node, zyreGroupName.c_str());
    zsock_signal (pipe, 0);     //  Signal "ready" to caller

    bool terminated = false;
    zpoller_t *poller = zpoller_new (pipe, zyre_socket (node), NULL);
    while (!terminated) {
        void *which = zpoller_wait (poller, -1);
        if (which == pipe) {
            zmsg_t *msg = zmsg_recv (which);
            if (!msg)
                break;              //  Interrupted

            char *command = zmsg_popstr (msg);
            if (streq (command, "$TERM"))
                terminated = true;
            else
            if (streq (command, "SHOUT")) {
                char *string = zmsg_popstr (msg);
                zyre_shouts (node, "ROPOD", "%s", string);
            }
            else {
                puts ("E: invalid message to actor");
                assert (false);
            }
            free (command);
            zmsg_destroy (&msg);
        }
        else
        if (which == zyre_socket (node)) {
            zmsg_t *msg = zmsg_recv (which);
            char *event = zmsg_popstr (msg);
            char *peer = zmsg_popstr (msg);
            char *name = zmsg_popstr (msg);
            char *group = zmsg_popstr (msg);
            char *message = zmsg_popstr (msg);

            if (streq (event, "ENTER"))
                printf ("%s has joined the chat\n", name);
            else
            if (streq (event, "EXIT"))
                printf ("%s has left the chat\n", name);
            else
            if (streq (event, "SHOUT")) {
                printf ("%s: %s\n", name, message);

            	/* Parse JSON here */
            	Json::Value msg;
            	Json::Reader reader;
            	bool parsingSuccessful = reader.parse(message, msg);     //parse process
            	if (parsingSuccessful) {

            		/* Debug topic */
            		Json::FastWriter fast;
		    		std_msgs::String rosMsg;
		    		rosMsg.data = fast.write(msg);
		    		zyreToRosPuplisher.publish(rosMsg);


            		std::cout  << "[DEBUG]   type = "  << msg["header"]["type"].asString() << std::endl;
            		std::string type = msg["header"]["type"].asString();
            		if(type.compare("CMD") == 0) {
            			std::cout  << "[INFO]    Received a command." << std::endl;

            			Json::Value payload = msg["payload"];
            			if(!payload){
            				std::cout  << "[WARNING] No payload specified." << std::endl;
            			} else {
            				std::cout  << "[DEBUG]   Payload found." << std::endl;
            				Json::Value commandList = payload["commandList"];


            				ropod_ros_msgs::ropod_sem_waypoint_list wayPointList;

            			    for (int i = 0; i < commandList.size(); i++){
            			    	std::cout << " command: " << commandList[i]["command"].asString();
            			    	std::cout << " location: " << commandList[i]["location"].asString();
            			    	std::cout << std::endl;

            			    	ropod_ros_msgs::ropod_sem_waypoint wayPoint;
            			    	wayPoint.command = commandList[i]["command"].asString();
            			    	wayPoint.location = commandList[i]["location"].asString();
            			    	wayPointList.sem_waypoint.push_back(wayPoint);


            			    	if(commandList[i]["command"].asString().compare("GOTO") == 0) {
            			    		std::string location = commandList[i]["location"].asString();
            			    		std::cout  << "[INFO]    Received a GOTO location = " << location << " command." << std::endl;

            			    		//ropod_ros_msgs::ropod_sem_waypoint_list cmdMsg;
            			    		//cmdMsg.

            			    		//zyreToRosCommandsPuplisher.publish(cmdMsg);

            			    	} else if(commandList[i]["command"].asString().compare("ENTER_ELEVATOR") == 0) {
            			    		std::cout  << "[INFO]    Received a ENTER_ELEVATOR  command." << std::endl;

            			    	} else if(commandList[i]["command"].asString().compare("EXIT_ELEVATOR") == 0) {
            			    		std::cout  << "[INFO]    Received a EXIT_ELEVATOR  command." << std::endl;

            			    	} else if(commandList[i]["command"].asString().compare("PAUSE") == 0) {
            			    		std::cout  << "[INFO]    Received a PAUSE  command." << std::endl;

            			    	} else if(commandList[i]["command"].asString().compare("RESUME") == 0) {
            			    		std::cout  << "[INFO]    Received a RESUME  command." << std::endl;

            			    	}

            			    }

            			    zyreToRosCommandsPuplisher.publish(wayPointList);
            			}

            		}

            	} else {
            		std::cout  << "Failed to parse" << reader.getFormattedErrorMessages();
            	}

            } else if (streq (event, "EVASIVE"))  {
                printf ("%s is being evasive\n", name);
        	}

            free (event);
            free (peer);
            free (name);
            free (group);
            free (message);
            zmsg_destroy (&msg);
        }
    }
    zpoller_destroy (&poller);
    zyre_stop (node);
    zclock_sleep (100);
    zyre_destroy (&node);
}

void processTfTopic (zactor_t *actor) {

	ros::Duration maxTFCacheDuration = ros::Duration(10.0); // [s]
	geometry_msgs::TransformStamped transform;
	double roll,yaw,pitch;
	try{
		transform = tfListener.lookupTransform(tfFrameReferenceId, tfFrameId, ros::Time(0));
		tf2::Quaternion q;
		q.setX(transform.transform.rotation.x); // there is certainly a more elegant way than this ...
		q.setY(transform.transform.rotation.y);
		q.setZ(transform.transform.rotation.z);
		q.setW(transform.transform.rotation.w);
		tf2::Matrix3x3 m;
		m.setRotation(q);
		m.getRPY(roll,pitch, yaw);
	}
	catch (tf2::TransformException ex){
		ROS_WARN("%s",ex.what());
		return;
	}

	if ( (ros::Time::now() - transform.header.stamp) > maxTFCacheDuration ) { //simply ignore outdated TF frames
		ROS_WARN("TF found for %s. But it is outdated. Skipping it.", tfFrameId.c_str());
		return;
	}
	ROS_INFO("TF found for %s.", tfFrameId.c_str());


	if (ros::Time::now() - lastSend > ros::Duration(minSendDurationInSec)) { // throttld down pose messages
		ROS_INFO("Sending Zyre message.");

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
	//	      "rencferenceId":"basement_map",
	//	      "x":10,
	//	      "y":20,
	//	      "theta":3.1415
	//	    }
	//	  }
	//	}
		msg["header"]["type"] = "RobotPose2D";
		msg["header"]["metamodel"] = "ropod-msg-schema.json";
		zuuid_t * uuid = zuuid_new();
		const char * uuid_str = zuuid_str_canonical(uuid);
		msg["header"]["msg_id"] = uuid_str;
		zuuid_destroy (&uuid);
		//int64_t now = zclock_time();
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
		zstr_sendx (actor, "SHOUT", poseMsg.str().c_str(), NULL);

		lastSend = ros::Time::now();
	}


}

int main(int argc, char **argv)
{

    if (argc < 2) {
        puts ("syntax: ./ropod_com_mediator <name>");
        exit (0);
    }

	std::string nodeName = "ropod_com_mediator_ropod_1";

	/* Initialize ROS framework */
	ros::init(argc, argv, nodeName);
	ros::NodeHandle node;



	/// Publisher used for the updates
	zyreToRosPuplisher = node.advertise<std_msgs::String>("ropod_zyre_debug", 100);
	zyreToRosCommandsPuplisher = node.advertise<ropod_ros_msgs::ropod_sem_waypoint_list>("ropod_commands", 100);
	std_msgs::String rosMsg;
	zyreToRosPuplisher.publish(rosMsg);

	/* parameters */
	node.param<std::string>("tfFrameId", tfFrameId, "base_link");
	node.param<std::string>("tfFrameReferenceId", tfFrameReferenceId, "map");
	node.param<std::string>("robotName", robotName, "ropod_0");
	node.param<std::string>("zyreGroupName", zyreGroupName, "ROPOD");
	node.param<double>("minSendDurationInSec", minSendDurationInSec, 1.0);

	ROS_INFO("Using parameters: ");
	ROS_INFO("tfFrameId = %s", tfFrameId.c_str());
	ROS_INFO("tfFrameReferenceId = %s", tfFrameReferenceId.c_str());
	ROS_INFO("robotName = %s", robotName.c_str());
	ROS_INFO("zyreGroupName = %s\n", zyreGroupName.c_str());
	ROS_INFO("minSendDurationInSec = %lf\n", minSendDurationInSec);

	/* Initialize Zyre framework */
    zactor_t *actor = zactor_new (chat_actor, argv); //TODO use configurable name
    assert (actor);

    /* Initialize TF */
	tf2_ros::TransformListener* tfUpdateListener = new tf2_ros::TransformListener(tfListener);
	tfListener._addTransformsChangedListener(boost::bind(processTfTopic, actor)); // call on change
	lastSend = ros::Time::now();

	ROS_INFO("Ready.");

    ros::Rate r(10);
    while (ros::ok() && !zsys_interrupted)
    {
	    ros::spinOnce();
        r.sleep();
    }

    /* Clean up */
    ROS_INFO("Cleaning up.");
    delete tfUpdateListener;
    zactor_destroy(&actor);

	return 0;
}
