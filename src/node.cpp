#include <iostream>
#include <fstream>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* Zyre + JONS includes */
#include "zyre.h"
#include <json/json.h>
#include <iostream>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/ropod_sem_waypoint_list.h>

ros::Publisher zyreToRosPuplisher;
ros::Publisher zyreToRosCommandsPuplisher;

static void
chat_actor (zsock_t *pipe, void *args)
{
    zyre_t *node = zyre_new ((char *) args);
    if (!node)
        return;                 //  Could not create new node

    zyre_start (node);
    zyre_join (node, "ROPOD");
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
//	ros::Publisher zyreToRosPuplisher;
	zyreToRosPuplisher = node.advertise<std_msgs::String>("ropod_zyre_debug", 100);
	zyreToRosCommandsPuplisher = node.advertise<ropod_ros_msgs::ropod_sem_waypoint_list>("ropod_commands", 100);
	std_msgs::String rosMsg;
	zyreToRosPuplisher.publish(rosMsg);

	/* parameters */


	/* Initialize Zyre framework */
    zactor_t *actor = zactor_new (chat_actor, argv); //TODO use configurable name
    assert (actor);

	ROS_INFO("Ready.");
	ros::spin();

	return 0;
}
