
//  --------------------------------------------------------------------------
//  Example Zyre distributed chat application
//
//  Copyright (c) 2010-2014 The Authors
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//  --------------------------------------------------------------------------


#include "zyre.h"
#include <json/json.h>
#include <iostream>


//  This actor will listen and publish anything received
//  on the CHAT group

static void
chat_actor (zsock_t *pipe, void *args)
{
    zyre_t *node = zyre_new ((char *) args);
    if (!node)
        return;                 //  Could not create new node

    zyre_start (node);
    zyre_join (node, "CHAT");
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
                zyre_shouts (node, "CHAT", "%s", string);
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
            		std::cout  << "[DEBUG]   type = "  << msg["header"]["type"].asString() << std::endl;
//            		if(!msg.isMember("header")) {
//            			std::cout  << "[WARNING] No header specified." << std::endl;
//            		} else {
//            			std::cout  << "[DEBUG]   Header found." << std::endl;
//            		}
            		std::string type = msg["header"]["type"].asString();
            		if(type.compare("CMD") == 0) {
            			std::cout  << "[INFO]    Received a command." << std::endl;

            			Json::Value payload = msg["payload"];
            			if(!payload){
            				std::cout  << "[WARNING] No payload specified." << std::endl;
            			} else {
            				std::cout  << "[DEBUG]   Payload found." << std::endl;
            				Json::Value commandList = payload["commandList"];

            			    for (int i = 0; i < commandList.size(); i++){
            			    	std::cout << " command: " << commandList[i]["command"].asString();
            			    	std::cout << " location: " << commandList[i]["location"].asString();
            			    	std::cout << std::endl;

            			    	if(commandList[i]["command"].asString().compare("GOTO") == 0) {
            			    		std::string location = commandList[i]["location"].asString();
            			    		std::cout  << "[INFO]    Received a GOTO location = " << location << " command." << std::endl;

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

int
main (int argc, char *argv [])
{
    if (argc < 2) {
        puts ("syntax: ./chat myname");
        exit (0);
    }
    zactor_t *actor = zactor_new (chat_actor, argv [1]);
    assert (actor);

    while (!zsys_interrupted) {
        char message [1024];
        if (!fgets (message, 1024, stdin))
            break;
        message [strlen (message) - 1] = 0;     // Drop the trailing linefeed
        zstr_sendx (actor, "SHOUT", message, NULL);
    }
    zactor_destroy (&actor);
    return 0;
}
