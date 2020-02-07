#!/usr/bin/python

import rospy
import std_msgs.msg
import sensor_msgs.msg
import subprocess as sp
import os

class RobotCommandProcessor:
    def __init__(self):
        self.rviz_process = None
        self.command_sub = rospy.Subscriber('~command', std_msgs.msg.String, self.command_cb)
        self.joypad_pub = rospy.Publisher('~joypad', sensor_msgs.msg.Joy, queue_size=1)

    def command_cb(self, msg):
        if msg.data == "REINIT-POSE":
            sp.Popen(['roslaunch', 'route_navigation', 'initial_pose_publisher.launch'],
                     stdout=sp.PIPE, stderr=sp.PIPE)
        elif msg.data == "START-RVIZ":
            self.rviz_process = sp.Popen('rviz',
                                    env=dict(os.environ, DISPLAY=":0.0", XAUTHORITY="~/.Xauthority"),
                                    stdout=sp.PIPE,
                                    stderr=sp.PIPE)
        elif msg.data == "STOP-RVIZ":
            if self.rviz_process is not None:
                self.rviz_process.kill()
        elif msg.data == 'START-CONTROLLER':
            joy_msg = sensor_msgs.msg.Joy()
            joy_msg.axes = [0.0, -0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
            joy_msg.buttons = [1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]
            joy_msg.header.stamp = rospy.Time.now()
            self.joypad_pub.publish(joy_msg)
        elif msg.data == 'STOP-CONTROLLER':
            joy_msg = sensor_msgs.msg.Joy()
            joy_msg.axes = [0.0, -0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
            joy_msg.buttons = [0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0]
            joy_msg.header.stamp = rospy.Time.now()
            self.joypad_pub.publish(joy_msg)

def main():
    rospy.init_node("robot_command_processor")
    rcp = RobotCommandProcessor()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
