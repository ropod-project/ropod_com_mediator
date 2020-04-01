#!/usr/bin/env python3
import os
import sys
import rospy

from fmlib.monitoring.version_shouter import VersionShouter


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    workspace = myargv[1]
    config_filename = myargv[2]

    robot_id = os.environ.get('ROPOD_ID', 'ropod_001')
    print(robot_id)
    shouter = VersionShouter(workspace, config_filename, robot_id=robot_id)
    shouter.run()
