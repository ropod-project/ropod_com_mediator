#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('robot_pose_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose_pub = rospy.Publisher('~pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    loop_rate = rospy.get_param('~loop_rate', 1.0)
    rate = rospy.Rate(loop_rate)

    reference_frame = rospy.get_param('~reference_frame', 'map')
    target_frame = rospy.get_param('~target_frame', 'ropod/base_link')

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(reference_frame, target_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.PoseStamped()
        msg.header = trans.header
        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.orientation.x = trans.transform.rotation.x
        msg.pose.orientation.y = trans.transform.rotation.y
        msg.pose.orientation.z = trans.transform.rotation.z
        msg.pose.orientation.w = trans.transform.rotation.w

        pose_pub.publish(msg)

        rate.sleep()
