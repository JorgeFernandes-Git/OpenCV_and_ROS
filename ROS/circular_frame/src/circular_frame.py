#!/usr/bin/env python3

import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from math import *


def main():

    rospy.init_node('circular_frame')
    rate = rospy.Rate(100)

    br = tf2_ros.TransformBroadcaster()

    rho = 3
    alpha = 0

    while not rospy.is_shutdown():

        alpha += 0.01

        if alpha > 2*pi:
            alpha = 0

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "parent"
        t.child_frame_id = "child"
        t.transform.translation.x = rho * cos(alpha)
        t.transform.translation.y = rho * sin(alpha)
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

        rate.sleep()


if __name__ == '__main__':
    main()