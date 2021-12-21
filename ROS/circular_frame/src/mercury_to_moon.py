#!/usr/bin/env python3

import rospy
from math import *
import tf2_ros


def main():
    rospy.init_node('mercury_to_moon')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform("mercury", 'moon', rospy.Time())
            dist_mercury_moon = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            rospy.loginfo(f'Distance from mercury to moon is {dist_mercury_moon} at {rospy.Time.now()}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Could find transformation")
            rate.sleep()
            continue

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
