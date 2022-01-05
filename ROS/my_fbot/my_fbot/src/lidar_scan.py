#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist

cmd_vel_node = rospy.remap_name("/cmd_vel")
cmd_vel_pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)
twist = Twist()

"""NOTES:

lidar range: 0 to 180 degrees
720 values in total
4 values per degree
msg.ranges[360] position in front of the robot
"""


def publisher(speed, turn):
    twist.linear.x = speed
    twist.angular.z = -float(turn) / 100
    cmd_vel_pub.publish(twist)


def callback_msg_received(msg):

    turn = 0
    err_inc = 0.1
    err_min = 0
    err_max = 90

    speed = 0.5
    spd_inc = 0.001
    spd_min = 0
    spd_max = 0.5

    obj_dist_min = 0.5
    angle_min = 160
    angle_max = 360

    for range in msg.ranges:
        if range < obj_dist_min:
            speed = 0
            turn = 90

    # only cover a range
    # for idx in range(angle_min, angle_max):
    #     if msg.ranges[idx] < obj_dist_min:
    #         speed = spd_min
    #         turn = err_max

    print(speed, turn)
    publisher(speed=speed, turn=turn)


def main():
    # subscriber
    rospy.init_node("lidar_sub_node")
    rospy.Subscriber("/scan", LaserScan, callback_msg_received)
    rospy.spin()


if __name__ == '__main__':
    main()
