#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial

cmd_vel_node = rospy.remap_name("/cmd_vel")
cam_left_node = rospy.remap_name("/camera/left/image_raw")

bridge = CvBridge()
cmd_vel_pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)
twist = Twist()
speed = 0.5
divider = 100
max_speed = 0.5
min_speed = 0.01
inc_speed = 0.01
left_limit = 100
right_limit = -100


def publisher(speed, err):
    twist.linear.x = speed
    twist.angular.z = -float(err) / divider
    cmd_vel_pub.publish(twist)


def img_callback_left(data, lower_yellow, upper_yellow):
    try:
        global speed
        global divider
        global max_speed
        global min_speed
        global inc_speed
        global left_limit
        global right_limit

        img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)

        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0

        M_yellow = cv2.moments(mask_yellow)

        if M_yellow['m00'] > 0:
            cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
            cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])
            cv2.circle(img, (cx_yellow, cy_yellow), 20, (0, 0, 255), -1)

            err = cx_yellow-(w/4)
            # print(err)

            if err > 100:
                # deceleration
                if speed > 0.01:
                    speed -= 0.01
                elif speed <= 0.01:
                    speed = speed
            else:
                # acceleration
                if speed < 0.5:
                    speed += 0.01
                elif speed >= 0.5:
                    speed = speed

            publisher(speed, err)

        cv2.imshow("mask_yellow", mask_yellow)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("image_left", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('road_follow_left')

        # from color_segment.py
        ranges_yellow = {"b": {"min": 0, "max": 43}, "g": {"min": 182, "max": 255}, "r": {"min": 109, "max": 255}}

        # numpy arrays
        lower_yellow = np.array([ranges_yellow['b']['min'], ranges_yellow['g']['min'], ranges_yellow['r']['min']])
        upper_yellow = np.array([ranges_yellow['b']['max'], ranges_yellow['g']['max'], ranges_yellow['r']['max']])

        img_sub_left = rospy.Subscriber(cam_left_node, Image, partial(img_callback_left,
                                                                      lower_yellow=lower_yellow,
                                                                      upper_yellow=upper_yellow))

        rospy.spin()


if __name__ == "__main__":
    main()
