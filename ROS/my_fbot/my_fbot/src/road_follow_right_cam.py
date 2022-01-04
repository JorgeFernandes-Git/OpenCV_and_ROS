#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial

cmd_vel_node = rospy.remap_name("/cmd_vel")
cam_right_node = rospy.remap_name("/camera/right/image_raw")

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


def img_callback_right(data, lower_white, upper_white):
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
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)

        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0

        M_white = cv2.moments(mask_white)

        if M_white['m00'] > 0:
            cx_white = int(M_white['m10'] / M_white['m00'])
            cy_white = int(M_white['m01'] / M_white['m00'])
            cv2.circle(img, (cx_white, cy_white), 20, (0, 255, 0), -1)

            err = cx_white - (3 * w / 4)
            # print(err)

            if err < -100:
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

        cv2.imshow("mask_white", mask_white)
        # # cv2.imshow("hsv", hsv)
        cv2.imshow("image_right", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('road_follow_right')

        # from color_segment.py
        ranges_white = {"b": {"min": 0, "max": 0}, "g": {"min": 0, "max": 0}, "r": {"min": 123, "max": 255}}

        # numpy arrays

        lower_white = np.array([ranges_white['b']['min'], ranges_white['g']['min'], ranges_white['r']['min']])
        upper_white = np.array([ranges_white['b']['max'], ranges_white['g']['max'], ranges_white['r']['max']])

        img_sub_right = rospy.Subscriber(cam_right_node, Image, partial(img_callback_right,
                                                                        lower_white=lower_white,
                                                                        upper_white=upper_white))

        rospy.spin()


if __name__ == "__main__":
    main()
