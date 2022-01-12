#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial

bridge = CvBridge()
twist = Twist()
cmd_vel_pub = rospy.Publisher('/p_jfernandes/cmd_vel', Twist, queue_size=10)


def img_callback(data, lower_left, upper_left, lower_right, upper_right):
    try:
        speed = 0.2
        divider = 100

        img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_left = cv2.inRange(hsv, lower_left, upper_left)
        mask_right = cv2.inRange(hsv, lower_right, upper_right)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)

        mask_left[0:search_top, 0:w] = 0
        mask_left[search_bot:h, 0:w] = 0

        mask_right[0:search_top, 0:w] = 0
        mask_right[search_bot:h, 0:w] = 0

        M_left = cv2.moments(mask_left)
        M_right = cv2.moments(mask_right)

        if M_left['m00'] > 0 and M_right['m00'] > 0:
            cx_left = int(M_left['m10'] / M_left['m00'])
            cy_left = int(M_left['m01'] / M_left['m00'])
            cv2.circle(img, (cx_left, cy_left), 20, (0, 0, 255), -1)

            cx_right = int(M_right['m10'] / M_right['m00'])
            cy_right = int(M_right['m01'] / M_right['m00'])
            cv2.circle(img, (cx_right, cy_right), 20, (0, 255, 0), -1)

            cx_car = int((cx_right - cx_left) / 2) + cx_left
            cy_car = cy_right

            cv2.circle(img, (cx_car, cy_car), 20, (255, 0, 0), -1)

            err = cx_car - w / 2
            # print(err)

            if err > 20 or err < -20:
                # deceleration
                if speed > 0.01:
                    speed -= 0.01
                    # if divider > 60:
                    #     divider -= 1
                    # elif divider <= 60:
                    #     divider = divider
                elif speed <= 0.01:
                    speed = speed
            else:
                # acceleration
                if speed < 0.5:
                    speed += 0.01
                    # if divider < 100:
                    #     divider += 1
                    # elif divider <= 100:
                    #     divider = divider
                elif speed >= 0.5:
                    speed = speed
        else:
            speed = 0.01
            err = 0

        # pub the speed in x and z on topic cmd_vel with Twist msg
        twist.linear.x = speed
        twist.angular.z = -float(err) / divider
        cmd_vel_pub.publish(twist)
        print(speed, divider, err)

        cv2.imshow("mask_left", mask_left)
        cv2.imshow("mask_right", mask_right)
        cv2.imshow("hsv", hsv)
        cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('road_follower')

        # from color_segment.py
        ranges_left = {"b": {"min": 0, "max": 0}, "g": {"min": 0, "max": 0}, "r": {"min": 123, "max": 255}}
        ranges_right = {"b": {"min": 0, "max": 0}, "g": {"min": 0, "max": 0}, "r": {"min": 123, "max": 255}}

        # numpy arrays
        lower_left = np.array([ranges_left['b']['min'], ranges_left['g']['min'], ranges_left['r']['min']])
        upper_left = np.array([ranges_left['b']['max'], ranges_left['g']['max'], ranges_left['r']['max']])

        lower_right = np.array([ranges_right['b']['min'], ranges_right['g']['min'], ranges_right['r']['min']])
        upper_right = np.array([ranges_right['b']['max'], ranges_right['g']['max'], ranges_right['r']['max']])

        img_sub = rospy.Subscriber('p_jfernandes/camera/rgb/image_raw', Image, partial(img_callback,
                                                                                       lower_left=lower_left,
                                                                                       upper_left=upper_left,
                                                                                       lower_right=lower_right,
                                                                                       upper_right=upper_right))

        rospy.spin()


if __name__ == "__main__":
    main()
