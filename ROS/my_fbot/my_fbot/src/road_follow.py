#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial


bridge = CvBridge()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
twist = Twist()


def img_callback(data, lower_yellow, upper_yellow, lower_white, upper_white):
    try:
        speed = 0.2
        divider = 100

        img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)

        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0

        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0

        M_yellow = cv2.moments(mask_yellow)
        M_white = cv2.moments(mask_white)

        if M_yellow['m00'] > 0:
            cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
            cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])
            cv2.circle(img, (cx_yellow, cy_yellow), 20, (0, 0, 255), -1)

        if M_white['m00'] > 0:
            cx_white = int(M_white['m10'] / M_white['m00'])
            cy_white = int(M_white['m01'] / M_white['m00'])
            cv2.circle(img, (cx_white, cy_white), 20, (0, 255, 0), -1)

        cx_car = int((cx_white - cx_yellow)/2) + cx_yellow
        cy_car = cy_white

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

        # pub the speed in x and z on topic cmd_vel with Twist msg
        twist.linear.x = speed
        twist.angular.z = -float(err) / divider
        cmd_vel_pub.publish(twist)
        print(speed, divider, err)

        cv2.imshow("mask_yellow", mask_yellow)
        cv2.imshow("mask_white", mask_white)
        cv2.imshow("hsv", hsv)
        cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('line_follower')

        # from color_segment.py
        ranges_yellow = {"b": {"min": 0, "max": 43}, "g": {"min": 182, "max": 255}, "r": {"min": 109, "max": 255}}
        ranges_white = {"b": {"min": 0, "max": 0}, "g": {"min": 0, "max": 0}, "r": {"min": 123, "max": 255}}   

        # numpy arrays
        lower_yellow = np.array([ranges_yellow['b']['min'], ranges_yellow['g']['min'], ranges_yellow['r']['min']])
        upper_yellow = np.array([ranges_yellow['b']['max'], ranges_yellow['g']['max'], ranges_yellow['r']['max']])

        lower_white = np.array([ranges_white['b']['min'], ranges_white['g']['min'], ranges_white['r']['min']])
        upper_white = np.array([ranges_white['b']['max'], ranges_white['g']['max'], ranges_white['r']['max']])

        img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, partial(img_callback,
                                                                           lower_yellow=lower_yellow,
                                                                           upper_yellow=upper_yellow, 
                                                                           lower_white=lower_white,
                                                                           upper_white=upper_white))

        rospy.spin()


if __name__ == "__main__":
    main()
