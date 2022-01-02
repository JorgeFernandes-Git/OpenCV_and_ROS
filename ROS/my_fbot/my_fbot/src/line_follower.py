#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial

bridge = CvBridge()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()
speed = 0.5
divider = 100


def img_callback(data, lower_yellow, upper_yellow):
    try:
        global speed
        global divider

        img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # transform the image and show it
        # mask = cv2.inRange(img, mins_pcss, maxs_pcss)  # colors mask
        # img = cv2.bitwise_and(img, img, mask=mask)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)

            # CONTROL starts
            err = cx - w / 2
            # print(err)

            if err > 100 or err < -100:
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

            if err > 300 or err < -300:
                speed = 0

        else:
            # stop if lose line and rotate to left
            speed = 0
            err = -300

        # pub the speed in x and z on topic cmd_vel with Twist msg
        twist.linear.x = speed
        twist.angular.z = -float(err) / divider
        cmd_vel_pub.publish(twist)
        print(speed, divider, err)

        cv2.imshow("mask", mask)
        cv2.imshow("hsv", hsv)
        cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('line_follower')

        # from color_segment.py
        ranges_pcss = {"b": {"min": 0, "max": 39}, "g": {"min": 222, "max": 255}, "r": {"min": 215, "max": 255}}

        # numpy arrays
        lower_yellow = np.array([ranges_pcss['b']['min'], ranges_pcss['g']['min'], ranges_pcss['r']['min']])
        upper_yellow = np.array([ranges_pcss['b']['max'], ranges_pcss['g']['max'], ranges_pcss['r']['max']])

        # change below lines to map the color you wanted robot to follow
        # lower_yellow = np.array([10, 10, 10])
        # upper_yellow = np.array([255, 255, 250])

        img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, partial(img_callback,
                                                                           lower_yellow=lower_yellow,
                                                                           upper_yellow=upper_yellow))

        rospy.spin()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
