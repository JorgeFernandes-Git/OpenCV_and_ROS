#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from functools import partial

cmd_vel_node = rospy.remap_name("/p_jfernandes/cmd_vel")
camera_node = rospy.remap_name("p_jfernandes/camera/rgb/image_raw")

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


def img_callback(data, lower_white, upper_white):
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

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 50)

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

            if err > left_limit or err < right_limit:
                # deceleration
                if speed > min_speed:
                    speed -= inc_speed
                    # if divider > 60:
                    #     divider -= 1
                    # elif divider <= 60:
                    #     divider = divider
                elif speed <= min_speed:
                    speed = speed
            else:
                # acceleration
                if speed < max_speed:
                    speed += inc_speed
                    # if divider < 100:
                    #     divider += 1
                    # elif divider <= 100:
                    #     divider = divider
                elif speed >= max_speed:
                    speed = speed

            if err > 300 or err < -300:
                speed = 0

        else:
            # stop if lose line
            speed = 0
            err = -0

        # pub the speed in x and z on topic cmd_vel with Twist msg
        twist.linear.x = speed
        twist.angular.z = -float(err) / divider
        cmd_vel_pub.publish(twist)
        print(speed, divider, err)

        cv2.imshow("mask", mask)
        # cv2.imshow("hsv", hsv)
        cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    while not rospy.is_shutdown():
        rospy.init_node('line_follower')

        # # from color_segment.py
        ranges_pcss = {"b": {"min": 0, "max": 0}, "g": {"min": 0, "max": 0}, "r": {"min": 123, "max": 255}}

        # numpy arrays
        lower_white = np.array([ranges_pcss['b']['min'], ranges_pcss['g']['min'], ranges_pcss['r']['min']])
        upper_white = np.array([ranges_pcss['b']['max'], ranges_pcss['g']['max'], ranges_pcss['r']['max']])

        # white line from course.world
        # lower_white = np.array([10, 10, 10])
        # upper_white = np.array([255, 255, 250])

        img_sub = rospy.Subscriber(camera_node, Image, partial(img_callback,
                                                               lower_white=lower_white,
                                                               upper_white=upper_white))

        rospy.spin()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
