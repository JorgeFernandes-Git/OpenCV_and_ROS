#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

bridge = CvBridge()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()


def img_callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # change below lines to map the color you wanted robot to follow
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])

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
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            cmd_vel_pub.publish(twist)

        cv2.imshow("mask", mask)
        cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('line_follower')
    img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
