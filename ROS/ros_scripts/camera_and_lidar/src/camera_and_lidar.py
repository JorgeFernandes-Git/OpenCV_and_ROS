#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, Image

# publisher
cmd_vel_node = rospy.remap_name("p_jfernandes/cmd_vel")
cmd_vel_pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)

# camera node
camera_node = rospy.remap_name("p_jfernandes/camera/rgb/image_raw")

# lidar node
lidar_node = rospy.remap_name("p_jfernandes/scan")

# objects
bridge = CvBridge()
twist = Twist()

# variables
robot_color = ""
speed = 0
turn = 550
robot_to_escape = False

# color ranges
blue_ranges = {"b": {"min": 185, "max": 255}, "g": {"min": 0, "max": 255}, "r": {"min": 0, "max": 255}}
red_rages = {"b": {"min": 0, "max": 255}, "g": {"min": 0, "max": 255}, "r": {"min": 184, "max": 255}}
green_rages = {"b": {"min": 0, "max": 255}, "g": {"min": 185, "max": 255}, "r": {"min": 0, "max": 255}}


def publisher(speed, turn):
    twist.linear.x = speed
    twist.angular.z = -float(turn) / 500
    cmd_vel_pub.publish(twist)


class Server:
    def __init__(self):
        self.laser_data = None
        self.image_data = None

    def laser_callback(self, msg):
        self.laser_data = msg
        self.overlap()

    def image_callback(self, msg):
        self.image_data = msg

    def overlap(self):
        if self.image_data is not None and self.laser_data is not None:
            try:

                global speed
                global turn
                global robot_color
                global robot_to_escape

                # # ***************** lidar code **************
                # obj_near = False
                # # print(len(self.laser_data.ranges))
                #
                # for range in self.laser_data.ranges:
                #     if range < 0.2:
                #         speed = 0
                #         turn = 350
                #         obj_near = True
                #         publisher(speed=speed, turn=turn)
                #
                #     else:
                #         obj_near = False
                #         turn = 350

                # ***************** camera code *************
                img = bridge.imgmsg_to_cv2(self.image_data, desired_encoding='bgr8')
                # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                # from color_segment.py red and green
                if robot_color == "Blue":
                    ranges_to_catch = red_rages
                    ranges_to_run = green_rages
                if robot_color == "Green":
                    ranges_to_catch = blue_ranges
                    ranges_to_run = red_rages
                if robot_color == "Red":
                    ranges_to_catch = green_rages
                    ranges_to_run = blue_ranges

                # numpy arrays
                lower_to_catch = np.array(
                    [ranges_to_catch['b']['min'], ranges_to_catch['g']['min'], ranges_to_catch['r']['min']])
                upper_to_catch = np.array(
                    [ranges_to_catch['b']['max'], ranges_to_catch['g']['max'], ranges_to_catch['r']['max']])

                lower_to_run = np.array(
                    [ranges_to_run['b']['min'], ranges_to_run['g']['min'], ranges_to_run['r']['min']])
                upper_to_run = np.array(
                    [ranges_to_run['b']['max'], ranges_to_run['g']['max'], ranges_to_run['r']['max']])

                # masks
                mask_to_catch = cv2.inRange(img, lower_to_catch, upper_to_catch)
                mask_to_run = cv2.inRange(img, lower_to_run, upper_to_run)

                # img dimensions
                height, width, dimension = img.shape

                M_to_catch = cv2.moments(mask_to_catch)
                M_to_run = cv2.moments(mask_to_run)

                # if not obj_near:
                # ****************** to catch *************************
                if M_to_catch['m00'] > 0 and not robot_to_escape:
                    cx_to_catch = int(M_to_catch['m10'] / M_to_catch['m00'])
                    cy_to_catch = int(M_to_catch['m01'] / M_to_catch['m00'])
                    cv2.circle(img, (cx_to_catch, cy_to_catch), 20, (0, 0, 255), -1)

                    # find center
                    threshold = cx_to_catch - width / 2

                    turn = threshold
                    # speed = 0.3

                    if abs(turn) < 150:  # accelerate
                        if speed < 1.0:
                            speed += 0.1
                    else:
                        if speed > 0.3:  # decelerate
                            speed -= 0.05
                        else:
                            speed = 0.3

                elif not robot_to_escape:  # not detected target, keep rotating
                    speed = 0
                    turn = 650

                # ****************** to run *************************
                if M_to_run['m00'] > 0:
                    cx_to_run = int(M_to_run['m10'] / M_to_run['m00'])
                    cy_to_run = int(M_to_run['m01'] / M_to_run['m00'])
                    cv2.circle(img, (cx_to_run, cy_to_run), 20, (255, 0, 0), -1)

                    # escape from the robot
                    robot_to_escape = True
                    speed = 1.0
                    turn = 650

                else:
                    robot_to_escape = False

                # resize
                # img = cv2.resize(img, (300, 300))
                # mask_to_catch = cv2.resize(mask_to_catch, (300, 300))
                # mask_to_run = cv2.resize(mask_to_run, (300, 300))

                # show images
                # cv2.imshow("mask_to_catch", mask_to_catch)
                # cv2.imshow("mask_to_run", mask_to_run)
                # cv2.imshow("image", img)

                print(speed, turn)
                publisher(speed=speed, turn=turn)

                # print(obj_near)
                k = cv2.waitKey(1) & 0xFF

            except CvBridgeError as e:
                print(e)


def main():
    global robot_color
    rospy.init_node('camera_and_lidar')  # init node

    robot_color = rospy.get_param("~robot_color", default="Blue")  # robot color param

    server = Server()  # server object

    # topics to subscriber
    rospy.Subscriber(lidar_node, LaserScan, server.laser_callback)
    rospy.Subscriber(camera_node, Image, server.image_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
