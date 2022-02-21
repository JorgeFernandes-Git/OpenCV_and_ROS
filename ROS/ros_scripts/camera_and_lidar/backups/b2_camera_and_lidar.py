#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

# publisher
cmd_vel_node = rospy.remap_name("p_jfernandes/cmd_vel")
cmd_vel_pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)

# camera node
camera_node = rospy.remap_name("p_jfernandes/camera/rgb/image_raw")

# lidar node
lidar_node = rospy.remap_name("p_jfernandes/scan")

# color ranges
blue_ranges = {"b": {"min": 116, "max": 148}, "g": {"min": 0, "max": 4}, "r": {"min": 0, "max": 6}}
red_ranges = {"b": {"min": 0, "max": 17}, "g": {"min": 0, "max": 25}, "r": {"min": 128, "max": 142}}
green_ranges = {"b": {"min": 0, "max": 31}, "g": {"min": 243, "max": 255}, "r": {"min": 0, "max": 23}}


class Server:
    def __init__(self, robot_color, show_camera_img=False):
        self.laser_data = None
        self.image_data = None

        self.speed = 0
        self.turn = 0

        self.twist = Twist()
        self.bridge = CvBridge()

        self.robot_color = robot_color
        self.robot_to_catch = False
        self.robot_to_escape = False
        self.robot_teammate = False
        self.show_camera_img = show_camera_img

        self.turning_right = False
        self.turning_left = False

        # from color_segment.py
        if self.robot_color == "Blue":
            self.ranges_to_catch = red_ranges
            self.ranges_to_run = green_ranges
            self.ranges_teammate = blue_ranges
        if self.robot_color == "Green":
            self.ranges_to_catch = blue_ranges
            self.ranges_to_run = red_ranges
            self.ranges_teammate = green_ranges
        if self.robot_color == "Red":
            self.ranges_to_catch = green_ranges
            self.ranges_to_run = blue_ranges
            self.ranges_teammate = red_ranges

        # numpy arrays
        self.lower_to_catch = np.array(
            [self.ranges_to_catch['b']['min'], self.ranges_to_catch['g']['min'], self.ranges_to_catch['r']['min']])
        self.upper_to_catch = np.array(
            [self.ranges_to_catch['b']['max'], self.ranges_to_catch['g']['max'], self.ranges_to_catch['r']['max']])

        self.lower_to_run = np.array(
            [self.ranges_to_run['b']['min'], self.ranges_to_run['g']['min'], self.ranges_to_run['r']['min']])
        self.upper_to_run = np.array(
            [self.ranges_to_run['b']['max'], self.ranges_to_run['g']['max'], self.ranges_to_run['r']['max']])

        self.lower_teammate = np.array(
            [self.ranges_teammate['b']['min'], self.ranges_teammate['g']['min'], self.ranges_teammate['r']['min']])
        self.upper_teammate = np.array(
            [self.ranges_teammate['b']['max'], self.ranges_teammate['g']['max'], self.ranges_teammate['r']['max']])

    def laser_callback(self, msg):
        self.laser_data = msg
        self.overlap()

    def image_callback(self, msg):
        self.image_data = msg

    def lidar_navigation(self, lidar_max_dist_to_obj, lidar_turn_speed):

        regions = {
            # lidar front ranges
            'fright': min(min(self.laser_data.ranges[295:341]), 10),
            'frontr': min(min(self.laser_data.ranges[342:359]), 10),
            'frontl': min(min(self.laser_data.ranges[0:17]), 10),
            'fleft': min(min(self.laser_data.ranges[18:63]), 10),
            # lidar back ranges
            'bright': min(min(self.laser_data.ranges[115:161]), 10),
            'backr': min(min(self.laser_data.ranges[162:179]), 10),
            'backl': min(min(self.laser_data.ranges[180:196]), 10),
            'bleft': min(min(self.laser_data.ranges[197:243]), 10),
        }

        if not self.robot_to_catch and not self.robot_to_escape and not self.robot_teammate:
            print("Robot in LIDAR mode")
            # lidar decisions **************************************
            if regions["frontr"] < lidar_max_dist_to_obj or regions["frontl"] < lidar_max_dist_to_obj:
                self.speed = 0

                # this if is used when the robot as only blocking objects in front of them
                if regions["fright"] <= regions["fleft"]:
                    self.turn = -lidar_turn_speed
                else:
                    self.turn = lidar_turn_speed
                # **************************************

                # make the robot go backwards when it is too close to an object
                if regions["frontr"] < 0.2 or regions["frontl"] < 0.2:
                    self.speed = -0.3
                # ******************************

                # turn decisions ********************
                if regions["fright"] < lidar_max_dist_to_obj:
                    self.turn = -lidar_turn_speed

                if regions["fleft"] < lidar_max_dist_to_obj:
                    self.turn = lidar_turn_speed
                # **********************
            else:
                self.speed = 0.7
                self.turn = 0
                open_space_dist = 3

                # some code that make the robot turn if distance to object is too high
                # this way the robot will find doors more easily
                # if regions["fright"] > 3:
                #     self.turn = lidar_turn_speed
                #     print("open area detected")

                if regions["fright"] > open_space_dist:  # and not self.turning_left:
                    self.turn = lidar_turn_speed
                    # self.turning_right = True
                    # self.turning_left = False
                    print("open area detected on the right")

                if regions["fleft"] > open_space_dist:  # and not self.turning_right:
                    self.turn = -lidar_turn_speed
                    # self.turning_right = False
                    # self.turning_left = True
                    print("open area detected on the left")

        if self.robot_to_escape:
            # lidar decisions **************************************
            if regions["backr"] < lidar_max_dist_to_obj or regions["backl"] < lidar_max_dist_to_obj:
                self.speed = -1
                self.turn = lidar_turn_speed

                if regions["backr"] < 0.2 or regions["backl"] < 0.2:
                    self.speed = 1

                # print("back")

                if regions["bright"] < lidar_max_dist_to_obj:
                    self.turn = -lidar_turn_speed

                if regions["bleft"] < lidar_max_dist_to_obj:
                    self.turn = lidar_turn_speed

            else:
                self.speed = -1.0
                self.turn = 0

        return self.speed, self.turn

    def camera_navigation(self):
        # ***************** camera code *************
        img = self.bridge.imgmsg_to_cv2(self.image_data, desired_encoding='bgr8')
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # masks
        mask_to_catch = cv2.inRange(img, self.lower_to_catch, self.upper_to_catch)
        mask_to_run = cv2.inRange(img, self.lower_to_run, self.upper_to_run)
        mask_teammate = cv2.inRange(img, self.lower_teammate, self.upper_teammate)

        # img dimensions
        height, width, dimension = img.shape

        # moments - center of the blob
        M_to_catch = cv2.moments(mask_to_catch)
        M_to_run = cv2.moments(mask_to_run)
        M_teammate = cv2.moments(mask_teammate)

        # ****************** to catch *************************
        if M_to_catch['m00'] > 0 and not self.robot_to_escape:
            print("Robot in CATCH mode")

            self.robot_to_catch = True

            # edges = cv2.Canny(mask_to_catch, 100, 200)
            edges = cv2.GaussianBlur(mask_to_catch, (7, 7), 2)
            kernel = np.ones((7, 7), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
            edges = cv2.medianBlur(edges, 9)
            edges = cv2.dilate(edges, kernel, iterations=4)
            kernel = np.ones((9, 9), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # for contours in contours:
            #     (x, y, w, h) = cv2.boundingRect(contours)
            #     # draw the rectangle over detected object
            #
            #     if cv2.contourArea(contours) < 3000:
            #         continue
            #
            #     cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            #     cv2.putText(edges, '+', (int(x+w/2), int(y+h/2)), cv2.FONT_ITALIC, 1, (255, 0, 0), 2, cv2.LINE_8)
            #     # print(cv2.contourArea(contours))

            if len(contours) != 0:
                # draw in blue the contours that were founded
                # cv2.drawContours(output, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)

                # draw the biggest contour (c) in green
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.putText(edges, '+', (int(x+w/2), int(y+h/2)), cv2.FONT_ITALIC, 2, (255, 0, 0), 2, cv2.LINE_8)

                cx_to_catch = int(x+w/2)
                cy_to_catch = int(y+h/2)
            else:
                cx_to_catch = int(M_to_catch['m10'] / M_to_catch['m00'])
                cy_to_catch = int(M_to_catch['m01'] / M_to_catch['m00'])
                # cv2.circle(img, (cx_to_catch, cy_to_catch), 20, (0, 0, 255), -1)
                cv2.putText(edges, '+', (cx_to_catch, cy_to_catch), cv2.FONT_ITALIC, 2, (255, 0, 0), 2, cv2.LINE_8)

            # find center
            threshold = cx_to_catch - width / 2

            self.turn = threshold

            if abs(self.turn) < 150:  # accelerate
                if self.speed < 1.0:
                    self.speed += 0.1
            else:
                if self.speed > 0.3:  # decelerate
                    self.speed -= 0.05
                else:
                    self.speed = 0.3

        elif not self.robot_to_escape:  # not detected target
            self.robot_to_catch = False

        # ****************** to escape *************************
        if M_to_run['m00'] > 0:
            print("Robot in ESCAPE mode")

            cx_to_run = int(M_to_run['m10'] / M_to_run['m00'])
            cy_to_run = int(M_to_run['m01'] / M_to_run['m00'])
            cv2.circle(img, (cx_to_run, cy_to_run), 20, (255, 0, 0), -1)

            # escape from the robot
            self.robot_to_escape = True
            self.speed, self.turn = self.lidar_navigation(lidar_max_dist_to_obj=1.5, lidar_turn_speed=350)

        else:
            self.robot_to_escape = False
            
        # ********************** team mate found *******************
        if M_teammate['m00'] > 0 and not self.robot_to_escape and not self.robot_to_catch:
            print("Team mate found")

            cx_teammate = int(M_teammate['m10'] / M_teammate['m00'])
            cy_teammate = int(M_teammate['m01'] / M_teammate['m00'])
            cv2.circle(img, (cx_teammate, cy_teammate), 20, (255, 0, 0), -1)
            
            # keep teammates apart from each other
            self.robot_teammate = True
            self.turn = 550
            self.speed = 0
        else:
            self.robot_teammate = False
            
        if self.show_camera_img:
            # resize
            img = cv2.resize(img, (300, 300))
            mask_to_catch = cv2.resize(mask_to_catch, (300, 300))
            mask_to_run = cv2.resize(mask_to_run, (300, 300))
            mask_teammate = cv2.resize(mask_teammate, (300, 300))

            # show images
            cv2.imshow("mask_to_catch", mask_to_catch)
            # cv2.imshow("edges", edges)

            cv2.imshow("mask_to_run", mask_to_run)
            cv2.imshow("mask_teammate", mask_teammate)
            cv2.imshow("image", img)

        k = cv2.waitKey(1) & 0xFF

        return self.speed, self.turn

    def publisher(self, speed, turn):
        self.twist.linear.x = speed
        self.twist.angular.z = -float(turn) / 500
        cmd_vel_pub.publish(self.twist)

    def overlap(self):
        if self.image_data is not None and self.laser_data is not None:
            try:

                # ***************** camera code *************
                self.speed, self.turn = self.camera_navigation()

                # # ***************** lidar code **************
                if not self.robot_to_catch and not self.robot_to_escape:
                    self.speed, self.turn = self.lidar_navigation(lidar_max_dist_to_obj=0.9, lidar_turn_speed=250)

                print(self.speed, self.turn)
                self.publisher(speed=self.speed, turn=self.turn)

            except CvBridgeError as e:
                print(e)


def main():
    rospy.init_node('camera_and_lidar')  # init node
    robot_color = rospy.get_param("~robot_color", default="Blue")  # robot color param
    server = Server(robot_color)  # server object

    # topics to subscriber
    rospy.Subscriber(lidar_node, LaserScan, server.laser_callback)
    rospy.Subscriber(camera_node, Image, server.image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
