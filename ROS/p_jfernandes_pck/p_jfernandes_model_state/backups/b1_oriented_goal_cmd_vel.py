#! /usr/bin/env python3

import math
from gazebo_msgs.srv import GetModelState
import rospy
import cv2
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from functools import partial

# variables
x_arena_min = -8                    # arena x min
x_arena_max = 8                     # arena x max
y_arena_min = 2                     # arena y min
y_arena_max = -2                    # arena y max

max_speed = 1.0                     # robot max speed
speed = 0                           # robot speed
angle = 0                           # robot angle to turn

# flags
obj_detected = False                # flag for lidar detection
robot_to_escape_near = False        # flag robot to escape detected
robot_to_catch_near = False         # flag robot to catch near
# robot_to_catch_on_sight = False

# list to store positions
robot_to_catch_poses_x = []
robot_to_catch_poses_y = []
robot_to_catch_ori_z = []
robot_to_escape_poses_x = []
robot_to_escape_poses_y = []
robot_to_escape_ori_z = []

# list to store distances
distances_to_catch = []
distances_to_escape = []

# rate_hz = rospy.get_param("~rate")
rate_hz = 20

# objects
twist = Twist()
# bridge = CvBridge()

# nodes
get_model_state = rospy.remap_name("/gazebo/get_model_state")
cmd_vel_node = rospy.remap_name("/blue1/cmd_vel")
lidar_node = rospy.remap_name("/blue1/scan")
# camera_node = rospy.remap_name("/blue1/camera/rgb/image_raw")

# publisher
pub_cmd_vel = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)

# # color ranges
# blue_ranges = {"b": {"min": 116, "max": 148}, "g": {"min": 0, "max": 4}, "r": {"min": 0, "max": 6}}
# red_ranges = {"b": {"min": 0, "max": 17}, "g": {"min": 0, "max": 25}, "r": {"min": 128, "max": 142}}
# green_ranges = {"b": {"min": 0, "max": 31}, "g": {"min": 243, "max": 255}, "r": {"min": 0, "max": 23}}


# def image_callback(image_data, robot_color):
#     global speed, angle, obj_detected, robot_to_escape_near, robot_to_catch_on_sight, robot_to_catch_near
#
#     if robot_to_catch_near:
#         # print(robot_color)
#         img = bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
#         # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#
#         # print(img)
#
#         # from color_segment.py
#         if robot_color == "Blue":
#             ranges_to_catch = red_ranges
#             ranges_to_escape = green_ranges
#             ranges_teammate = blue_ranges
#         if robot_color == "Green":
#             ranges_to_catch = blue_ranges
#             ranges_to_escape = red_ranges
#             ranges_teammate = green_ranges
#         if robot_color == "Red":
#             ranges_to_catch = green_ranges
#             ranges_to_escape = blue_ranges
#             ranges_teammate = red_ranges
#
#         # numpy arrays
#         lower_to_catch = np.array(
#             [ranges_to_catch['b']['min'], ranges_to_catch['g']['min'], ranges_to_catch['r']['min']])
#         upper_to_catch = np.array(
#             [ranges_to_catch['b']['max'], ranges_to_catch['g']['max'], ranges_to_catch['r']['max']])
#
#         lower_to_escape = np.array(
#             [ranges_to_escape['b']['min'], ranges_to_escape['g']['min'], ranges_to_escape['r']['min']])
#         upper_to_escape = np.array(
#             [ranges_to_escape['b']['max'], ranges_to_escape['g']['max'], ranges_to_escape['r']['max']])
#
#         lower_teammate = np.array(
#             [ranges_teammate['b']['min'], ranges_teammate['g']['min'], ranges_teammate['r']['min']])
#         upper_teammate = np.array(
#             [ranges_teammate['b']['max'], ranges_teammate['g']['max'], ranges_teammate['r']['max']])
#
#         # masks
#         mask_to_catch = cv2.inRange(img, lower_to_catch, upper_to_catch)
#         mask_to_escape = cv2.inRange(img, lower_to_escape, upper_to_escape)
#         mask_teammate = cv2.inRange(img, lower_teammate, upper_teammate)
#
#         # img dimensions
#         height, width, dimension = img.shape
#
#         # moments - center of the blob
#         M_to_catch = cv2.moments(mask_to_catch)
#         M_to_escape = cv2.moments(mask_to_escape)
#         M_teammate = cv2.moments(mask_teammate)
#
#         if M_to_catch['m00'] > 0 and not robot_to_escape_near and robot_to_catch_near:
#             print("Robot in CATCH mode")
#
#             robot_to_catch_on_sight = True
#
#             # edges = cv2.Canny(mask_to_catch, 100, 200)
#             edges = cv2.GaussianBlur(mask_to_catch, (7, 7), 2)
#             kernel = np.ones((7, 7), np.uint8)
#             edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
#             edges = cv2.medianBlur(edges, 9)
#             edges = cv2.dilate(edges, kernel, iterations=4)
#             kernel = np.ones((9, 9), np.uint8)
#             edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
#
#             contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#
#             # for contours in contours:
#             #     (x, y, w, h) = cv2.boundingRect(contours)
#             #     # draw the rectangle over detected object
#             #
#             #     if cv2.contourArea(contours) < 3000:
#             #         continue
#             #
#             #     cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
#             #     cv2.putText(edges, '+', (int(x+w/2), int(y+h/2)), cv2.FONT_ITALIC, 1, (255, 0, 0), 2, cv2.LINE_8)
#             #     # print(cv2.contourArea(contours))
#
#             if len(contours) != 0:
#                 # draw in blue the contours that were founded
#                 # cv2.drawContours(output, contours, -1, 255, 3)
#
#                 # find the biggest countour (c) by the area
#                 c = max(contours, key=cv2.contourArea)
#                 x, y, w, h = cv2.boundingRect(c)
#
#                 # draw the biggest contour (c) in green
#                 cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
#                 cv2.putText(edges, '+', (int(x + w / 2), int(y + h / 2)), cv2.FONT_ITALIC, 2, (255, 0, 0), 2,
#                             cv2.LINE_8)
#
#                 cx_to_catch = int(x + w / 2)
#                 cy_to_catch = int(y + h / 2)
#             else:
#                 cx_to_catch = int(M_to_catch['m10'] / M_to_catch['m00'])
#                 cy_to_catch = int(M_to_catch['m01'] / M_to_catch['m00'])
#                 # cv2.circle(img, (cx_to_catch, cy_to_catch), 20, (0, 0, 255), -1)
#                 cv2.putText(edges, '+', (cx_to_catch, cy_to_catch), cv2.FONT_ITALIC, 2, (255, 0, 0), 2, cv2.LINE_8)
#
#             # find center
#             threshold = cx_to_catch - width / 2
#
#             angle = threshold
#
#             if abs(angle) < 150:  # accelerate
#                 if speed < 1.0:
#                     speed += 0.1
#             else:
#                 if speed > 0.3:  # decelerate
#                     speed -= 0.05
#                 else:
#                     speed = 0.3
#
#             angle = -float(angle) / 500
#
#             # pub on cmd_vel
#             twist.linear.x = speed
#             twist.angular.z = angle
#             pub_cmd_vel.publish(twist)
#
#         else:  # not detected target
#             robot_to_catch_on_sight = False
#
#         cv2.imshow("mask", mask_to_catch)
#         cv2.imshow("image", img)
#         k = cv2.waitKey(1) & 0xFF
#     else:
#         pass


def laser_callback(laser_data):
    global speed, angle, obj_detected, robot_to_escape_near, robot_to_catch_near

    if not robot_to_escape_near and not robot_to_catch_near:
        regions = {
            # lidar front ranges
            'right': min(min(laser_data.ranges[295:348]), 10),
            'frontr': min(min(laser_data.ranges[349:359]), 10),
            'frontl': min(min(laser_data.ranges[0:10]), 10),
            'left': min(min(laser_data.ranges[11:63]), 10),
            # lidar back ranges
            'bright': min(min(laser_data.ranges[115:161]), 10),
            'backr': min(min(laser_data.ranges[162:179]), 10),
            'backl': min(min(laser_data.ranges[180:196]), 10),
            'bleft': min(min(laser_data.ranges[197:243]), 10),
        }

        lidar_max_dist_to_obj = 0.9
        lidar_angle_speed = 350

        if regions["frontr"] < lidar_max_dist_to_obj or regions["frontl"] < lidar_max_dist_to_obj:
            speed = 0
            obj_detected = True
            # angle = lidar_angle_speed

            # this is used when the robot as only blocking objects in front of them
            # otherwise it will stop moving
            if regions["right"] <= regions["left"]:
                angle = -lidar_angle_speed
            else:
                angle = lidar_angle_speed
            # **************************************

            # make the robot go backwards when it is too close to an object
            if regions["frontr"] < 0.2 or regions["frontl"] < 0.2:
                speed = -0.3
            # ******************************

            # angle decisions ********************
            if regions["right"] < lidar_max_dist_to_obj:
                angle = -lidar_angle_speed

            elif regions["left"] < lidar_max_dist_to_obj:
                angle = lidar_angle_speed
            # **********************

            angle = -float(angle) / 500
            print("wall detected in front")

            # pub on cmd_vel
            twist.linear.x = speed
            twist.angular.z = angle
            pub_cmd_vel.publish(twist)

        elif regions['right'] < lidar_max_dist_to_obj and regions["frontr"] > lidar_max_dist_to_obj and regions["frontl"] > lidar_max_dist_to_obj and regions['left'] > lidar_max_dist_to_obj:
            speed = 0.5
            angle = 0
            obj_detected = True

            print("wall detected on the right")

            # pub on cmd_vel
            twist.linear.x = speed
            twist.angular.z = angle
            pub_cmd_vel.publish(twist)

        elif regions['right'] > lidar_max_dist_to_obj and regions["frontr"] > lidar_max_dist_to_obj and regions["frontl"] > lidar_max_dist_to_obj and regions['left'] < lidar_max_dist_to_obj:
            speed = 0.5
            angle = 0
            obj_detected = True

            print("wall detected on the left")

            # pub on cmd_vel
            twist.linear.x = speed
            twist.angular.z = angle
            pub_cmd_vel.publish(twist)
        
        elif regions['right'] < lidar_max_dist_to_obj and regions['left'] < lidar_max_dist_to_obj:
            speed = 0.5
            angle = 0
            obj_detected = True

            print("wall detected on the sides")

            # pub on cmd_vel
            twist.linear.x = speed
            twist.angular.z = angle
            pub_cmd_vel.publish(twist)

        elif regions['right'] > lidar_max_dist_to_obj and regions['frontr'] > lidar_max_dist_to_obj and regions['frontl'] > lidar_max_dist_to_obj and regions['left'] > lidar_max_dist_to_obj:
            speed = speed
            angle = angle
            obj_detected = False

        # print(laser_data.ranges)
    else:
        speed = speed
        angle = angle
        obj_detected = False


def main():
    
    """
    Initialization *****************************
    """

    global speed, angle, robot_to_escape_near, robot_to_catch_near

    rospy.init_node('get_robot_position')  # init node
    rate = rospy.Rate(rate_hz)  # rate to sleep

    robot_name = rospy.get_param("~robot_name", default="blue1")  # robot player

    robot_color = rospy.get_param("~robot_color", default="Blue")  # robot color param

    robot_to_catch_1 = rospy.get_param("~robot_to_catch_1", default="red1")  # robot to catch 1
    robot_to_catch_2 = rospy.get_param("~robot_to_catch_2", default="red2")  # robot to catch 2
    robot_to_catch_3 = rospy.get_param("~robot_to_catch_3", default="red3")  # robot to catch 3

    robot_to_escape_1 = rospy.get_param("~robot_to_escape_1", default="green1")  # robot to escape 1
    robot_to_escape_2 = rospy.get_param("~robot_to_escape_2", default="green2")  # robot to escape 2
    robot_to_escape_3 = rospy.get_param("~robot_to_escape_3", default="green3")  # robot to escape 3

    # gazebo get model state
    rospy.wait_for_service(get_model_state)
    g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)

    goal_to_catch = PointStamped()
    goal_to_escape = PointStamped()

    # define object for converting to robot link
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    """
    Processing ************************************
    """

    while not rospy.is_shutdown():
        # reset the lists to store new position
        robot_to_catch_poses_x.clear()
        robot_to_catch_poses_y.clear()
        robot_to_escape_poses_x.clear()
        robot_to_escape_poses_y.clear()

        distances_to_catch.clear()
        distances_to_escape.clear()

        # get robot positions
        state_my_pos = g_get_state(model_name=robot_name)

        # to catch positions
        state_to_catch_1 = g_get_state(model_name=robot_to_catch_1)
        state_to_catch_2 = g_get_state(model_name=robot_to_catch_2)
        state_to_catch_3 = g_get_state(model_name=robot_to_catch_3)

        # to escape positions
        state_to_escape_1 = g_get_state(model_name=robot_to_escape_1)
        state_to_escape_2 = g_get_state(model_name=robot_to_escape_2)
        state_to_escape_3 = g_get_state(model_name=robot_to_escape_3)

        # print(state_my_pos)

        # player robot position
        x_pos = state_my_pos.pose.position.x
        y_pos = state_my_pos.pose.position.y
        z_pos = state_my_pos.pose.orientation.z
        # print(x_pos, y_pos)

        # position of targets to catch **************************
        x_to_catch_1 = state_to_catch_1.pose.position.x
        y_to_catch_1 = state_to_catch_1.pose.position.y
        z_ori_to_catch_1 = state_to_catch_1.pose.orientation.z
        x_to_catch_2 = state_to_catch_2.pose.position.x
        y_to_catch_2 = state_to_catch_2.pose.position.y
        z_ori_to_catch_2 = state_to_catch_2.pose.orientation.z
        x_to_catch_3 = state_to_catch_3.pose.position.x
        y_to_catch_3 = state_to_catch_3.pose.position.y
        z_ori_to_catch_3 = state_to_catch_3.pose.orientation.z

        robot_to_catch_poses_x.append(x_to_catch_1)
        robot_to_catch_poses_x.append(x_to_catch_2)
        robot_to_catch_poses_x.append(x_to_catch_3)
        robot_to_catch_poses_y.append(y_to_catch_1)
        robot_to_catch_poses_y.append(y_to_catch_2)
        robot_to_catch_poses_y.append(y_to_catch_3)
        robot_to_catch_ori_z.append(z_ori_to_catch_1)
        robot_to_catch_ori_z.append(z_ori_to_catch_2)
        robot_to_catch_ori_z.append(z_ori_to_catch_3)

        # distance to robot to catch
        distance_to_catch_1 = math.sqrt((x_to_catch_1 - x_pos) ** 2 + (y_to_catch_1 - y_pos) ** 2)
        distance_to_catch_2 = math.sqrt((x_to_catch_2 - x_pos) ** 2 + (y_to_catch_2 - y_pos) ** 2)
        distance_to_catch_3 = math.sqrt((x_to_catch_3 - x_pos) ** 2 + (y_to_catch_3 - y_pos) ** 2)
        distances_to_catch.append(distance_to_catch_1)
        distances_to_catch.append(distance_to_catch_2)
        distances_to_catch.append(distance_to_catch_3)

        # position of robots to escape **************************
        x_to_escape_1 = state_to_escape_1.pose.position.x
        y_to_escape_1 = state_to_escape_1.pose.position.y
        z_ori_to_escape_1 = state_to_escape_1.pose.orientation.z
        x_to_escape_2 = state_to_escape_2.pose.position.x
        y_to_escape_2 = state_to_escape_2.pose.position.y
        z_ori_to_escape_2 = state_to_escape_2.pose.orientation.z
        x_to_escape_3 = state_to_escape_3.pose.position.x
        y_to_escape_3 = state_to_escape_3.pose.position.y
        z_ori_to_escape_3 = state_to_escape_3.pose.orientation.z

        robot_to_escape_poses_x.append(x_to_escape_1)
        robot_to_escape_poses_x.append(x_to_escape_2)
        robot_to_escape_poses_x.append(x_to_escape_3)
        robot_to_escape_poses_y.append(y_to_escape_1)
        robot_to_escape_poses_y.append(y_to_escape_2)
        robot_to_escape_poses_y.append(y_to_escape_3)
        robot_to_escape_ori_z.append(z_ori_to_escape_1)
        robot_to_escape_ori_z.append(z_ori_to_escape_2)
        robot_to_escape_ori_z.append(z_ori_to_escape_3)

        # distances to robots to escape
        distance_to_escape_1 = math.sqrt((x_to_escape_1 - x_pos) ** 2 + (y_to_escape_1 - y_pos) ** 2)
        distance_to_escape_2 = math.sqrt((x_to_escape_2 - x_pos) ** 2 + (y_to_escape_2 - y_pos) ** 2)
        distance_to_escape_3 = math.sqrt((x_to_escape_3 - x_pos) ** 2 + (y_to_escape_3 - y_pos) ** 2)
        distances_to_escape.append(distance_to_escape_1)
        distances_to_escape.append(distance_to_escape_2)
        distances_to_escape.append(distance_to_escape_3)

        thresh_dist_to_escape = 0.01  # max proximity from robot to escape
        thresh_dist_to_catch = 1  # proximity from robot to catch

        # change from catch mode to escape mode
        if distance_to_escape_1 < thresh_dist_to_escape or distance_to_escape_2 < thresh_dist_to_escape or distance_to_escape_3 < thresh_dist_to_escape:
            robot_to_escape_near = True
        else:
            robot_to_escape_near = False

        if distance_to_catch_1 < thresh_dist_to_catch or distance_to_catch_2 < thresh_dist_to_catch or distance_to_catch_3 < thresh_dist_to_catch:
            robot_to_catch_near = True
        else:
            robot_to_catch_near = False

        # catch mode ********************************
        if not robot_to_escape_near:
            # final pose to catch *****************************

            # find min distance to a robot to catch
            min_dist_to_catch = min(distances_to_catch)  # find the closest robot to catch
            # print(min_dist_to_catch)

            # decide what robot to catch (the closer one)
            if min_dist_to_catch == distances_to_catch[0]:
                goal_to_catch = state_to_catch_1
            if min_dist_to_catch == distances_to_catch[1]:
                goal_to_catch = state_to_catch_2
            if min_dist_to_catch == distances_to_catch[2]:
                goal_to_catch = state_to_catch_3

            # print(goal_to_catch)

            # create a pointstamp position equal to the robot to catch position to fed intro tf transformation
            goal_to_catch_present_time = PointStamped()
            goal_to_catch_present_time.header.stamp = rospy.Time.now()
            goal_to_catch_present_time.header.frame_id = robot_name + "/odom"
            goal_to_catch_present_time.point.x = goal_to_catch.pose.position.x
            goal_to_catch_present_time.point.y = goal_to_catch.pose.position.y
            goal_to_catch_present_time.point.z = goal_to_catch.pose.position.z
            # print(goal_to_catch_present_time)

            # create tf transformation
            target_frame = robot_name + "/base_link"  # frame to transform final position
            # print(target_frame)

            goal_in_base_link_to_catch = tf_buffer.transform(goal_to_catch_present_time, target_frame, rospy.Duration(1))
            # print(goal_in_base_link_to_catch)

            # go to that position
            x = goal_in_base_link_to_catch.point.x
            y = goal_in_base_link_to_catch.point.y
            orientation = goal_in_base_link_to_catch.point.z

            if not obj_detected:
                if abs(orientation) > 0.5:  # decelerate and turn to face target
                    angle = math.atan2(y, x)
                    if speed >= 0.5:
                        speed -= 0.15
                    else:
                        speed = 0.5
                else:
                    angle = math.atan2(y, x)

                    if abs(angle) < 0.20:  # accelerate
                        if speed < max_speed:
                            speed += 0.1
                    else:
                        speed = 0.5

                # pub on cmd_vel
                twist.linear.x = speed
                twist.angular.z = angle
                pub_cmd_vel.publish(twist)
                print("no wall detected")

            # lidar code *********************************************
            rospy.Subscriber(lidar_node, LaserScan, laser_callback)
            
            # camera code ********************************************
            # img_sub = rospy.Subscriber(camera_node, Image, partial(image_callback, robot_color=robot_color))

            # print(speed, angle)

            # print robot to catch position
            print(f'Red at position {round(goal_to_catch.pose.position.x, 1)},{round(goal_to_catch.pose.position.y, 1)}. Moving at speed:{round(speed, 1)}, angle:{round(angle, 1)}')

        # escape mode ******************************************
        else:
            # IGNORE ESCAPE MODE FOR NOW *********************** 
            # final pose to escape *****************************
            pass
            # # find min distance to a robot to escape
            # min_dist_to_escape = min(distances_to_escape)  # find the closest robot to escape
            #
            # # decide what robot to escape (the closer one)
            # if min_dist_to_escape == distances_to_escape[0]:
            #     goal_to_escape = state_to_escape_1
            # if min_dist_to_escape == distances_to_escape[1]:
            #     goal_to_escape = state_to_escape_2
            # if min_dist_to_escape == distances_to_escape[2]:
            #     goal_to_escape = state_to_escape_3
            #
            # # print(goal_to_escape)
            #
            # # create a pointstamp position equal to the robot to escape position to fed intro tf transformation
            # goal_to_escape_present_time = PointStamped()
            # goal_to_escape_present_time.header.stamp = rospy.Time.now()
            # goal_to_escape_present_time.header.frame_id = robot_name + "/odom"
            # goal_to_escape_present_time.point.x = goal_to_escape.pose.position.x
            # goal_to_escape_present_time.point.y = goal_to_escape.pose.position.y
            # goal_to_escape_present_time.point.z = goal_to_escape.pose.position.z
            # # print(goal_to_escape_present_time)
            #
            # # create tf transformation
            # target_frame = robot_name + "/base_link"  # frame to transform final position
            # # print(target_frame)
            #
            # goal_in_base_link_to_escape = tf_buffer.transform(goal_to_escape_present_time, target_frame, rospy.Duration(1))
            # # print(goal_in_base_link_to_escape)
            #
            # x_to_escape = goal_to_escape.pose.position.x
            # y_to_escape = goal_to_escape.pose.position.y
            #
            # # go to safe position
            # x = goal_in_base_link_to_escape.point.x
            # y = goal_in_base_link_to_escape.point.y
            # orientation = goal_in_base_link_to_escape.point.z
            #
            # if x >= 0:
            #     x = x - 5
            #     y = y
            # else:
            #     x = x + 5
            #     y = y
            #
            # # keep the robot in the boundaries of the arena *******************
            # # x limits
            # if x > x_arena_max:
            #     x = x_arena_max
            #
            # if x < x_arena_min:
            #     x = x_arena_min
            #
            # # y limits
            # if y > y_arena_max:
            #     y = y_arena_max
            #
            # if y < y_arena_min:
            #     y = y_arena_min
            # # ******************************************************************
            #
            # if abs(orientation) > 0.5:  # decelerate and turn to face target
            #     angle = math.atan2(y, x)
            #     if speed >= 0:
            #         speed -= 0.5
            #     else:
            #         speed = 0
            #
            # else:  # accelerate to target
            #     angle = math.atan2(y, x)
            #
            #     if abs(angle) < 0.10:  # accelerate
            #         if speed < max_speed:
            #             speed += 0.1
            #     else:
            #         speed = speed
            #
            # # pub on cmd_vel
            # twist.linear.x = -speed
            # twist.angular.z = angle
            # pub_cmd_vel.publish(twist)
            #
            # print(f'Green at position {x_to_escape},{y_to_escape} go to position {x}, {y} at speed:{speed}, angle:{angle}')

        rate.sleep()
        # rospy.spin()

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
