#! /usr/bin/env python3

import math
from gazebo_msgs.srv import GetModelState
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# variables
x_arena_min = -8
x_arena_max = 8
y_arena_min = 2
y_arena_max = -2

# list to store positions
robot_to_catch_poses_x = []
robot_to_catch_poses_y = []
robot_to_escape_poses_x = []
robot_to_escape_poses_y = []

# list to store distances
distances_to_catch = []
distances_to_escape = []

# rate_hz = rospy.get_param("~rate")
rate_hz = 0.5

# nodes
get_model_state = rospy.remap_name("/gazebo/get_model_state")


def main():
    rospy.init_node('get_robot_position')  # init node
    rate = rospy.Rate(rate_hz)  # rate to sleep

    robot_name = rospy.get_param("~robot_name", default="blue1")  # robot player

    robot_to_catch_1 = rospy.get_param("~robot_to_catch_1", default="red1")  # robot to catch 1
    robot_to_catch_2 = rospy.get_param("~robot_to_catch_2", default="red2")  # robot to catch 2
    robot_to_catch_3 = rospy.get_param("~robot_to_catch_3", default="red3")  # robot to catch 3

    robot_to_escape_1 = rospy.get_param("~robot_to_escape_1", default="green1")  # robot to escape 1
    robot_to_escape_2 = rospy.get_param("~robot_to_escape_2", default="green2")  # robot to escape 2
    robot_to_escape_3 = rospy.get_param("~robot_to_escape_3", default="green3")  # robot to escape 3

    # pose stamped objects
    goal_to_catch = MoveBaseGoal()
    goal_to_escape = MoveBaseGoal()

    # gazebo get model state
    rospy.wait_for_service(get_model_state)
    g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient(robot_name, MoveBaseAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    while True:
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
        # print(x_pos, y_pos)

        # position of targets to catch **************************
        x_to_catch_1 = state_to_catch_1.pose.position.x
        y_to_catch_1 = state_to_catch_1.pose.position.y
        x_to_catch_2 = state_to_catch_2.pose.position.x
        y_to_catch_2 = state_to_catch_2.pose.position.y
        x_to_catch_3 = state_to_catch_3.pose.position.x
        y_to_catch_3 = state_to_catch_3.pose.position.y

        robot_to_catch_poses_x.append(x_to_catch_1)
        robot_to_catch_poses_x.append(x_to_catch_2)
        robot_to_catch_poses_x.append(x_to_catch_3)
        robot_to_catch_poses_y.append(y_to_catch_1)
        robot_to_catch_poses_y.append(y_to_catch_2)
        robot_to_catch_poses_y.append(y_to_catch_3)

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
        x_to_escape_2 = state_to_escape_2.pose.position.x
        y_to_escape_2 = state_to_escape_2.pose.position.y
        x_to_escape_3 = state_to_escape_3.pose.position.x
        y_to_escape_3 = state_to_escape_3.pose.position.y

        robot_to_escape_poses_x.append(x_to_escape_1)
        robot_to_escape_poses_x.append(x_to_escape_2)
        robot_to_escape_poses_x.append(x_to_escape_3)
        robot_to_escape_poses_y.append(y_to_escape_1)
        robot_to_escape_poses_y.append(y_to_escape_2)
        robot_to_escape_poses_y.append(y_to_escape_3)

        # distances to robots to escape
        distance_to_escape_1 = math.sqrt((x_to_escape_1 - x_pos) ** 2 + (y_to_escape_1 - y_pos) ** 2)
        distance_to_escape_2 = math.sqrt((x_to_escape_2 - x_pos) ** 2 + (y_to_escape_2 - y_pos) ** 2)
        distance_to_escape_3 = math.sqrt((x_to_escape_3 - x_pos) ** 2 + (y_to_escape_3 - y_pos) ** 2)
        distances_to_escape.append(distance_to_escape_1)
        distances_to_escape.append(distance_to_escape_2)
        distances_to_escape.append(distance_to_escape_3)

        thresh_dist_to_escape = 3  # max proximity from robot to escape

        # change from catch mode to escape mode
        if distance_to_escape_1 < thresh_dist_to_escape or distance_to_escape_2 < thresh_dist_to_escape or distance_to_escape_3 < thresh_dist_to_escape:
            to_escape_near = True
        else:
            to_escape_near = False

        # catch mode or escape mode
        if not to_escape_near:
            # final pose to catch *****************************
            goal_to_catch.target_pose.header.stamp = rospy.Time.now()
            goal_to_catch.target_pose.header.frame_id = "map"

            min_dist_to_catch = min(distances_to_catch)  # find the closest robot to catch
            # print(min_dist_to_catch)

            if min_dist_to_catch == distances_to_catch[0]:
                goal_to_catch.target_pose.pose.position.x = x_to_catch_1
                goal_to_catch.target_pose.pose.position.y = y_to_catch_1
            if min_dist_to_catch == distances_to_catch[1]:
                goal_to_catch.target_pose.pose.position.x = x_to_catch_2
                goal_to_catch.target_pose.pose.position.y = y_to_catch_2
            if min_dist_to_catch == distances_to_catch[2]:
                goal_to_catch.target_pose.pose.position.x = x_to_catch_3
                goal_to_catch.target_pose.pose.position.y = y_to_catch_3

            goal_to_catch.target_pose.pose.orientation.w = 0.5

            print(f'Red at position {goal_to_catch.target_pose.pose.position.x},{goal_to_catch.target_pose.pose.position.y}')

            # Sends the goal to the action server.
            client.send_goal(goal_to_catch)

        else:
            # final pose to escape *****************************
            goal_to_escape.target_pose.header.stamp = rospy.Time.now()
            goal_to_escape.target_pose.header.frame_id = "map"

            min_dist_to_escape = min(distances_to_escape)  # find the closest robot to escape

            if min_dist_to_escape == distances_to_escape[0]:
                goal_to_escape.target_pose.pose.position.x = x_to_escape_1
                goal_to_escape.target_pose.pose.position.y = y_to_escape_1
            if min_dist_to_escape == distances_to_escape[1]:
                goal_to_escape.target_pose.pose.position.x = x_to_escape_2
                goal_to_escape.target_pose.pose.position.y = y_to_escape_2
            if min_dist_to_escape == distances_to_escape[2]:
                goal_to_escape.target_pose.pose.position.x = x_to_escape_3
                goal_to_escape.target_pose.pose.position.y = y_to_escape_3

            x_to_escape = goal_to_escape.target_pose.pose.position.x
            y_to_escape = goal_to_escape.target_pose.pose.position.y

            if x_to_escape >= 0:
                goal_to_escape.target_pose.pose.position.x = x_to_escape - 5
                goal_to_escape.target_pose.pose.position.y = y_to_escape
            else:
                goal_to_escape.target_pose.pose.position.x = x_to_escape + 5
                goal_to_escape.target_pose.pose.position.y = y_to_escape

            # keep the robot in the boundaries of the arena *******************
            # x limits
            if goal_to_escape.target_pose.pose.position.x > x_arena_max:
                goal_to_escape.target_pose.pose.position.x = x_arena_max

            if goal_to_escape.target_pose.pose.position.x < x_arena_min:
                goal_to_escape.target_pose.pose.position.x = x_arena_min

            # y limits
            if goal_to_escape.target_pose.pose.position.y > y_arena_max:
                goal_to_escape.target_pose.pose.position.y = y_arena_max

            if goal_to_escape.target_pose.pose.position.y < y_arena_min:
                goal_to_escape.target_pose.pose.position.y = y_arena_min
            # ******************************************************************

            goal_to_escape.target_pose.pose.orientation.w = 0.5

            print(f'Green at position {x_to_escape},{y_to_escape} go to position '
                  f'{goal_to_escape.target_pose.pose.position.x},{goal_to_escape.target_pose.pose.position.y}')

            # Sends the goal to the action server.
            client.send_goal(goal_to_escape)

        rate.sleep()


if __name__ == '__main__':
    main()
