#! /usr/bin/env python3

import math

from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import PoseStamped
import rospy

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
pose_stamped_node = rospy.remap_name("p_jfernandes/move_base_simple/goal")  # in rviz 2D Nav Goal
get_model_state = rospy.remap_name("/gazebo/get_model_state")


def main():
    rospy.init_node('get_robot_position')
    rate = rospy.Rate(rate_hz)

    robot_name = rospy.get_param("~robot_name", default="blue1")  # robot player

    robot_to_catch_1 = rospy.get_param("~robot_to_catch_1", default="red1")  # robot to catch
    robot_to_catch_2 = rospy.get_param("~robot_to_catch_2", default="red2")  # robot to catch
    robot_to_catch_3 = rospy.get_param("~robot_to_catch_3", default="red3")  # robot to catch

    robot_to_escape_1 = rospy.get_param("~robot_to_escape_1", default="green1")  # robot to escape
    robot_to_escape_2 = rospy.get_param("~robot_to_escape_2", default="green2")  # robot to escape
    robot_to_escape_3 = rospy.get_param("~robot_to_escape_3", default="green3")  # robot to escape

    # topics to publish on
    pub = rospy.Publisher(pose_stamped_node, PoseStamped, queue_size=10)  # publisher on the move_base_simple/goal topic

    # pose stamped objects
    goal_to_catch = PoseStamped()
    goal_to_escape = PoseStamped()

    while True:
        g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)
        rospy.wait_for_service(get_model_state)

        try:
            state_my_pos = g_get_state(model_name=robot_name)

            state_to_catch_1 = g_get_state(model_name=robot_to_catch_1)
            state_to_catch_2 = g_get_state(model_name=robot_to_catch_2)
            state_to_catch_3 = g_get_state(model_name=robot_to_catch_3)

            state_to_escape_1 = g_get_state(model_name=robot_to_escape_1)
            state_to_escape_2 = g_get_state(model_name=robot_to_escape_2)
            state_to_escape_3 = g_get_state(model_name=robot_to_escape_3)

        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))
            return

        # player robot position
        x_pos = state_my_pos.pose.position.x
        y_pos = state_my_pos.pose.position.y

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

        # position of targets to escape **************************
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

        # distance to robot to escape
        distance_to_escape_1 = math.sqrt((x_to_escape_1 - x_pos) ** 2 + (y_to_escape_1 - y_pos) ** 2)
        distance_to_escape_2 = math.sqrt((x_to_escape_2 - x_pos) ** 2 + (y_to_escape_2 - y_pos) ** 2)
        distance_to_escape_3 = math.sqrt((x_to_escape_3 - x_pos) ** 2 + (y_to_escape_3 - y_pos) ** 2)
        distances_to_escape.append(distance_to_escape_1)
        distances_to_escape.append(distance_to_escape_2)
        distances_to_escape.append(distance_to_escape_3)

        thresh_dist_to_escape = 3  # max proximity from robot to escape

        for i, dist_to_escape in enumerate(distances_to_escape):
            if dist_to_escape > thresh_dist_to_escape:

                # final pose to catch *****************************
                goal_to_catch.header.stamp = rospy.Time.now()
                goal_to_catch.header.frame_id = "map"

                min_dist_to_catch = min(distances_to_catch)

                if min_dist_to_catch == distances_to_catch[0]:
                    goal_to_catch.pose.position.x = x_to_catch_1
                    goal_to_catch.pose.position.y = y_to_catch_1
                if min_dist_to_catch == distances_to_catch[1]:
                    goal_to_catch.pose.position.x = x_to_catch_2
                    goal_to_catch.pose.position.y = y_to_catch_2
                if min_dist_to_catch == distances_to_catch[2]:
                    goal_to_catch.pose.position.x = x_to_catch_3
                    goal_to_catch.pose.position.y = y_to_catch_3

                goal_to_catch.pose.orientation.w = 0.5

                # print(f'{robot_to_catch} at position {x_to_catch},{y_to_catch}')
                pub.publish(goal_to_catch)

            else:
                # final pose to escape *****************************
                goal_to_escape.header.stamp = rospy.Time.now()
                goal_to_escape.header.frame_id = "map"

                x_to_escape = robot_to_escape_poses_x[i]
                y_to_escape = robot_to_escape_poses_y[i]
                # THIS PART CAN BE IMPROVED IF THE ARENA AS ONLY POSITIVE COORDINATES
                # # go to new position base on robot to escape position
                if x_to_escape >= 0:
                    goal_to_escape.pose.position.x = x_to_escape - 5
                    goal_to_escape.pose.position.y = y_to_escape
                else:
                    goal_to_escape.pose.position.x = x_to_escape + 5
                    goal_to_escape.pose.position.y = y_to_escape

                # keep the robot in the boundaries of the arena *******************
                # x limits
                if goal_to_escape.pose.position.x > x_arena_max:
                    goal_to_escape.pose.position.x = x_arena_max

                if goal_to_escape.pose.position.x < x_arena_min:
                    goal_to_escape.pose.position.x = x_arena_min

                # y limits
                if goal_to_escape.pose.position.y > y_arena_max:
                    goal_to_escape.pose.position.y = y_arena_max

                if goal_to_escape.pose.position.y < y_arena_min:
                    goal_to_escape.pose.position.y = y_arena_min
                # ******************************************************************

                goal_to_escape.pose.orientation.w = 0.5

                # print(f'{robot_to_escape} at position {x_to_escape},{y_to_escape} go to position '
                #       f'{goal_to_escape.pose.position.x},{goal_to_escape.pose.position.y}')
                pub.publish(goal_to_escape)

        rate.sleep()


if __name__ == '__main__':
    main()
