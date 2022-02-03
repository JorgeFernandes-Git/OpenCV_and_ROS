#! /usr/bin/env python3

import math

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import rospy

# rate_hz = rospy.get_param("~rate")
rate_hz = 1

# nodes
get_model_state = rospy.remap_name("/gazebo/get_model_state")


def main():
    rospy.init_node('get_robot_position')
    # rate = rospy.Rate(rate_hz)

    robot_name = rospy.get_param("~robot_name", default="blue1")  # robot player

    robot_to_catch_1 = rospy.get_param("~robot_to_catch_1", default="red1")  # robot to catch
    robot_to_catch_2 = rospy.get_param("~robot_to_catch_2", default="red2")  # robot to catch
    robot_to_catch_3 = rospy.get_param("~robot_to_catch_3", default="red3")  # robot to catch

    robot_to_escape_1 = rospy.get_param("~robot_to_escape_1", default="green1")  # robot to escape
    robot_to_escape_2 = rospy.get_param("~robot_to_escape_2", default="green2")  # robot to escape
    robot_to_escape_3 = rospy.get_param("~robot_to_escape_3", default="green3")  # robot to escape

    rospy.wait_for_service(get_model_state)
    g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)

    while True:
        state_my_pos = g_get_state(model_name=robot_name)

        state_to_catch_1 = g_get_state(model_name=robot_to_catch_1)
        state_to_catch_2 = g_get_state(model_name=robot_to_catch_2)
        state_to_catch_3 = g_get_state(model_name=robot_to_catch_3)

        state_to_escape_1 = g_get_state(model_name=robot_to_escape_1)
        state_to_escape_2 = g_get_state(model_name=robot_to_escape_2)
        state_to_escape_3 = g_get_state(model_name=robot_to_escape_3)

        # print(state_to_catch_1)


if __name__ == '__main__':
    main()
