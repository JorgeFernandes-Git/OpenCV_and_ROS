#! /usr/bin/env python3
import math

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import rospy

# variables
# robot_name = rospy.remap_name("Red")

# rate_hz = rospy.get_param("~rate")
rate_hz = 1.0

# nodes
pose_stamped_node = rospy.remap_name("p_jfernandes/move_base_simple/goal")
get_model_state = rospy.remap_name("/gazebo/get_model_state")


def main():
    rospy.init_node('get_robot_position')
    rate = rospy.Rate(rate_hz)

    robot_name = rospy.get_param("~robot_name", default="p_jfernandes")  # robot player
    robot_to_catch = rospy.get_param("~robot_to_catch", default="Red")  # robot to catch
    robot_to_escape = rospy.get_param("~robot_to_escape", default="Green")  # robot to escape

    pub = rospy.Publisher(pose_stamped_node, PoseStamped, queue_size=10)  # publisher on the move_base_simple/goal topic

    # pose stamped objects
    goal_to_catch = PoseStamped()
    goal_to_escape = PoseStamped()

    while True:
        g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)
        rospy.wait_for_service(get_model_state)

        try:
            state_my_pos = g_get_state(model_name=robot_name)
            state_to_catch = g_get_state(model_name=robot_to_catch)
            state_to_escape = g_get_state(model_name=robot_to_escape)
        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))
            return

        # player robot position
        x_pos = state_my_pos.pose.position.x
        y_pos = state_my_pos.pose.position.y

        # position of targets
        x_to_catch = state_to_catch.pose.position.x
        y_to_catch = state_to_catch.pose.position.y
        
        x_to_escape = state_to_escape.pose.position.x
        y_to_escape = state_to_escape.pose.position.y

        # distance to robot to escape
        distance_to_escape = math.sqrt((x_to_escape-x_pos)**2 + (y_to_escape-y_pos)**2)
        thresh_dist_to_escape = 5   # max proximity from robot to escape

        if distance_to_escape > thresh_dist_to_escape:

            # final pose to catch *****************************
            goal_to_catch.header.stamp = rospy.Time.now()
            goal_to_catch.header.frame_id = "map"

            goal_to_catch.pose.position.x = x_to_catch
            goal_to_catch.pose.position.y = y_to_catch
            goal_to_catch.pose.orientation.w = 0.5

            print(f'{robot_to_catch} at position {x_to_catch},{y_to_catch}')
            pub.publish(goal_to_catch)

        else:
            # must escape from catcher so what to do?????
            # final pose to escape *****************************
            goal_to_escape.header.stamp = rospy.Time.now()
            goal_to_escape.header.frame_id = "map"

            # go to new position base on robot to escape position
            if x_to_escape > 0:
                goal_to_escape.pose.position.x = x_to_escape - 7
                goal_to_escape.pose.position.y = y_to_escape
            else:
                goal_to_escape.pose.position.x = x_to_escape + 7
                goal_to_escape.pose.position.y = y_to_escape

            goal_to_escape.pose.orientation.w = 0.5

            print(f'{robot_to_escape} at position {x_to_escape},{y_to_escape} go to position '
                  f'{goal_to_escape.pose.position.x},{ goal_to_escape.pose.position.y}')
            pub.publish(goal_to_escape)

        rate.sleep()


if __name__ == '__main__':
    main()
