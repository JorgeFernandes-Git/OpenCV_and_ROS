#! /usr/bin/env python3

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
import rospy

# variables
robot_name = rospy.remap_name("p_jfernandes")
# rate_hz = rospy.get_param("~rate")
rate_hz = 1.0

# nodes
pose_stamped_node = rospy.remap_name("Green/move_base_simple/goal")
get_model_state = rospy.remap_name("/gazebo/get_model_state")


def main():
    rospy.init_node('get_robot_position_Green')
    rate = rospy.Rate(rate_hz)

    pub = rospy.Publisher(pose_stamped_node, PoseStamped, queue_size=10)
    goal = PoseStamped()

    while True:
        g_get_state = rospy.ServiceProxy(get_model_state, GetModelState)
        rospy.wait_for_service(get_model_state)

        try:
            state = g_get_state(model_name=robot_name)
        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))
            return

        x = state.pose.position.x
        y = state.pose.position.y

        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 0.5

        pub.publish(goal)

        print(f'{robot_name} at position {x},{y}')
        # print(f'x: {x}, y: {y}')
        # print(state.pose)
        rate.sleep()


if __name__ == '__main__':
    main()
