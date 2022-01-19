#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelState


def callback_msg_received():
    pass
    # model_state = ModelState
    #
    # model_state.


def main():
    rospy.init_node("model_state_node")
    rospy.Subscriber("gazebo/model_states", ModelState, callback_msg_received)
    rospy.spin()


if __name__ == '__main__':
    main()
