#!/usr/bin/python3
import copy
import math

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped, Twist

# topics
cmd_vel_node = rospy.remap_name("/p_jfernandes/cmd_vel")
pose_stamped_node = rospy.remap_name("/move_base_simple/goal")


class Driver:
    def __init__(self):
        self.twist = Twist()
        self.speed = 0
        self.angle = 0

        self.goal = PointStamped()
        self.goal_active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)
        self.sub = rospy.Subscriber(pose_stamped_node, PoseStamped, self.goal_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.send_command_callback)

    def goal_callback(self, goal_msg):
        self.goal = goal_msg
        self.goal_active = True

    def send_command_callback(self, msg):

        if self.goal_active:
            self.speed, self.angle = self.drive_straight(self.goal)
        else:
            self.speed = 0
            self.angle = 0

        self.twist.linear.x = self.speed
        self.twist.angular.z = self.angle
        self.pub.publish(self.twist)

    def drive_straight(self, goal, speed_min=0.1, speed_max=0.9):

        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        # print(goal_present_time)

        target_frame = "p_jfernandes/base_link"
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))
            # print(goal_in_base_link)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f'Could not transform goal from {goal_present_time.header.frame_id} to {target_frame}')
            self.speed = 0
            self.angle = 0
            return self.speed, self.angle

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)

        self.speed = min(self.speed, speed_max)
        self.speed = max(self.speed, speed_min)

        return self.speed, self.angle


def main():
    rospy.init_node("move_to_goal")
    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
