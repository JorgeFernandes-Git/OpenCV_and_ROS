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

        # define speed commands
        self.twist = Twist()
        self.speed = 0
        self.angle = 0

        # define object for final position
        self.goal = PointStamped()
        self.goal_active = False
        self.distance_to_goal = 0

        # define object for converting to robot link
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # define pubs and subs
        self.pub = rospy.Publisher(cmd_vel_node, Twist, queue_size=10)
        self.sub = rospy.Subscriber(pose_stamped_node, PoseStamped, self.goal_callback)

        # define time to call function
        self.timer = rospy.Timer(rospy.Duration(0.1), self.send_command_callback)  # call function in each 0.1 seconds

    def goal_callback(self, goal_msg):
        self.goal = goal_msg  # give goal x, y, z coordinates
        self.goal_active = True  # goal received flag

    def send_command_callback(self, msg):

        # if goal position is received
        if self.goal_active:
            self.speed, self.angle, self.distance_to_goal = self.drive_straight(self.goal)

            # verify if goal achieved
            if self.distance_to_goal < 0.1:
                self.goal_active = False

        else:  # if no goal stop
            self.speed = 0
            self.angle = 0

        # publish a speed msg
        self.twist.linear.x = self.speed
        self.twist.angular.z = self.angle
        self.pub.publish(self.twist)

    def drive_straight(self, goal, speed_min=0.1, speed_max=1.0):

        # for a copy of the goal to keep goal in buffer and don't lose actual position over time
        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        # print(goal_present_time)

        target_frame = "p_jfernandes/base_link"  # frame to transform final position
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))
            # print(goal_in_base_link)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f'Could not transform goal from {goal_present_time.header.frame_id} to {target_frame}')
            self.speed = 0
            self.angle = 0
            return self.speed, self.angle

        # final x, y and orientation coordinates converted to base link
        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y
        orientation = goal_in_base_link.pose.orientation.z

        # print(orientation)

        # calculate distance to goal
        self.distance_to_goal = math.sqrt(x**2 + y**2)

        # calculate angle and speed based on distance
        if abs(orientation) > 0.5:  # stay still until orientation are similar
            self.angle = math.atan2(y, x)
            self.speed = 0
        else:
            self.angle = math.atan2(y, x)
            self.speed = 0.5 * self.distance_to_goal  # decelerate based on distance to goal

        self.speed = min(self.speed, speed_max)
        self.speed = max(self.speed, speed_min)

        return self.speed, self.angle, self.distance_to_goal


def main():
    rospy.init_node("driver")
    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
