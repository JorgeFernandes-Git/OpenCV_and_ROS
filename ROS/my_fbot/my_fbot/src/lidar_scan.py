#!/usr/bin/python3

from functools import partial
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math
import random


###################
# rosrun lidar_example lidar_subscriber.py
# rostopic list
# rostopic echo /left_laser/point_cloud --noarr
###################


# function to create marker
# def create_marker():
#     # Create marker
#     marker = Marker()
#     marker.header.frame_id = "left_laser"
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "my_namespace"
#     marker.id = 0
#     marker.type = Marker.SPHERE_LIST
#     marker.pose.orientation.w = 1.0 # otherwise quaternion is not normalized
#     marker.scale.x = 0.1
#     marker.scale.y = 0.1
#     marker.scale.z = 0.1
#     marker.color.a = 0.5
#     marker.color.r = random.random()
#     marker.color.g = random.random()
#     marker.color.b = random.random()
#
#     return marker


def callback_msg_received(msg):
    # rospy.loginfo("Received laser scan message")

    # marker_array = MarkerArray()

    z = 0
    for idx, range in enumerate(msg.ranges):
        print(range)

        # if range < 0.1:  # 100 cm is the min distance so don't attach the point if tit's under
        #     continue

        # print(range)

    #     # convert from polar to cartesian coordinates
    #     theta = msg.angle_min + msg.angle_increment * idx
    #     x = range * math.cos(theta)
    #     y = range * math.sin(theta)
    #
    #     # Should I create a new cluster?
    #     dist = math.sqrt((x_prev - x) ** 2 + (y_prev - y) ** 2)  # dist between points
    #
    #     if dist > dist_threshold:  # new cluster
    #         marker = create_marker()
    #         idx_group = len(marker_array.markers)
    #         marker.id = idx_group  # give new id to marker
    #
    #         marker.points = []
    #         marker_array.markers.append(marker)
    #
    #     last_marker = marker_array.markers[-1]  # last item of the list
    #     last_marker.points.append(Point(x=x, y=y, z=z))
    #
    #     # print(last_marker)
    #
    #     # calculate the centroid
    #     # cent_x = [p[0] for p in last_marker.pose.position.x]
    #     # cent_y = [p[1] for p in marker]
    #     # centroid = (sum(cent_x) / len(marker_array.markers), sum(cent_y) / len(marker_array.markers))
    #     # print(centroid)
    #
    #     x_prev = x
    #     y_prev = y
    #
    # publisher.publish(marker_array)
    # rospy.loginfo('Published marker array')


def main():
    # subscriber
    rospy.init_node("lidar_sub_node")
    rospy.Subscriber("/scan", LaserScan, callback_msg_received)
    rospy.spin()


if __name__ == '__main__':
    main()
