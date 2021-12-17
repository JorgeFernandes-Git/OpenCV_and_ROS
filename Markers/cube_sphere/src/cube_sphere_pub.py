#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random


# function to create marker
def create_marker(ns, marker_form, x_pos, y_pos,
                  z_pos, x_scl, y_scl, z_scl, a, r, g, b):
    # Create marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = 0
    marker.type = marker_form
    marker.pose.position.x = x_pos
    marker.pose.position.y = y_pos
    marker.pose.position.z = z_pos
    marker.pose.orientation.w = 1.0  # otherwise, quaternion is not normalized
    marker.scale.x = x_scl
    marker.scale.y = y_scl
    marker.scale.z = z_scl
    marker.color.a = a
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    return marker


def main():
    rospy.init_node("publisher", anonymous=False)
    pub = rospy.Publisher("/cube_sphere", MarkerArray, queue_size=1)
    rate = rospy.Rate(10)

    idx = 0  # counter
    increment = 0.1  # increment
    limit = 3  # limit

    # create marker array
    marker_array = MarkerArray()

    # list of objects fix
    marker_list_spheres = create_marker("sphere_list", Marker.SPHERE_LIST, 0, 0, 0, 1, 1, 1, 0.8, 1, 1, 0)
    marker_list_spheres.points = []
    for i in range(0, 10):  # random positions for create the objects
        x = random.randint(-3, 3)
        y = random.randint(-3, 3)
        z = random.randint(-3, 3)
        marker_list_spheres.points.append(Point(x=x, y=y, z=z))

    # list of cubes
    marker_list_cubes = create_marker("cube_list", Marker.CUBE_LIST, 0, 0, 0, 0.5, 0.5, 0.5, 0.8, 1, 1, 0)
    marker_list_cubes.points = []
    for i in range(0, 5):  # random positions for create the objects
        x = random.randint(-3, 3)
        y = random.randint(-3, 3)
        z = random.randint(-3, 3)
        marker_list_cubes.points.append(Point(x=x, y=y, z=z))

    while not rospy.is_shutdown():
        rospy.loginfo("Sending marker array")  # sending test

        idx += increment

        if idx > limit or idx < -limit:
            increment = -increment

        # cube
        marker = create_marker("cube", Marker.CUBE, idx, 0, 0, 1, 1, 1, 0.5, random.random(), random.random(),
                               random.random())
        marker_array.markers.append(marker)

        # cube 2
        marker = create_marker("cube2", Marker.CUBE, 0, idx, 0, 1, 1, 1, 0.5, random.random(), random.random(),
                               random.random())
        marker_array.markers.append(marker)

        # sphere
        marker = create_marker("sphere", Marker.SPHERE, 0, 0, 0, abs(idx), abs(idx), abs(idx), 0.8, 0, 1, 0)
        marker_array.markers.append(marker)

        # sphere 2
        marker = create_marker("sphere2", Marker.SPHERE, 0, 0, idx, 3, 0, abs(idx), 1, 0, 0, 1)
        marker_array.markers.append(marker)

        # text
        marker = create_marker("text", Marker.TEXT_VIEW_FACING, 3, 3, 1, 0.5, 0.5, 0.5, 1, 255, 255, 255)
        marker.text = f'value: {round(idx, 1)}'
        marker_array.markers.append(marker)

        # list of objects moving parameters
        marker_list_spheres.color.g = random.random()
        marker_list_spheres.scale.x = abs(idx / 3)
        marker_list_spheres.scale.y = abs(idx / 3)
        marker_list_spheres.scale.z = abs(idx / 3)
        marker_list_spheres.pose.position.z = idx
        marker_array.markers.append(marker_list_spheres)

        # list of cubes
        marker_list_cubes.scale.x = abs(idx / 3)
        marker_list_cubes.pose.position.z = idx
        marker_array.markers.append(marker_list_cubes)

        # publishing
        pub.publish(marker_array)
        rate.sleep()


if __name__ == '__main__':
    main()
