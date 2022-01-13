#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2, Image


class Server:
    def __init__(self):
        self.laser_data = None
        self.image_data = None

    def laser_callback(self, msg):
        self.laser_data = msg
        self.overlap()

    def image_callback(self, msg):
        self.image_data = msg

    def overlap(self):
        if self.image_data is not None and self.laser_data is not None:
            


def main():
    rospy.init_node('camera_and_lidar')
    server = Server()
    rospy.Subscriber("/scan", PointCloud2, server.laser_callback)
    rospy.Subscriber("/camera/rgb/image_raw", Image, server.image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
