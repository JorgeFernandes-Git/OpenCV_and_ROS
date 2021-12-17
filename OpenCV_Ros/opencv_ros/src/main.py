#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    cap = cv2.VideoCapture(0)

    bridge = CvBridge()

    rospy.init_node("publisher", anonymous=False)
    pub = rospy.Publisher("~image", Image, queue_size=1)

    rate = rospy.Rate(15)

    while True:
        # create img from camera
        _, img = cap.read()
        cv2.imshow('Image', img)  # Display the image

        # create the msg
        im_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")

        # publishing the msg
        rospy.loginfo(f'Sending image')
        pub.publish(im_msg)
        rate.sleep()

        # ESC to close
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    cap.release()  # free the webcam for other uses if needed
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
